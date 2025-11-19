#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace gazebo
{

class TorchLightPlugin : public ModelPlugin
{
public:
  TorchLightPlugin() = default;

  ~TorchLightPlugin() override
  {
    if (render_conn_) render_conn_.reset();
    if (executor_) executor_->cancel();
    if (ros_thread_.joinable()) ros_thread_.join();
    if (context_) context_->shutdown("TorchLightPlugin shutdown");
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    light_name_    = sdf->Get<std::string>("light_name", "torch_light").first;
    command_topic_ = sdf->Get<std::string>("topic", "/overrack/torch_enable").first;

    // ---------------- ROS2 ----------------
    context_ = std::make_shared<rclcpp::Context>();
    int argc = 0; char **argv = nullptr;
    context_->init(argc, argv);

    rclcpp::ExecutorOptions opt;
    opt.context = context_;

    ros_node_ = std::make_shared<rclcpp::Node>(
        "torch_light_plugin",
        rclcpp::NodeOptions().context(context_));

    RCLCPP_INFO(ros_node_->get_logger(),
                "[Load] Plugin attached to model: %s",
                model_->GetName().c_str());

    // Subscriber (ATTENZIONE: non passiamo atomic a RCLCPP_INFO)
    sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      command_topic_,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        const bool value = msg->data;
        bool prev = desired_on_.load();

        RCLCPP_INFO(ros_node_->get_logger(),
                    "[ROS MSG] Received: %s (prev=%d)",
                    value ? "TRUE" : "FALSE",
                    static_cast<int>(prev));

        desired_on_.store(value);
      });

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(opt);
    executor_->add_node(ros_node_);
    ros_thread_ = std::thread([this](){ executor_->spin(); });

    // ---------------- Gazebo transport ----------------
    gz_node_ = transport::NodePtr(new transport::Node());
    gz_node_->Init(model_->GetWorld()->Name());        // es. "overrack_indoor"
    light_pub_ = gz_node_->Advertise<msgs::Light>("~/light/modify");

    // Hook grafico
    render_conn_ = event::Events::ConnectPreRender(
        std::bind(&TorchLightPlugin::OnPreRender, this));

    RCLCPP_INFO(ros_node_->get_logger(),
                "[Load] TorchLightPlugin initialized (light=%s, topic=%s)",
                light_name_.c_str(), command_topic_.c_str());
  }

private:

  void OnPreRender()
  {
    if (!scene_)
    {
      scene_ = rendering::get_scene();

      if (!scene_)
      {
        RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 2000,
                             "[OnPreRender] Scene not ready yet");
        return;
      }

      RCLCPP_INFO(ros_node_->get_logger(),
                  "[OnPreRender] Scene detected: %s", scene_->Name().c_str());
    }

    if (!light_)
    {
      FindLight();
      if (!light_) return;

      scoped_light_name_ = light_->Name();  // es. "iris_opt_flow::torch_link::torch_light"

      RCLCPP_INFO(ros_node_->get_logger(),
                  "[OnPreRender] Binding successful: %s",
                  scoped_light_name_.c_str());

      // Spegni all’avvio
      ApplyLightState(false);
      current_on_ = false;
    }

    bool desired = desired_on_.load();

    if (desired != current_on_)
    {
      RCLCPP_INFO(ros_node_->get_logger(),
                  "[STATE CHANGE] desired=%d current=%d",
                  static_cast<int>(desired),
                  static_cast<int>(current_on_));

      ApplyLightState(desired);
      current_on_ = desired;
    }
  }

  void FindLight()
  {
    RCLCPP_INFO(ros_node_->get_logger(),
                "[FindLight] Searching for light '%s'...", light_name_.c_str());

    // 1) nome semplice
    light_ = scene_->LightByName(light_name_);
    if (light_)
    {
      RCLCPP_INFO(ros_node_->get_logger(),
                  "[FindLight] Found (simple): %s", light_->Name().c_str());
      return;
    }

    // 2) nome scoped
    if (model_)
    {
      std::string scoped = model_->GetScopedName(true) + "::" + light_name_;
      RCLCPP_INFO(ros_node_->get_logger(),
                  "[FindLight] Trying scoped name: %s", scoped.c_str());

      light_ = scene_->LightByName(scoped);
      if (light_)
      {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "[FindLight] Found (scoped): %s", light_->Name().c_str());
        return;
      }
    }

    // 3) brute force
    RCLCPP_INFO(ros_node_->get_logger(),
                "[FindLight] Trying brute force search on %u lights...",
                scene_->LightCount());

    const std::string suffix = "::" + light_name_;
    for (unsigned int i = 0; i < scene_->LightCount(); ++i)
    {
      auto cand = scene_->LightByIndex(i);
      if (!cand) continue;

      std::string nm = cand->Name();
      RCLCPP_INFO(ros_node_->get_logger(), "[FindLight] Candidate: %s", nm.c_str());

      if (nm.size() >= suffix.size() &&
          nm.compare(nm.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        light_ = cand;
        break;
      }
    }

    if (!light_)
    {
      RCLCPP_WARN(ros_node_->get_logger(),
                  "[FindLight] FAILED to find '%s'", light_name_.c_str());
    }
  }

  void ApplyLightState(bool on)
  {
    if (!light_) return;

    RCLCPP_INFO(ros_node_->get_logger(),
                "[ApplyLightState] ramo %s", on ? "ON" : "OFF");

    // --- lato client (GUI) ---
    if (on)
    {
      light_->SetDiffuseColor(ignition::math::Color(1, 0.95, 0.8, 1));
      light_->SetSpecularColor(ignition::math::Color(0.3, 0.3, 0.3, 1));
      light_->SetAttenuation(1.0, 0.1, 0.01);
      light_->SetRange(20.0);
      light_->SetVisible(true);
      light_->SetCastShadows(true);
    }
    else
    {
      light_->SetDiffuseColor(ignition::math::Color(0, 0, 0, 1));
      light_->SetSpecularColor(ignition::math::Color(0, 0, 0, 1));
      light_->SetAttenuation(0.0, 1.0, 1.0);
      light_->SetRange(0.01);
      light_->SetVisible(false);
      light_->SetCastShadows(false);
    }

    // --- lato server (fisica) via ~/light/modify ---
    if (light_pub_ && !scoped_light_name_.empty())
    {
      msgs::Light msg;
      msg.set_name(scoped_light_name_);
      msg.set_cast_shadows(on);
      msg.set_range(on ? 20.0 : 0.01);

      auto *diff = msg.mutable_diffuse();
      if (on)
      {
        diff->set_r(1.0);
        diff->set_g(0.95);
        diff->set_b(0.8);
        diff->set_a(1.0);
      }
      else
      {
        diff->set_r(0.0);
        diff->set_g(0.0);
        diff->set_b(0.0);
        diff->set_a(1.0);
      }

      light_pub_->Publish(msg);
    }
  }

  // Members
  physics::ModelPtr model_;
  rendering::ScenePtr scene_;
  rendering::LightPtr light_;
  event::ConnectionPtr render_conn_;

  std::string light_name_;
  std::string command_topic_;
  std::string scoped_light_name_;

  std::atomic<bool> desired_on_{false};
  bool current_on_{false};

  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread ros_thread_;

  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr light_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(TorchLightPlugin)

} // namespace gazebo
