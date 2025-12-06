# Migrazione simulazione multi-drone

Appunti operativi per portare la pipeline PX4 + Gazebo Classic + ROS 2 a più droni indipendenti (SITL), mantenendo missioni e logging separati e minimizzando le modifiche.

## Schema di configurazione proposto
- Introdurre `config/sim/multi.yaml` come singola fonte di verità.
- Sezione `run_ros2_system` per valori globali (world, agent di default, QoS comuni).
- Sezione `drones` con elenco di istanze: nome/namespace, pose di spawn per evitare collisioni, porte/ID unici, missione, logdir.
- Riutilizzare loop nei launcher per evitare duplicazioni (meno codice, meno bug).

Esempio (bozza):
```yaml
run_ros2_system:
  ros__parameters:
    world_file: worlds/overrack_indoor.world
    agent_cmd_default: MicroXRCEAgent udp4 -p 8888 -v 4

drones:
  - name: drone1
    namespace: /drone1
    model: iris_opt_flow
    spawn: {x: -2.5, y: -1.5, z: 0.3, yaw: 1.57}
    mission_file: config/mission_drone1.yaml
    mav_sys_id: 1
    mavlink_udp_port: 14540
    agent_cmd: "MicroXRCEAgent udp4 -p 8888 -v 4"
    px4_params: {SIM_BAT_DRAIN: 20.0}
    log_dir: data/logs/drone1
  - name: drone2
    namespace: /drone2
    model: iris_opt_flow
    spawn: {x: 2.5, y: 1.5, z: 0.3, yaw: -1.57}
    mission_file: config/mission_drone2.yaml
    mav_sys_id: 2
    mavlink_udp_port: 14550
    agent_cmd: "MicroXRCEAgent udp4 -p 8889 -v 4"
    px4_params: {SIM_BAT_DRAIN: 25.0}
    log_dir: data/logs/drone2
logging:
  base_dir: data/logs/multi_run
  metrics:
  base_dir: data/metrics/multi_run
```

Nota: lanciamo un agent Micro XRCE per ogni drone con porta dedicata; specifica l'`agent_cmd` completo nei blocchi `drones` e usa `agent_cmd_default` solo come fallback.

## Roadmap tecnica
- **Parsing config unico**
  - Estendere `run_ros2_system.sh` per leggere `config/sim/multi.yaml` (PyYAML già usato) e iterare sui blocchi `drones`.
  - Introdurre default globali + override per drone (porte/parametri PX4/missione).

- **PX4 + Gazebo multi-istanza**
  - Riutilizzare `Tools/simulation/gazebo-classic/sitl_multiple_run.sh` di PX4 o ciclare `scripts/launch_px4_gazebo.sh` con `PX4_INSTANCE` per ottenere rootfs, porte MAVLink/MicroDDS e log separati.
  - Passare per drone: `PX4_GZ_MODEL_NAME`, `PX4_GZ_MODEL_POSE` (spawn offset), `PX4_SIM_MODEL` e ID (`MAV_SYS_ID`) per evitare conflitti.
  - Mantenere un solo `gazebo`/`gzserver`, ma senza modelli baked-in nel world; lasciare a PX4 (factory plugin) lo spawn dei modelli con pose derivate da YAML.
  - Un agent Micro XRCE per drone su porta dedicata (scelta attuale); valutare un solo agent con client key per istanza solo se il rcS lo supporta.

- **Launch ROS 2 namespaced**
  - Convertire `mission.sim.launch.py` in un launcher multi-istanza: `GroupAction` + `PushRosNamespace` per ogni drone, nomi univoci dei nodi (`mission_runner_<name>`, `inspection_<name>`, ecc.).
  - Parametri per drone: `mission_file`, `gazebo_model_name`, `mavlink_url` (porta MAVLink corrispondente), `image_topic` già namespaced (`/<ns>/iris/front_camera/image_raw`).
  - Usare `params_file` condiviso per default (world bounds, QoS) e overlay per drone con `ros__parameters` inline dalla YAML multi.

- **Logging/metriche/bag**
  - Creare `data/logs/<drone>/` e `data/metrics/<drone>/` automaticamente, con timestamp run-level per separarli (`data/logs/multi_run/<ts>/drone1/...`).
  - Nominare i file `px4.out`, `agent.out`, `mission_runner.out` per drone; opzionale `ros2 bag record` per namespace con storage per-drone.

- **Missioni e separazione spaziale**
  - Un file missione per drone, con waypoints già offsettati rispetto allo spawn o con offset applicato dal mission runner (riusare logica `spawn_offset`).
  - Validare `world_bounds` per ogni drone (box non sovrapposti) e definire yaw iniziale nel YAML di spawn per evitare orientamenti convergenti.
  - Documentare posizioni di spawn nel world (heatmap) per evitare collisioni e saturazione sensori/luci.

- **Verifica**
  - Dry-run headless con due droni: controllare `ros2 topic list` per namespace puliti e heartbeat Offboard separati.
  - Confermare che ogni PX4 scriva log nel proprio folder e che `MicroXRCEAgent` veda client distinti.
  - Eseguire missioni dummy (takeoff-hover-land) per verificare clamp dei limiti e corretto uso delle missioni separate.

## Passi effettivamente realizzati (cronologia sintetica)
- Centralizzato la configurazione in `config/sim/multi.yaml` con default globali e override per drone (spawn, missione, SYSID, porte, logdir).
- Spostati i parametri single/multi in `config/sim/default.yaml` e `config/sim/multi.yaml` con fallback legacy nei launcher; le route ora vivono tutte in `config/routes/`.
- Aggiunte missioni precomputed per i due corridoi (`config/mission_drone1.yaml` con `routes/drone1_shelf_east.yaml`, `config/mission_drone2.yaml` con `routes/drone2_shelf_west.yaml`) e collegate ai rispettivi droni in `config/sim/multi.yaml`.
- Adeguato `run_ros2_system.sh` e `mission.sim.launch.py` per iterare i blocchi `drones`, creare namespace ROS separati e passare per-drone mission file/parametri/QoS senza duplicare codice.
- Allineata la risoluzione del nome modello Gazebo in `mission.sim.launch.py` ai nomi effettivi usati da `sitl_multiple_run.sh` (inclusi `iris_opt_flow` e suffix `_k`), così la Telemetry trova il modello corretto e cattura lo spawn offset invece di andare in timeout.
- Usato `sitl_multiple_run.sh` per PX4 multi-istanza: un solo `gzserver`, spawn offset dal YAML, `iris_opt_flow` supportato (airframe riusato da iris via `PX4_SIM_MODEL`), log separati per istanza.
- Stabilita l’architettura XRCE: un Micro XRCE Agent per drone (porte dedicate) con client PX4 chiave/namespace coerente (`/px4_k`), lanciati prima dei nodi ROS per evitare “EKF local position” infinito.
- Corretto il mismatch SYSID/`target_system`: Telemetry legge `system_id` da `VehicleStatus` e SetpointPublisher lo usa per indirizzare i `VehicleCommand` (fallback a `vehicle_id` con warning), così arming/offboard funzionano per tutte le istanze anche se `sitl_multiple_run` applica l’offset `mavlink_id=1+N`.
- Documentati i problemi ricorrenti (plugin Gazebo ROS, modelli supportati, naming dei topic PX4) e la procedura di verifica con hover/land per ogni drone.

## Problemi incontrati e fix
- Plugin Gazebo ROS non trovati (libgazebo_ros_api_plugin.so, libgazebo_ros_factory.so): esporta `GAZEBO_PLUGIN_PATH` includendo `/opt/ros/$ROS_DISTRO/lib` anche nel launcher multi.
- `iris_opt_flow` ora supportato da `sitl_multiple_run.sh`: usa l’SDF dedicato e riutilizza l’airframe di iris (`PX4_SIM_MODEL=gazebo-classic_iris`) se non definito diversamente.
- Attesa infinita su “Waiting for EKF local position…” in multi-istanza: senza micrortps_client/Agent per ogni PX4 non arrivano i topic `/fmu/out/*`; avvia un client per istanza con porte dedicate e un Agent per drone (namespace corretto) prima di lanciare i nodi ROS.
- Topics PX4 namespacizzati: se il bridge pubblica solo `/fmu/...`, lascia `px4_namespace` vuoto nei nodi; il namespacing PX4 va riattivato solo quando anche il bridge usa `/droneX/fmu/...`.
- Architettura agent XRCE: usare un Micro XRCE-DDS Agent per drone su porta dedicata (es. 8888, 8889, ...). `sitl_multiple_run.sh` imposta `px4_instance`, `UXRCE_DDS_KEY` e il namespace ROS (`/px4_1`, `/px4_2`, …); i nodi ROS vanno lanciati nello stesso namespace (`__ns:=/px4_k`) mantenendo i topic PX4 su `/fmu/in|out/*`. Così si evita la collisione tra client_key e si ricevono correttamente i topic di ogni istanza.
- `px4_param_setter` RuntimeError in shutdown (Context must be initialized...): dopo l’applicazione parametri il crash è benigno; se dà fastidio, eseguirlo solo quando `px4_params` è presente o gestire l’uscita senza `rclpy.shutdown()` forzato.
- Mismatch MAV_SYS_ID vs `target_system`: `sitl_multiple_run.sh` genera `--mavlink_id $((1+N))` con `N` che parte da 1, quindi le istanze di default hanno SYSID 2,3,... mentre il mission runner inviava `VehicleCommand` a 1,2; PX4 ignorava arming/offboard sul drone “sbagliato”. Fix: Telemetry legge `system_id` da `VehicleStatus` e SetpointPublisher usa quel valore per `target_system`/`source_system` (fallback a `vehicle_id` con warning), così i comandi seguono sempre il SYSID reale.

- **Problemi**
  - `sitl_multiple_run.sh` cercava i template solo sotto `PX4-Autopilot/Tools/.../models/<model>/<model>.sdf.jinja`, ignorando `GAZEBO_MODEL_PATH`: il nostro `models/iris_opt_flow/iris_opt_flow.sdf` (e le aggiunte camera/torcia) veniva ignorato e lo spawn falliva con `TemplateNotFound`. Fix: aggiunta funzione di lookup sul `GAZEBO_MODEL_PATH` e template Jinja nel repo (`models/iris_opt_flow/iris_opt_flow.sdf.jinja`). Snippet della patch:
    ```bash
    find_model_template() {
      # cerca <model>.sdf.jinja in GAZEBO_MODEL_PATH, altrimenti fallback PX4
      IFS=':' read -r -a model_paths <<< "${GAZEBO_MODEL_PATH:-}"
      for path in "${model_paths[@]}"; do
        if [[ -f "${path}/${search_model}/${search_model}.sdf.jinja" ]]; then
          echo "${path}/${search_model}/${search_model}.sdf.jinja"; return 0; fi
      done
      fallback="${src_path}/Tools/.../models/${search_model}/${search_model}.sdf.jinja"
      [[ -f "$fallback" ]] && echo "$fallback"
    }
    ```
    Così `iris_opt_flow` usa il template locale e `jinja_gen.py` può inserire porte/ID per ogni istanza.
  - `jinja_gen.py` richiede `env_dir` coerente con il template: prima era sempre impostato al path PX4 e i template esterni fallivano comunque. Fix: lo script ora passa come `env_dir` la cartella del template (es. `models/iris_opt_flow`) quando usa modelli fuori da PX4.
  - multi run conosce solo irisi, plane, standard_vtol, rover, r1_rover, typhoon_h480 oltre a `iris_opt_flow`; per altri modelli non previsti occorre estendere la whitelist.
  
- TODO
-  se servono modelli custom oltre a `iris_opt_flow`, valutare duplicato dell’iris con estensioni e aggiunta alla whitelist di `sitl_multiple_run.sh`
-  Dobbiamo mandare i setting px4 in modo elettivo in base al drone, cambaire la classe (modifica) e ovviametne anche il .yalm
-  testare se funziona davvero il lalto della connessione!!!
- Elimnar i file in param nel workspace ros2 li abbimao in config
- Capire se possiamo non piu utilizzare la logica di drone singolo dopo aver generalizzato
- inspection node, serve davvero? ha senso tenerlo?
- debug framse a che serve 
- precomputed? teniamo?
- torch controller non utilizzabile probabilemte con mult_sitl 
