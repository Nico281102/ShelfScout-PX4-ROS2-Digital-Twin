# YAML parsing condiviso per gli script di launch

I file di launch e gli script shell ora leggono i parametri dal modulo Python
`overrack_mission.param_utils` (`ros2_ws/src/overrack_mission/overrack_mission/param_utils.py`).
Lo stesso codice fornisce sia le utility usate dai launch ROS2 sia la piccola
CLI che emette variabili per gli script bash, così non duplichiamo parsing
sparse in shell e Python.

## Uso rapido (shell)

```bash
# Con PYTHONPATH puntato ai sorgenti ROS2
PYTHONPATH="$(pwd)/ros2_ws/src" \
  python3 -m overrack_mission.param_utils --file config/sim/default.yaml defaults --format shell
# YAML_MISSION_DEFAULT='config/mission_precomputed.yaml'
# YAML_WORLD_DEFAULT='worlds/overrack_indoor.world'
# YAML_AGENT_DEFAULT='MicroXRCEAgent udp4 -p 8888 -v 6'
# YAML_DRONES_COUNT=0

python3 -m overrack_mission.param_utils --file config/sim/default.yaml drones --format shell
# DRONE_COUNT=1
# DRONE_NAME[0]='drone1'
# DRONE_NS[0]='drone1'
# ...
```

Gli script `scripts/run_ros2_system.sh` e `scripts/launch_px4_gazebo_multi.sh`
chiamano direttamente questa CLI per popolare i valori di default e gli array
di droni, evitando frammenti Python embedded.

## Ordine di precedenza (override)

Il launch `mission.sim.launch.py` applica queste regole:
- Parametri globali: valori sotto `mission_runner.ros__parameters`, `inspection_node.ros__parameters`, `torch_controller.ros__parameters` sono la base per tutti i droni.
- Per-drone: se un drone in `drones_yaml` contiene una sezione `mission_runner`/`inspection_node`/`torch_controller`, quei campi sovrascrivono i globali per quel drone.
- Missione: `mission_file` è (in ordine) `mission_runner` per-drone → `mission_file` del drone → argomento `mission_file` passato via `ros2 launch` → default globale in `mission_runner.ros__parameters` → default in `run_ros2_system.ros__parameters.mission_file`.
- Mavlink: `mavlink_url` per-drone → `mavlink_udp_port` per-drone (tramutato in `udp://:<port>`) → default in `px4_param_setter.ros__parameters.mavlink_url`.

Questo garantisce che un blocco per-drone possa specializzare i valori senza duplicare il resto della configurazione, mentre i default globali restano validi quando le sezioni specifiche non sono presenti.

## Uso nei launch ROS2

Il file `overrack_mission/launch/mission.sim.launch.py` importa le stesse
funzioni (`load_config`, `drones_from_config`, `default_mission`,
`default_model`, `default_mavlink_url`, `section_params`, `resolve_gazebo_model_name`)
per costruire i nodi per ogni drone senza duplicare helper locali.

## Estendere il parsing

Se aggiungiamo nuovi campi nel YAML dei parametri (ad es. nuove chiavi in
`run_ros2_system.ros__parameters` o nei blocchi dei droni), aggiungere la
logica in `param_utils.py` e consumarla dagli script/launch. In questo modo il
comportamento rimane allineato e testabile da un unico punto.
