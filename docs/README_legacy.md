# Over-Rack Inventory Scan (PX4 + ROS 2)

## Overview
Over-Rack Scan è un ambiente di test per validare missioni PX4 indoor con ROS 2 e Gazebo Classic. Il progetto fornisce uno stack riproducibile basato su PX4 SITL (release 1.14.4), il bridge DDS Micro XRCE-DDS Agent e un nodo ROS 2 che pubblica setpoint offboard partendo da un piano missione YAML. L'indoor world replica una corsia di scaffali con marker barcode per verificare pipeline di scansione e copertura. Gli script Bash orchestrano l'avvio completo: build del workspace, estensione del `GAZEBO_MODEL_PATH`, lancio di PX4, avvio dell'agent e del mission runner. Log e asset vengono salvati nella repo per velocizzare il debug e analizzare metriche (hover drift, copertura, temperature CPU). Un set di utility opzionali supporta la visione artificiale e la manutenzione dei modelli Gazebo.

## Struttura del repo
```text
overrack-scan/
├── scripts/
│   · Orchestratori (`run_ros2_system.sh`) e wrapper PX4/Gazebo (`launch_px4_gazebo.sh`, `launch_gazebo.sh`).
│   · Tool di supporto (patch modello Iris, setup venv) e pipeline opzionali (`run_vision.py`).
│   · Script legacy/manuali (es. `run_manual_like.sh`, demo FSM) e asset di test (`data/`, `test_data/`).
│   · Dipende da `PX4_DIR`, `config/mission.yaml`, `worlds/overrack_indoor.world` e Micro XRCE Agent.
│   · Stato: baseline attivo; alcune utility segnate come legacy o zombie nel report dedicato.
├── ros2_ws/src/
│   · Workspace ROS 2 isolato con `overrack_mission` (mission runner), `px4_msgs` e reference `px4_ros_com`.
│   · Build gestita da `run_ros2_system.sh` via `colcon --packages-select px4_msgs overrack_mission`.
│   · Sorgente principale per il nodo `mission_runner` eseguito in runtime.
│   · Integra i messaggi PX4 ufficiali (DDS) e QoS coerenti con SITL.
│   · Stato: attivo (mission runner), reference (px4_ros_com) non compilato di default.
├── config/
│   · `mission.yaml` definisce waypoints, quota, hover e soglie (copertura, drift, temperatura).
│   · `px4_sitl.params` è un placeholder vuoto (caricamento disabilitato per evitare regressioni).
│   · Caricato da `overrack_mission` e, in passato, dal mission runner legacy in `src/`.
│   · Modificare qui per personalizzare missioni e soglie di allerta.
│   · Stato: `mission.yaml` attivo; param file in quarantena finché non validato.
├── models/
│   · Modelli Gazebo custom: scaffali (`overrack_shelf`), marker (`scan_marker`), barcode board.
│   · Usati dal mondo SDF per riprodurre la corsia indoor.
│   · Aggiunti all'ambiente tramite `GAZEBO_MODEL_PATH` in `launch_px4_gazebo.sh`.
│   · Compatibili con Gazebo Classic 11 e con il modello PX4 `iris_opt_flow`.
│   · Stato: attivo (sincronizzato con il world corrente).
├── worlds/
│   · `overrack_indoor.world`: SDF principale con stanza, scaffali, camera overhead e marker.
│   · Referenzia i modelli della cartella `models/` e salva frame in `data/images/downcam`.
│   · Utilizzato dagli script di lancio per PX4 e per avviare Gazebo standalone.
│   · Modificare waypoint o asset richiede aggiornare sia SDF sia `config/mission.yaml`.
│   · Stato: attivo.
├── data/
│   · Raccolta output runtime (`logs/`, `images/`, `state/`) generati dagli script.
│   · I log includono PX4, Micro XRCE Agent e mission runner; le immagini arrivano dalla camera SDF.
│   · Directory svuotate automaticamente dagli script solo se necessario (non cancellare a mano).
│   · Utile per troubleshooting e per alimentare la pipeline vision.
│   · Stato: generata a runtime.
├── src/
│   · Codice legacy del mission runner (FSM completa) e demo (`run_demo.py`, `run_visual_demo.py`).
│   · Non integrato nel workspace ROS 2; richiede esecuzione manuale (`python -m src.main`).
│   · Almomento alcune demo sono rotte (classe `MissionFSM` rimossa).
│   · Tenere solo se si intende recuperare la FSM avanzata o pipeline CSV personalizzate.
│   · Stato: legacy/sperimentale (vedi report structure_review).
└── docs/
    · Documentazione aggiuntiva: specifiche, note di migrazione, review struttura.
    · `docs/structure_review.md` contiene l'analisi dettagliata + zombie.
    · Aggiornare qui qualsiasi modifica architetturale o procedure condivise.
    · Mantenere PDF e note legacy separati dal README principale.
    · Stato: attivo con contenuti misti (alcuni da aggiornare).
```

## Prerequisiti
- Ubuntu 22.04 LTS (desktop o con forwarding grafico) e Python 3.10.
- ROS 2 Humble Foxy (desktop install + `colcon` e `rosdep`).
- PX4-Autopilot v1.14.4 già clonato e buildato almeno una volta (`make px4_sitl_default`).
- Micro XRCE-DDS Agent v2.4.3 (binary `MicroXRCEAgent` nel `PATH` o build locale).
- Gazebo Classic 11 con pacchetti `gazebo`, `gzserver`, `gzclient` e dipendenze PX4.
- Dipendenze opzionali per visione: `opencv-python`, `pyzbar`, `zxing-cpp`, `mavsdk`, `psutil` (installabili via `pip install -r requirements.txt`).

## Setup rapido
- **Preparare PX4**
  ```bash
  git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4/PX4-Autopilot
  cd ~/PX4/PX4-Autopilot
  git checkout v1.14.4
  make px4_sitl_default
  ```
  > Imposta `PX4_DIR` verso la cartella PX4 quando lanci gli script (`export PX4_DIR=~/PX4/PX4-Autopilot`).

- **Installare dipendenze ROS 2 e colcon**
  ```bash
  sudo apt update
  sudo apt install ros-humble-desktop python3-colcon-common-extensions ros-humble-micro-xrce-dds-agent
  ```

- **Build del workspace ROS 2** (sept già fatta dagli script, ma utile per debug)
  ```bash
  cd ~/projects/overrack-scan/ros2_ws
  colcon build --symlink-install --packages-select px4_msgs overrack_mission
  source install/setup.bash
  ```

- **Variabili d'ambiente suggerite**
  ```bash
  export PX4_DIR=~/PX4/PX4-Autopilot
  export PROJECT_ROOT=~/projects/overrack-scan
  # opzionale: aggiungi i modelli al path (gli script lo fanno automaticamente)
  export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$PROJECT_ROOT/models:${GAZEBO_MODEL_PATH:-}"
  export PX4_SIM_MODEL=iris_opt_flow   # evita override involontari
  ```
  > `px4_sitl.params` è disabilitato: non esportare `PX4_RC_PARAMS_FILE` finché il file non viene popolato con valori verificati.

## Run
- **Stack automatico**
  ```bash
  cd ~/projects/overrack-scan
  PX4_DIR=~/PX4/PX4-Autopilot \
  scripts/run_ros2_system.sh --gui \  # usa --headless per disabilitare la GUI
    --world worlds/overrack_indoor.world \
    --mission config/mission.yaml
  ```
  - Avvia PX4 SITL con modello `gazebo-classic_iris_opt_flow` (vedi `scripts/launch_px4_gazebo.sh:78-103`).
  - Estende `GAZEBO_MODEL_PATH` con i modelli PX4 e quelli locali, mantenendo l'ordine corretto.
  - Cerca o lancia `MicroXRCEAgent udp4 -p 8888 -v 6` e aspetta i topic `/fmu/out/*` (`scripts/run_ros2_system.sh:182-195`).
  - Esegue `ros2 run overrack_mission mission_runner` con il mission file selezionato.

- **Script manual-like (legacy)**
  ```bash
  # run_manual_like.sh è stato rimosso; usa scripts/run_ros2_system.sh
  # usa scripts/stop_manual_like.sh per killare PX4/Gazebo/Agent se necessario
  ```

- **Verifica**
  ```bash
  source ros2_ws/install/setup.bash
  ros2 topic list | grep /fmu
  tail -f data/logs/px4_sitl_default.out
  ```

## Troubleshooting
- **Ambiente & PATH**
  - `ros2` non vede i topic → `source ros2_ws/install/setup.bash` e evita di mischiare `/opt/ros/humble/setup.bash` se non necessario.
  - Gli script devono fare `source "$ROS2_WS/install/setup.bash"` dopo aver definito `ROS2_WS` (vedi `scripts/run_ros2_system.sh`).

- **Micro XRCE-DDS Agent**
  - PX4 non pinga l'agent → avvia `MicroXRCEAgent udp4 -p 8888 -v 6` prima del mission runner e controlla i log `data/logs/micro_xrce_agent.out`.
  - Su SSH/ambienti headless avvialo nello stesso terminale o tramite `tmux` (evita `gnome-terminal`).

- **PX4 + Gazebo**
  - Modello errato (compare `iris`) → forza `PX4_SIM_MODEL=iris_opt_flow` e assicurati che `models/` non contenga `iris*` (`launch_px4_gazebo.sh:84-94`).
  - Drone spawnato in ostacolo → verifica il `<pose>` nel world o lascia gestire lo spawn a PX4 impostando l'ambiente vuoto.
  - "PX4 server already running" → `pkill -f "px4|gzserver|gzclient|MicroXRCEAgent"`.

- **Parametri & preflight**
  - Arming bloccato (IMU/GPS) → modello `iris_opt_flow` fornisce i plugin richiesti; per indoor abilita `COM_ARM_WO_GPS=1` direttamente dalla console PX4.
  - File `.params` → se vuoi sperimentare, duplica `config/px4_sitl.params` ma non riattivare il loader finché non è validato (vedi commento in `launch_px4_gazebo.sh:58-65`).

- **Tempi di start**
  - Topic `/fmu/out` assenti → attendi 10-15s dopo il bootstrap PX4 prima di lanciare ROS 2 o usa il `wait_for_topic` integrato.
  - Mission runner avvia troppo presto → controlla `px4_sitl_default.out` per "Ready for takeoff".

- **Visione & Log**
  - Immagini mancanti → la camera overhead salva in `data/images/downcam`; verifica permessi della cartella e spazio disco.
  - Pipeline `run_vision.py` → richiede librerie opzionali; avviala con `python scripts/run_vision.py --watch data/images/downcam`.

- **Shutdown pulito**
  ```bash
  pkill -f mission_runner
  pkill -f MicroXRCEAgent
  pkill -f px4
  pkill -f gzserver
  pkill -f gzclient
  ```

## Contributi & Convenzioni
1. **Nuovi script**: mantieni il prefisso descrittivo (`run_`, `launch_`, `fix_`) e documenta l'utilizzo nel README e in `docs/structure_review.md`. Ricorda di gestire `PX4_DIR`, `ROS2_WS` e le variabili d'ambiente come negli script esistenti.
2. **Modelli & world**: aggiungi nuovi asset sotto `models/<name>` con `model.config` + `model.sdf` e aggiorna il world o crea un file `.world` dedicato. Specifica nel README quale world/modello usare.
3. **Parametri PX4**: lavora su copie di `px4_sitl.params`; non abilitare il caricamento automatico finché non sono testati. Documenta ogni parametro rilevante in `docs/structure_review.md` o in una nota dedicata.
4. **Mission runner ROS 2**: integra nuove funzionalità dentro `ros2_ws/src/overrack_mission`. Aggiorna `package.xml`, `setup.py` e aggiungi test/launch file se introdotti topic aggiuntivi.
5. **Documentazione**: ogni modifica sostanziale (nuovo script, variazione del flow di lancio, cambio del mondo) deve essere riflessa sia nel README sia nel report in `docs/structure_review.md`. Mantieni `docs/ROS2_MIGRATION.md` allineato quando cambi architettura bridge.

Per i dettagli completi sulla struttura e sui componenti legacy consulta `docs/structure_review.md` aggiornato.
