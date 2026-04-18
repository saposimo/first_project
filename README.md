# first_project

Package ROS2 con:

- nodo `odometer`
- nodo `tf_error`
- launch file `first_project.launch.py`
- configurazione RViz `rviz/first_project.rviz`

Questi comandi assumono:

- workspace in `~/colcon_ws`
- ROS 2 Humble installato
- `rviz2` gia' funzionante
- rosbag di test in `~/bags/rosbag2_2026_04_08-16_38_55`

## Build

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to first_project --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

Note rapide:

- `--packages-up-to first_project` builda `first_project` e le dipendenze necessarie, incluso `bunker_msgs`
- `source install/setup.bash` va rieseguito in ogni nuovo terminale prima di lanciare nodi o comandi ROS

## Launch del pacchetto

Avvio senza RViz:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch first_project first_project.launch.py use_rviz:=false use_sim_time:=true
```

Avvio con RViz:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch first_project first_project.launch.py use_rviz:=true use_sim_time:=true
```

Parametri utili:

- `use_rviz:=true` apre anche RViz
- `use_rviz:=false` lancia solo i nodi
- `use_sim_time:=true` va usato quando riproduci un rosbag
- `use_sim_time:=false` va bene per esecuzione normale

## Avvio dei nodi singolarmente

Terminale 1, `odometer`:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run first_project odometer --ros-args -p use_sim_time:=true
```

Terminale 2, `tf_error`:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run first_project tf_error --ros-args -p use_sim_time:=true
```

## Rosbag di test

Info del bag:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag info ~/bags/rosbag2_2026_04_08-16_38_55
```

Riproduzione in loop:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play --clock 20 --rate 0.5 --loop ~/bags/rosbag2_2026_04_08-16_38_55
```

Riproduzione senza loop:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play --clock 20 --rate 0.5 ~/bags/rosbag2_2026_04_08-16_38_55
```

Significato dei flag principali:

- `--clock` pubblica il tempo simulato ROS
- `--rate 0.5` riproduce il bag a meta' velocita'
- `--loop` riavvia automaticamente la riproduzione

## Debug rapido

Leggere l'odometria calcolata:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /project_odom
```

Leggere l'errore TF:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /tf_error_msg
```

Verifiche rapide:

```bash
ros2 node list
ros2 topic list
ros2 service list
```

Controllo del clock simulato:

```bash
ros2 topic echo /clock
```

Controllo dei frame TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo odom base_link2
```

Reset odometria:

```bash
ros2 service call /reset std_srvs/srv/Empty "{}"
```

## Sequenza minima consigliata

Terminale 1:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to first_project --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 launch first_project first_project.launch.py use_rviz:=false use_sim_time:=true
```

Terminale 2:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play --clock 20 --rate 0.5 --loop ~/bags/rosbag2_2026_04_08-16_38_55
```

Terminale 3 opzionale:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /tf_error_msg
```
