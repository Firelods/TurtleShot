# TurtleShot - Quick Start Guide

Guide rapide pour lancer la simulation Gazebo Fortress avec le robot TurtleBot3 + Catapaf.

## 🚀 Démarrage Rapide

### 1. Build (si pas déjà fait)

```bash
cd ~/catapaf_ws/TurtleShot
source /opt/ros/humble/setup.bash
colcon build --packages-select catapaf_description catapaf_gazebo
source install/setup.bash
```

### 2. Lancer la Simulation

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

### 3. Contrôler le Robot

**Terminal 1 - Simulation** (déjà lancée)

**Terminal 2 - Contrôleur du bras:**

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash
ros2 run catapaf_gazebo catapaf_arm_controller
```

**Terminal 3 - Téléopération du robot:**

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 4 - Lancer la catapulte:**

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger
```

## 🎯 Tester la Catapulte

### 1. Spawner une balle

```bash
ros2 run ros_gz_sim create \
  -name ball_1 \
  -file $(ros2 pkg prefix catapaf_gazebo)/share/catapaf_gazebo/models/ball/model.sdf \
  -x -0.05 -y -0.09 -z 0.15
```

### 2. Positionner le bras

```bash
# Position de repos (chargé)
ros2 topic pub /catapaf_arm/position std_msgs/msg/Float64 "data: -1.2" --once

# Attendre que la balle soit stable
sleep 2
```

### 3. Lancer !

```bash
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger
```

### 4. Reset

```bash
ros2 service call /catapaf_arm/reset std_srvs/srv/Trigger
```

## 📊 Vérifier les Topics

```bash
# Liste tous les topics ROS
ros2 topic list

# Voir les données du LiDAR
ros2 topic echo /scan

# Voir l'odométrie
ros2 topic echo /odom

# Voir la position des joints
ros2 topic echo /joint_states

# Voir l'image de la caméra (dans RViz ou rqt_image_view)
ros2 run rqt_image_view rqt_image_view
```

## 🎮 Commandes du Robot

### Déplacement manuel via topics

```bash
# Avancer
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Tourner à gauche
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Arrêter
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

### Position du bras manuel

```bash
# Positions en radians (limites: -1.5 à 1.3)

# Repos (balle chargée)
ros2 topic pub /catapaf_arm/position std_msgs/msg/Float64 "data: -1.2"

# Position intermédiaire
ros2 topic pub /catapaf_arm/position std_msgs/msg/Float64 "data: 0.0"

# Lancé (bras levé)
ros2 topic pub /catapaf_arm/position std_msgs/msg/Float64 "data: 1.0"
```

## 🔧 Options de Lancement

### Position de spawn personnalisée

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py x_pose:=2.0 y_pose:=1.0
```

### Mode headless (sans GUI)

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py gui:=false
```

### Monde personnalisé

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py \
  world:=/chemin/vers/votre/monde.sdf
```

## 📁 Structure des Fichiers Importants

```
TurtleShot/
├── src/
│   ├── catapaf_description/          # Description URDF du robot
│   │   ├── urdf/
│   │   │   ├── turtlebot_with_catapaf_gz.urdf.xacro  # Robot complet
│   │   │   └── catapaf.urdf.xacro                     # Catapulte seule
│   │   └── meshes/                                    # Fichiers 3D (STL)
│   │
│   └── catapaf_gazebo/                # Simulation Gazebo
│       ├── launch/
│       │   └── gz_simulation.launch.py                # Launch principal
│       ├── worlds/
│       │   └── turtlebot_world.sdf                    # Monde avec obstacles
│       ├── models/
│       │   └── ball/                                  # Modèle de balle
│       ├── config/
│       │   └── catapaf_bridge.yaml                    # Bridge ROS-Gazebo
│       └── catapaf_gazebo/
│           └── catapaf_arm_controller.py              # Contrôleur du bras
│
├── test_gazebo_fortress.sh            # Script de test
├── MIGRATION_GUIDE.md                 # Guide de migration détaillé
└── QUICKSTART.md                      # Ce fichier
```

## 🐛 Problèmes Courants

### Les meshes ne se chargent pas

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix catapaf_description)/share/catapaf_description/meshes
```

### Le robot ne bouge pas

1. Vérifier que cmd_vel est publié: `ros2 topic echo /cmd_vel`
2. Vérifier le bridge: `ros2 topic list | grep cmd_vel`
3. Relancer la simulation

### La balle traverse le sol

C'est normal au début, la physique se stabilise en ~1 seconde.

### Le bras ne répond pas

1. Vérifier que le contrôleur tourne: `ros2 node list | grep catapaf`
2. Vérifier le service: `ros2 service list | grep catapaf`
3. Relancer le contrôleur: `ros2 run catapaf_gazebo catapaf_arm_controller`

## 🎓 Prochaines Étapes

1. **Explorer le monde**
   - Utiliser teleop pour naviguer
   - Tester l'évitement d'obstacles avec le LiDAR

2. **Tester la catapulte**
   - Spawner plusieurs balles
   - Essayer différentes positions de bras
   - Viser la cible jaune (à x=4, y=0)

3. **Développer des comportements**
   - Navigation autonome
   - Visée automatique avec la caméra
   - Détection de cibles

4. **Lire la documentation**
   - [src/catapaf_gazebo/README.md](src/catapaf_gazebo/README.md) - Documentation complète
   - [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - Détails techniques
   - [CLAUDE.md](CLAUDE.md) - Notes de développement

## 📞 Support

Si vous rencontrez des problèmes :

1. Vérifier les logs: `ros2 topic echo /rosout`
2. Tester les packages: `./test_gazebo_fortress.sh`
3. Consulter la documentation Gazebo Fortress: https://gazebosim.org/docs/fortress

---

**Bon amusement avec TurtleShot ! 🚀🎯**
