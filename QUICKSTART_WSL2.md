# TurtleShot - Guide Rapide pour WSL2

## ⚠️ Important pour WSL2

Gazebo Fortress avec GUI peut crasher dans WSL2 à cause de problèmes avec OGRE2 et OpenGL.

**Solution recommandée : Mode Headless** (sans GUI)

## 🚀 Lancement Rapide (Mode Headless)

### Méthode 1 : Script rapide

```bash
cd ~/catapaf_ws/TurtleShot
bash launch_headless.sh
```

### Méthode 2 : Commande complète

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash
ros2 launch catapaf_gazebo gz_simulation_headless.launch.py
```

## 📊 Vérification

Dans un autre terminal :

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash

# Liste des topics
ros2 topic list

# Test du LiDAR
ros2 topic echo /scan --once

# Test de l'odométrie
ros2 topic echo /odom --once
```

**Résultat attendu :** Les topics doivent afficher des données !

## 🎮 Contrôle du Robot

### Terminal 1 : Simulation (déjà lancée)

### Terminal 2 : Contrôleur du bras

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash
ros2 run catapaf_gazebo catapaf_arm_controller
```

### Terminal 3 : Téléopération

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Terminal 4 : Commandes

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash

# Lancer la catapulte
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger

# Avancer
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once

# Tourner
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once
```

## 👁️ Visualisation avec RViz2

```bash
# Dans un nouveau terminal
source install/setup.bash
rviz2
```

**Configuration RViz2 :**

1. **Fixed Frame** : Changer de `map` à `odom`
2. **Add** (bouton en bas à gauche) :
   - **RobotModel** - Pour voir le robot
   - **LaserScan** - Topic : `/scan` - Pour voir le LiDAR
   - **TF** - Pour voir les transformées
   - **Odometry** - Topic : `/odom` - Pour voir la trajectoire

## 🎯 Spawner une Balle

```bash
ros2 run ros_gz_sim create \
  -name ball_1 \
  -file $(ros2 pkg prefix catapaf_gazebo)/share/catapaf_gazebo/models/ball/model.sdf \
  -x -0.05 -y -0.09 -z 0.15
```

## ⚙️ Si vous voulez quand même essayer le mode GUI

**Attention :** Peut crasher dans WSL2 !

```bash
# Essayer avec OGRE (au lieu de OGRE2)
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

Si ça crash, voir [WSL2_TROUBLESHOOTING.md](WSL2_TROUBLESHOOTING.md) pour plus de solutions.

## 🔧 Dépannage Rapide

### Gazebo ne démarre pas

```bash
# Vérifier que Gazebo est installé
which gz

# Rebuild les packages
cd ~/catapaf_ws/TurtleShot
colcon build --packages-select catapaf_description catapaf_gazebo
source install/setup.bash
```

### Pas de topics

```bash
# Vérifier que le bridge fonctionne
ros2 topic list

# Relancer la simulation
# Ctrl+C puis relancer
bash launch_headless.sh
```

### Le robot ne bouge pas

```bash
# Vérifier cmd_vel
ros2 topic echo /cmd_vel

# Publier manuellement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

## 📖 Documentation Complète

- **[WSL2_TROUBLESHOOTING.md](WSL2_TROUBLESHOOTING.md)** - Guide de dépannage WSL2 complet
- **[QUICKSTART.md](QUICKSTART.md)** - Guide général (pas spécifique WSL2)
- **[MIGRATION_GUIDE.md](MIGRATION_GUIDE.md)** - Détails techniques

## ✅ Checklist de Démarrage

- [ ] Ouvrir terminal WSL
- [ ] `cd ~/catapaf_ws/TurtleShot`
- [ ] `source install/setup.bash`
- [ ] `bash launch_headless.sh`
- [ ] Attendre que Gazebo démarre (~10 secondes)
- [ ] Nouveau terminal → `ros2 topic list` → Vérifier que les topics existent
- [ ] Nouveau terminal → `rviz2` → Visualiser le robot
- [ ] Nouveau terminal → `ros2 run catapaf_gazebo catapaf_arm_controller`
- [ ] S'amuser ! 🚀

## 🎉 Résumé

| Commande | Description |
|----------|-------------|
| `bash launch_headless.sh` | Lance la simulation (RECOMMANDÉ) |
| `rviz2` | Visualise le robot |
| `ros2 run catapaf_gazebo catapaf_arm_controller` | Contrôle le bras |
| `ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger` | Lance la catapulte |
| `ros2 topic pub /cmd_vel ...` | Déplace le robot |

---

**Le mode headless fonctionne parfaitement dans WSL2 ! 🚀**
