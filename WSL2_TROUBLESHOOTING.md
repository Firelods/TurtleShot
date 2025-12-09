# WSL2 Troubleshooting Guide - Gazebo Fortress

## Problème : Gazebo crash avec OGRE2

### Symptôme

```
OGRE EXCEPTION(9:UnimplementedException): in GL3PlusTextureGpu::copyTo
Aborted (Signal sent by tkill())
```

### Cause

OGRE2 (le moteur de rendu moderne) ne fonctionne pas bien avec l'accélération graphique WSL2 et certaines cartes graphiques.

## Solutions

### Solution 1 : Mode Headless (RECOMMANDÉ pour WSL2)

Lancer Gazebo **sans GUI** (mode serveur uniquement) :

```bash
ros2 launch catapaf_gazebo gz_simulation_headless.launch.py
```

**Avantages :**
- ✅ Stable dans WSL2
- ✅ Moins de ressources utilisées
- ✅ Parfait pour tests automatisés
- ✅ Visualisation possible avec RViz2

**Pour visualiser :**
```bash
# Dans un autre terminal
rviz2
```

### Solution 2 : Utiliser OGRE (au lieu de OGRE2)

Le fichier world a été modifié pour utiliser OGRE au lieu de OGRE2.

**Fichier modifié :** `src/catapaf_gazebo/worlds/turtlebot_world.sdf`

```xml
<render_engine>ogre</render_engine>  <!-- au lieu de ogre2 -->
```

**Test :**
```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

### Solution 3 : Variables d'environnement OpenGL

Essayez ces variables d'environnement avant de lancer :

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

### Solution 4 : Mise à jour WSL et pilotes graphiques

```bash
# Dans PowerShell (Windows)
wsl --update

# Installer/mettre à jour les pilotes GPU
# Télécharger depuis le site du fabricant :
# - NVIDIA: https://www.nvidia.com/Download/index.aspx
# - AMD: https://www.amd.com/en/support
# - Intel: https://www.intel.com/content/www/us/en/download-center/home.html
```

### Solution 5 : Désactiver les capteurs visuels

Si vous n'avez pas besoin de la caméra, modifiez le URDF pour commenter les capteurs caméra :

```bash
# Éditer le fichier
nano ~/catapaf_ws/TurtleShot/src/catapaf_description/urdf/turtlebot_with_catapaf_gz.urdf.xacro

# Commenter les sections camera sensor (lignes ~453-490)
```

## Mode Headless - Guide Complet

### 1. Lancer la simulation headless

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash
ros2 launch catapaf_gazebo gz_simulation_headless.launch.py
```

### 2. Vérifier que tout fonctionne

```bash
# Vérifier les topics
ros2 topic list

# Tester le LiDAR
ros2 topic echo /scan --once

# Tester l'odométrie
ros2 topic echo /odom --once
```

### 3. Contrôler le robot

```bash
# Déplacer le robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Contrôler le bras
ros2 run catapaf_gazebo catapaf_arm_controller
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger
```

### 4. Visualiser avec RViz2

```bash
# Dans un nouveau terminal
source install/setup.bash
rviz2
```

**Configuration RViz2 :**
1. Fixed Frame: `odom`
2. Add → LaserScan → Topic: `/scan`
3. Add → RobotModel
4. Add → TF

## Comparaison des Modes

| Mode | GUI | Stabilité WSL2 | Utilisation CPU | Caméra |
|------|-----|----------------|-----------------|--------|
| **Headless** | ❌ | ✅ Excellent | Faible | ✅ (données disponibles) |
| **GUI OGRE** | ✅ | ⚠️ Moyen | Moyen | ✅ |
| **GUI OGRE2** | ✅ | ❌ Instable | Élevé | ✅ |

## Tests Recommandés

### Test 1 : Headless fonctionne-t-il ?

```bash
ros2 launch catapaf_gazebo gz_simulation_headless.launch.py
```

**Attendu :** Pas de crash, topics disponibles

### Test 2 : GUI avec OGRE fonctionne-t-il ?

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

**Attendu :** GUI s'ouvre (peut être lent)

### Test 3 : RViz2 fonctionne-t-il ?

```bash
rviz2
```

**Attendu :** Fenêtre RViz s'ouvre

## Optimisations WSL2

### 1. Augmenter la mémoire WSL

Créer/éditer `.wslconfig` dans `C:\Users\<VotreNom>\`:

```ini
[wsl2]
memory=8GB
processors=4
swap=4GB
```

Redémarrer WSL :
```powershell
wsl --shutdown
```

### 2. Activer WSLg (GUI support)

WSLg est normalement activé par défaut dans WSL2. Vérifier :

```bash
echo $DISPLAY
# Devrait afficher quelque chose comme `:0`
```

### 3. Limiter la charge CPU

Si Gazebo consomme trop de CPU :

```bash
export IGN_GAZEBO_PHYSICS_THREAD_COUNT=2
ros2 launch catapaf_gazebo gz_simulation_headless.launch.py
```

## Alternatives

### Option A : Utiliser Gazebo Classic (ancien)

Le package `robot_gazebo_sim` utilise Gazebo Classic qui est plus stable dans WSL2 :

```bash
ros2 launch robot_gazebo_sim gazebo_wsl.launch.py
```

### Option B : Docker avec GPU

Utiliser Docker avec support GPU :

```bash
# Installer Docker Desktop avec WSL2 backend
# Puis lancer le container avec GPU support
```

### Option C : Machine virtuelle native

- VirtualBox ou VMware avec Ubuntu natif
- Meilleure performance graphique
- Plus de configuration initiale

## Logs de Débogage

### Activer les logs détaillés

```bash
export IGN_VERBOSE=4
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

### Vérifier OpenGL

```bash
glxinfo | grep "OpenGL version"
glxinfo | grep "OpenGL renderer"
```

### Tester la carte graphique

```bash
# Installer mesa-utils si nécessaire
sudo apt install mesa-utils

# Tester OpenGL
glxgears
```

## Recommandation Finale pour WSL2

**Pour le développement quotidien :**
```bash
# Mode headless + RViz2
ros2 launch catapaf_gazebo gz_simulation_headless.launch.py
```

**Pour les démos :**
- Utiliser une machine native Linux
- Ou Gazebo Classic (`robot_gazebo_sim`)

## Support

Si les problèmes persistent :

1. Vérifier les issues GitHub Gazebo : https://github.com/gazebosim/gz-sim/issues
2. Forums ROS : https://answers.ros.org/
3. Consulter la doc WSLg : https://github.com/microsoft/wslg

---

**Note :** Le mode headless est **recommandé** pour WSL2 et fonctionne parfaitement pour le développement !
