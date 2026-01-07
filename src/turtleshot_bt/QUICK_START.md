# Quick Start - TurtleShot Behavior Tree

## 🚀 Lancement Complet (Tout-en-un)

```bash
# Build
colcon build --packages-select turtleshot_bt
source install/setup.bash

# Lancer TOUT (Gazebo + Nav2 + BT)
ros2 launch turtleshot_bt full_mission.launch.py
```

C'est tout ! La mission démarre automatiquement.

## 📊 Monitoring avec Groot2

Pendant que le système tourne :

1. **Ouvrir Groot2**
2. **Onglet "Monitor"** (en haut)
3. **Click "Connect"**
4. **Configuration** :
   - Address : `127.0.0.1`
   - Publisher Port : `1666`
   - Server Port : `1667`
5. **Click "Connect"**

Vous verrez l'arbre s'exécuter en temps réel ! 🎨

## 🎯 Déroulé de la Mission

L'arbre va :

1. 🔍 **Chercher une personne avec une balle** (vision)
   - Retry 30 fois
   - Tourne sur place en cherchant

2. 🚗 **Naviguer vers la personne** (Nav2)
   - Utilise la carte SLAM
   - Évite les obstacles

3. ⏳ **Attendre que la personne soit à portée** (LiDAR)
   - Retry 20 fois
   - Vérifie distance < 0.5m

4. 🔍 **Chercher la poubelle** (vision)
   - Retry 30 fois
   - Tourne pour scanner

5. 🚗 **Naviguer vers la poubelle** (Nav2)
   - Path planning

6. 🎯 **Tirer la catapulte**
   - Appel du service

## 🧪 Test Sans Vision

Si vous n'avez pas de vision AI active, simulez les détections :

### Simuler personne détectée :
```bash
ros2 topic pub /detections vision_msgs/msg/Detection2DArray \
  "detections: [{results: [{hypothesis: {class_id: 'person'}}]}]" --once
```

### Simuler poubelle détectée :
```bash
ros2 topic pub /detections vision_msgs/msg/Detection2DArray \
  "detections: [{results: [{hypothesis: {class_id: 'trash'}}]}]" --once
```

## 🔧 Lancement Étape par Étape

Si vous préférez lancer séparément :

### Terminal 1 : Simulation
```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

### Terminal 2 : Navigation
```bash
ros2 launch catapaf_gazebo navigation.launch.py
```

### Terminal 3 : Behavior Tree
```bash
ros2 run turtleshot_bt turtleshot_bt_node
```

### Terminal 4 : Groot2
Ouvrir Groot2 et se connecter

## 📝 Options de Lancement

### Sans GUI Gazebo (headless)
```bash
ros2 launch turtleshot_bt full_mission.launch.py gui:=false
```

### Ne pas démarrer auto
```bash
ros2 launch turtleshot_bt full_mission.launch.py auto_start:=false
```

### Combinaison
```bash
ros2 launch turtleshot_bt full_mission.launch.py gui:=false auto_start:=false
```

## 🐛 Troubleshooting

### "Action server not available"

**Problème** : Nav2 n'est pas prêt

**Solution** : Attendez ~30 secondes que Nav2 démarre complètement

### Le BT reste en "RUNNING" sur FindPerson

**Normal** : Il attend des détections vision

**Solution** : Publiez un mock detection (voir ci-dessus)

### Groot2 ne se connecte pas

**Vérifiez** :
1. Le BT node est lancé
2. Vous voyez les logs "ZMQ Publisher started"
3. Pas de `ws://` dans l'adresse (juste `127.0.0.1`)
4. Ports corrects : 1666/1667

### Nav2 échoue

**Vérifiez** :
1. SLAM génère une carte : `ros2 topic echo /map --once`
2. TF tree est complet : `ros2 run tf2_tools view_frames`
3. Paramètres Nav2 : `src/catapaf_gazebo/config/nav2/nav2_params.yaml`

## 📂 Fichiers Importants

| Fichier | Description |
|---------|-------------|
| `trees/turtleshot_mission.xml` | Behavior tree definition (éditable Groot2) |
| `src/turtleshot_bt_node.cpp` | Main BT node |
| `src/actions/navigate_to_pose_action.cpp` | Nav2 integration |
| `launch/full_mission.launch.py` | Launch complet |

## 🎨 Éditer le Behavior Tree

### Dans Groot2

1. **Onglet "Editor"**
2. **File → Load Tree**
3. Charger : `install/turtleshot_bt/share/turtleshot_bt/trees/turtleshot_mission.xml`
4. **Éditer graphiquement**
5. **File → Save**
6. **Redémarrer le BT node**

Les changements sont appliqués !

## 🎓 Architecture

```
TurtleShot Mission
├─ Gazebo Simulation (catapaf_gazebo)
├─ Nav2 + SLAM (catapaf_gazebo/navigation.launch.py)
└─ Behavior Tree (turtleshot_bt)
    ├─ ZMQ Publisher → Groot2
    ├─ Actions
    │   ├─ NavigateToPose (Nav2 client)
    │   └─ FireCatapult (service client)
    ├─ Conditions
    │   ├─ HasTarget (vision subscriber)
    │   └─ TargetInRange (LiDAR subscriber)
    └─ Decorators
        └─ Retry (custom)
```

## 📖 Documentation Complète

- `README.md` - Documentation détaillée
- `GROOT2_GUIDE.md` - Guide Groot2
- `ARCHITECTURE.md` - Diagrammes architecture (catapaf_gazebo)

## 🎉 C'est Parti !

```bash
ros2 launch turtleshot_bt full_mission.launch.py
```

Puis connectez Groot2 pour voir la magie opérer ! ✨
