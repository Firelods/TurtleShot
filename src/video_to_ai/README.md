# video_to_ai

Package ROS2 pour l'inférence vidéo en temps réel avec YOLO. Ce nœud effectue la segmentation d'instance sur les images caméra et calcule les positions 3D des objets détectés via le nuage de points.

## Fonctionnalités

- Segmentation d'instance avec YOLO (modèle personnalisé)
- Calcul de position 3D médiane des objets via PointCloud2
- Publication d'un nuage de points coloré selon les classes détectées
- Classes détectées :
  - **Human** (jaune)
  - **Red Ball** (rouge)
  - **Trashcan** (vert)

## Installation

### Dépendances Python

```bash
pip install -r src/video_to_ai/video_to_ai/requirements.txt
```

### Build

```bash
colcon build --packages-select video_to_ai --symlink-install
source install/setup.bash
```

## Utilisation

### Lancer le nœud d'inférence

```bash
# Lancement par défaut (avec nuage de points coloré)
ros2 run video_to_ai video_inference_node

# Désactiver la publication du nuage de points coloré
ros2 run video_to_ai video_inference_node --ros-args -p publish_object_cloud:=false
```

### Paramètres

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `publish_object_cloud` | bool | `true` | Active/désactive la publication du nuage de points coloré sur `/ia/object_cloud` |

## Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Flux vidéo d'entrée |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | Nuage de points pour la position 3D |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/ia/segmented_image` | `sensor_msgs/Image` | Image avec overlay de segmentation |
| `/ia/result` | `std_msgs/String` | Résultats JSON des détections (**plus utilisé dans le projet**) |
| `/ia/object_cloud` | `sensor_msgs/PointCloud2` | Nuage de points coloré par classe |

## Format des résultats

Le topic `/ia/result` publie un JSON avec la structure suivante :

```json
{
  "detections": [
    {
      "label": "Human",
      "score": 0.95,
      "bbox": {
        "x_min": 100,
        "y_min": 50,
        "x_max": 300,
        "y_max": 400
      },
      "position_3d": {
        "x": 1.5,
        "y": 0.2,
        "z": 2.3
      }
    }
  ]
}
```

**NB : Ce topic n'est plus utilisé ! C'est une trace du travail que l'on a effectué mais le calcul de position est fait par NAV2 !**

## Modèles

Les modèles YOLO sont stockés dans le dossier `models/` :
- `yolo_best.pt` - Modèle principal entraîné

## Architecture

```
video_to_ai/
├── models/
│   └── yolo_best.pt
├── video_to_ai/
│   ├── __init__.py
│   ├── video_inference_node.py
│   ├── fake_oak_publisher.py
│   └── requirements.txt
├── package.xml
├── setup.py
└── README.md
```
