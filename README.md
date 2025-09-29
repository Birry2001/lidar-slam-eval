# Évaluation de la librairie LiDAR-SLAM de Kitware

Ce dépôt contient le code et les résultats produits dans le cadre d’un stage de Master 1 Robotique à l’Institut Pascal (équipe PerSyst, Université Clermont Auvergne).  
Le projet porte sur l’**évaluation de la librairie open-source LiDAR-SLAM de Kitware** (v3.0, wrapper ROS 2 Jazzy), exploitée sur des données réelles enregistrées en environnement **indoor** et **outdoor**.

L’objectif est de mesurer l’impact de différentes familles de paramètres (extraction de keypoints, ICP/LM, keyframes, undistortion, interpolation…) sur la précision, la robustesse et le temps de calcul, à l’aide d’une campagne systématique d’expériences.

---

## 📂 Contenu du dépôt

- **`slam_orchestrator.py`** : orchestrateur principal, exécute les campagnes de tests depuis un plan YAML, enregistre les bags, extrait les métriques et génère des rapports HTML/JSON.  
- **`scripts_python/`** : scripts Python/Bash complémentaires (métriques de confiance, extraction/plots de trajectoires, résumés, etc.).  
- **`plans/`** : fichiers YAML décrivant les expériences et les paramètres balayés.  
- **`resultats/`** : résultats complets (trajectoires, cartes, CSV de métriques, graphiques).  
- **`slam_config_indoor.yaml`**, **`slam_config_outdoor.yaml`** : configurations de référence utilisées pour initialiser le SLAM.  
- **`rapport_de_stage.pdf`** : rapport final détaillant la méthodologie et l’analyse.

---

## ⚙️ Dépendances

### SLAM Kitware
- [LiDAR-SLAM](https://gitlab.kitware.com/keu-computervision/slam) v3.0  
- Wrapper ROS 2 (`ros2_wrapping`), compilé avec **ROS 2 Jazzy** (Ubuntu 24.04).

### Python (venv recommandé)
Installez ces paquets **dans votre environnement virtuel** :
```bash
pip install typer pyyaml jinja2 pandas numpy matplotlib tqdm
pip install evo --upgrade
```

### Système / ROS 2
- **Ubuntu 24.04** + **ROS 2 Jazzy** :
  - `ros-jazzy-ros-base`
  - `ros-jazzy-rclpy`
  - **rosbag2 (Python)** : `ros-jazzy-rosbag2-*` (inclut `rosbag2_py`)
  - Messages utilisés : `nav_msgs/Odometry`, `lidar_slam/Confidence`  
    → nécessite **le package `lidar_slam`** (wrapper ROS 2 Kitware) **build dans votre workspace** :
    ```
    <ws>/
    └─ src/ros2_wrapping/lidar_slam
    ```
  - Outils build : `colcon`, `ament`, etc.

---

## 🚀 Utilisation rapide

1. **Cloner et installer**
   ```bash
   git clone https://github.com/Birry2001/lidar-slam-eval.git
   cd lidar-slam-eval

   # ROS 2
   source /opt/ros/$ROS_DISTRO/setup.bash  # ex: jazzy

   # Python
   python3 -m venv .venv
   source .venv/bin/activate
   pip install typer pyyaml jinja2 pandas numpy matplotlib tqdm
   pip install evo --upgrade
   ```

2. **Lancer une campagne**
   ```bash
   ./scripts_python/slam_orchestrator.py --plan plans/plan_exemple.yaml --node-name /lidar_slam
   ```

3. **Consulter les résultats**
   - Rapport lisible : `report.html`
   - Rapport machine : `report.json`

---

## 📑 Licence

Ce dépôt orchestre des expériences autour de **LiDAR-SLAM (Kitware)** et d’outils ROS 2.  
Merci de respecter les licences associées aux projets utilisés.
