# Évaluation de la librairie LiDAR-SLAM de Kitware

Ce dépôt regroupe le code et les résultats produits dans le cadre d’un stage de Master 1 Robotique à l’Institut Pascal (équipe PerSyst, Université Clermont Auvergne).  
Le travail porte sur l’**évaluation de la librairie open-source LiDAR-SLAM développée par Kitware** (v3.0, wrapper ROS 2 Jazzy), avec des données réelles enregistrées en environnement **indoor** et **outdoor**.  
L’objectif est de mesurer l’impact de différents paramètres du SLAM (extraction de features, ICP/LM, keyframes, undistortion, interpolation…) sur la précision, la robustesse et le temps de calcul.



## Contenu du dépôt

- **`scripts_de_test/`** : ensemble de scripts Python et Bash utilisés pour automatiser les expériences  
  (orchestration, extraction de trajectoires, calcul des métriques avec *evo*, génération de graphiques).
- **`plans/`** : fichiers YAML décrivant les campagnes de tests et les paramètres balayés.
- **`resultats/`** : résultats complets (trajectoires, cartes, CSV de métriques, graphiques).
- **`slam_config_indoor.yaml`** et **`slam_config_outdoor.yaml`** : configurations de référence utilisées pour lancer le SLAM.
- **`rapport_de_stage.pdf`** : rapport final détaillant la méthodologie et l’analyse des résultats.



## Dépendances

### SLAM Kitware
- [LiDAR-SLAM](https://gitlab.kitware.com/keu-computervision/slam) v3.0  
- Wrapper ROS 2 (`ros2_wrapping`), compilé avec **ROS 2 Jazzy** (Ubuntu 24.04)

### Scripts et post-traitement
- Python 3.12 avec :
  - `typer` (interface CLI pour l’orchestrateur)
  - `pyyaml`
  - `pandas`
  - `matplotlib`
  - `jinja2` (rapports HTML/JSON)
  - `tqdm`
- [**evo**](https://github.com/MichaelGrupp/evo) pour l’évaluation des trajectoires (ATE, RPE).

### Données
Les **rosbags** utilisés (Velodyne VLP-16) ne sont pas inclus dans le dépôt à cause de leur taille.  
 Les répertoires `ros_bag/` sont ignorés via `.gitignore`.



## Comment l’utiliser

### 1. Installation
```bash
git clone https://github.com/Birry2001/lidar-slam-eval.git
cd lidar-slam-eval

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
