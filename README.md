# Évaluation de la librairie LiDAR-SLAM de Kitware

Ce dépôt contient l’ensemble des **scripts**, **plans d’expériences** et **résultats** produits dans le cadre d’un stage de Master 1 Robotique (Institut Pascal — équipe PerSyst, UCA).  
L’objectif est d’**évaluer la librairie open-source LiDAR-SLAM de Kitware** (v3.0, wrapper ROS 2 Jazzy) sur des données réelles (indoor & outdoor), en étudiant la sensibilité des performances aux paramètres clés (extraction de points caractéristiques, ICP/LM, keyframes, undistortion, interpolation…).

---

## 🚀 Contenu du dépôt

- **`scripts_de_test/`** : scripts Python & Bash développés pour automatiser les expériences (orchestrateur, extraction de trajectoires, calcul des métriques evo, consolidation des CSV, génération de graphes).
- **`plans/`** : fichiers `.yaml` décrivant les campagnes de tests (paramètres balayés, configurations par défaut).  
  - `plan_indoor.yaml`  
  - `plan_outdoor.yaml`
- **`resultats/`** : résultats complets des campagnes Indoor & Outdoor.  
  Chaque expérience contient les cartes, trajectoires, métriques CSV, figures evo, graphes de confiance et synthèses.
- **`slam_config_indoor.yaml` / `slam_config_outdoor.yaml`** : configurations de base du SLAM Kitware utilisées comme référence.
- **`rapport_de_stage.pdf`** : version finale du rapport de stage documentant la méthodologie et les résultats.

---

## 📦 Dépendances

### SLAM Kitware
- [LiDAR-SLAM de Kitware](https://gitlab.kitware.com/keu-computervision/slam) (v3.0)  
- Wrapper ROS 2 (`ros2_wrapping`) compilé sous **ROS 2 Jazzy** (Ubuntu 24.04)

### Scripts et analyse
- Python 3.12 avec :
  - [`typer`](https://typer.tiangolo.com/) (CLI orchestrateur),
  - [`pyyaml`](https://pyyaml.org/),
  - [`pandas`](https://pandas.pydata.org/),
  - [`matplotlib`](https://matplotlib.org/),
  - [`jinja2`](https://jinja.palletsprojects.com/) (rapports HTML/JSON),
  - [`tqdm`](https://tqdm.github.io/).
- [**evo**](https://github.com/MichaelGrupp/evo) pour les métriques de trajectoire (ATE, RPE).

### Données
- **Bags ROS 2** indoor et outdoor (Velodyne VLP-16).  
⚠️ Ces fichiers sont lourds et **exclus du dépôt** (`.gitignore`).

---

## ⚙️ Utilisation

### 1. Préparer l’environnement
```bash
# Cloner le dépôt
git clone https://github.com/Birry2001/lidar-slam-eval.git
cd lidar-slam-eval

# Créer un venv
python3 -m venv .venv
source .venv/bin/activate

# Installer les dépendances
pip install -r requirements.txt
