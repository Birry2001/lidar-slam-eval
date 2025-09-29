# SLAM Orchestrator — README

> Orchestrateur d’expériences ROS 2 pour **lidar_slam** : il applique des variations de paramètres (depuis un plan YAML), lance les runs, enregistre les bags / journaux, extrait les métriques et génère un rapport HTML/JSON.

---

## ✨ Fonctionnalités

* **Plan d’expériences YAML** : liste de tests + configurations de paramètres ROS 2.
* **Override YAML automatique** : écrit les valeurs dans `slam_config_{indoor|outdoor}.yaml` avant chaque run, puis restaure.
* **Lancement des launches** : `ros2 launch lidar_slam <slam_*.launch.py> outdoor:=true|false`.
* **Lecture des paramètres à chaud** via `ros2 --no-daemon param get` (sur un **nom de nœud explicite**).
* **Enregistrement rosbag2** des topics demandés (`--record-topics`, plugin `--record-storage`).
* **Lecture bag** via `ros2 bag play` (args personnalisables).
* **Post-traitements** (optionnels) : scripts d’agrégation métriques, evo, graphs, etc.
* **Rapports enrichis** (`report.json` + `report.html`) avec commandes, timings, statuts, logs.


## 📦 Prérequis

* Linux, Python 3.8+ (poetry/venv recommandé)
* ROS 2 Jazzy
* Colcon (`colcon build`)
* Packages sources dans le workspace :

  * `ros2_wrapping/lidar_slam` (et ses deps)
  * `odom_csv_extractor`
* Dépendances Python (installez dans votre venv):

  * `typer`, `jinja2`, `pyyaml`
* Scripts de post-traitement (placés à côté du script) :
  `collect_confidence_metrics.py`, `collect_summaries.py`, `extract_trajectories.py`, `extract_trajectory_plots.py`, `summarize_by_metric.py`, `compute_evo_metrics_auto_align.py`, `run_plots_confiance.sh`

---

## 📁 Organisation des sorties

Par défaut :

```
<BASE_DIR>/<experiment>/
  ├─ <test>/
  │   ├─ logs/
  │   ├─ metriques_evo/
  │   ├─ metriques_confidence/
  │   ├─ graphe_metriques_confiance/
  │   ├─ summary/
  │   └─ <config>/
  │       ├─ logs/                # launch.log, record.log, play.log
  │       ├─ ros_bag/all_bag/    # rosbag2 enregistré
  │       ├─ evo/                # .tum etc.
  │       ├─ fichiers_csv/       # export CSV (si odom_csv_extractor présent)
  │       └─ maps/
  ├─ dirs/<experiment>.txt       # index déplacé à la fin
  ├─ report.json                 # rapport machine
  └─ report.html                 # rapport lisible
```

---

## 🧩 Plan d’expériences (YAML)

Deux styles sont supportés:

### Style A (compact)

```yaml
experiment: "NOUVEAU_test_1"
lidar: "velodyne"           # hesai | livox | ouster | velodyne
mode: "outdoor"             # indoor | outdoor

bag: "/chemin/vers/bag.db3" # ou dossier rosbag2, ou .mcap
ref_tum: "/chemin/vers/ref.tum"

# Optionnel (si vous ne préférez pas la CLI) :
# record_topics: "/slam_odom /slam_confidence"
# play_args: "--clock --rate 1.0"
# launch_file: "slam_velodyne.launch.py"
# launches: { velodyne: "slam_velodyne.launch.py" }
# node_name: "/lidar_slam"

tests:
  KE_blob_enable:
    tags: [ke, outdoor]
    configs:
      "par-defaut-disabled":
        /lidar_slam.ros__parameters.slam.ke.enable.blob: false
      "enabled":
        /lidar_slam.ros__parameters.slam.ke.enable.blob: true
```

### Style B (orchestrator + tests list)

```yaml
orchestrator:
  experiment: "NOUVEAU_test_1"
  lidar: "velodyne"
  mode: "outdoor"
  bag: "/chemin/vers/bag.db3"
  ref_tum: "/chemin/vers/ref.tum"
  # Optionnel : record_topics, play_args, launch_file, launches, node_name

tests:
  - name: "KE_blob_enable"
    tags: [ke]
    configs:
      - name: "par-defaut-disabled"
        values:
          /lidar_slam.ros__parameters.slam.ke.enable.blob: false
      - name: "enabled"
        values:
          /lidar_slam.ros__parameters.slam.ke.enable.blob: true
```

#### Règles de validation

* Chaque **test** doit avoir au moins une config dont le nom commence par `par-defaut`.
* Les clés paramètres doivent commencer par :

  * `/lidar_slam.ros__parameters.` **ou** `lidar_slam.ros__parameters.`
* Les types sont vérifiés **contre le YAML de référence** : bool/int/float.
* `bag` peut être un **fichier** `.db3`/`.mcap` ou un **dossier** rosbag2.


---

## 🚀 Utilisation

### 1) Préparer l’environnement

```bash
# activer ROS 2 (adapté à votre distro)
source /opt/ros/$ROS_DISTRO/setup.bash

# activer le venv python
python3 -m venv .venv && source .venv/bin/activate
pip install typer jinja2 pyyaml
```

### 2) Construire (colcon)

> L’orchestrateur peut s’occuper de `colcon build` tout seul (par défaut).
> Sinon, passez `--skip-build` si vous avez déjà build.

### 3) Lancer un run minimal

```bash
./slam_orchestrator.py   --plan /chemin/plan.yaml   --ws /home/<user>/test_ws   --base-dir /home/<user>/test_ws/Evaluation_SLAM_KITWARE/Nouveaux_tests   --active-yaml-outdoor /home/<user>/test_ws/src/ros2_wrapping/lidar_slam/params/slam_config_outdoor.yaml   --orig-yaml-outdoor /home/<user>/Téléchargements/slam_config_outdoor.yaml   --record-topics "/slam_odom /slam_confidence"   --record-storage sqlite3   --play-args "--clock --rate 1.0"
```

**Options utiles** :

* `--only-tests KE_blob_enable,KE_voxel_size`
* `--tags ke,downsample` (filtre OR sur les tags)
* `--resume/--no-resume` (respecter le checkpoint)
* `--skip-post` (désactiver les scripts de post-traitement)
* `--dry-run` (affiche les commandes, ne lance pas ROS)

### 4) Variables d’environnement (fallbacks)

* `SLAM_WS`, `SLAM_BASE_DIR`
* `SLAM_ACTIVE_YAML_OUT`, `SLAM_ACTIVE_YAML_IN`
* `SLAM_ORIG_YAML_OUT`, `SLAM_ORIG_YAML_IN`

CLI > ENV > défauts.

---

## ⚙️ Paramètres clés & temps

Valeurs par défaut (modifiables en CLI) :

```python
# Délais/temps
RECORDER_FLUSH_TIMEOUT_S = 290   # délai max pour flush propre du 'ros2 bag record' (SIGINT→wait)
LAUNCH_STOP_TIMEOUT_S    = 30    # délai max d'arrêt propre du launch
SPAWN_GRACE_S            = 3     # petite marge après spawn des process

# Patience / lectures de paramètres
WAIT_SERVICE_TIMEOUT_S   = 0     # (non utilisé pour découverte — node_name requis)
WAIT_PUB_TIMEOUT_S       = 15.0  # attente optionnelle d'un publisher /slam_odom si --wait-pub
PARAM_CHECK_MAX_TRIES    = 1     # nb tentatives 'ros2 param get' par param
PARAM_CHECK_SLEEP_S      = 1.0   # sleep initial entre tentatives (backoff)
```


---

## 🧪 Exemple de plan + commande

**plan.yaml**

```yaml
experiment: "NOUVEAU_test_1"
lidar: "velodyne"
mode: "outdoor"
bag: "/home/nochi/STAGE_M1_NOCHI/bag1/bag1.db3"
ref_tum: "/home/nochi/test_ws/Evaluation_SLAM_KITWARE/depot_git/resultats/Outdoor/test_bag_1/par_defaut.tum"
tests:
  KE_blob_enable:
    configs:
      "par-defaut-disabled":
        /lidar_slam.ros__parameters.slam.ke.enable.blob: false
      "enabled":
        /lidar_slam.ros__parameters.slam.ke.enable.blob: true
```

**Commande**

```bash
./slam_orchestrator.py   --plan /home/nochi/test_ws/Evaluation_SLAM_KITWARE/Nouveaux_tests/plan.yaml   --ws /home/nochi/test_ws   --base-dir /home/nochi/test_ws/Evaluation_SLAM_KITWARE/Nouveaux_tests   --active-yaml-outdoor /home/nochi/test_ws/src/ros2_wrapping/lidar_slam/params/slam_config_outdoor.yaml   --orig-yaml-outdoor /home/nochi/Téléchargements/slam_config_outdoor.yaml   --record-topics "/slam_odom /slam_confidence"   --record-storage sqlite3   --play-args "--clock --rate 1.0"
```

Résultat : création de `.../Nouveaux_tests/NOUVEAU_test_1/` + rapport.

---

## 🧰 Options CLI (référence)

```
--ws PATH                         Workspace ROS 2
--base-dir PATH                   Dossier où créer <experiment>/
--active-yaml-indoor PATH         YAML actif indoor
--active-yaml-outdoor PATH        YAML actif outdoor
--orig-yaml-indoor PATH           YAML d’origine indoor
--orig-yaml-outdoor PATH          YAML d’origine outdoor

--install-mode [symlink|merge]    Mode d’install colcon
--odom-pkg-path PATH              Chemin du package odom_csv_extractor (optionnel)

--plan PATH                       Plan YAML
--only-tests CSV                  Exécuter uniquement ces tests
--tags CSV                        Filtrer par tags (OR logique)
--resume / --no-resume            Respecter le checkpoint

--dry-run                         N’exécute pas ROS, log seulement
--skip-build                      Ne pas lancer colcon build
--skip-post                       Ne pas lancer les post-traitements
--wait-pub                        Attendre /slam_odom avant lecture params
--param-max-tries INT             Tentatives 'ros2 param get' (par param)
--param-sleep FLOAT               Sleep initial entre tentatives

--record-topics "LISTE"           Topics à enregistrer (ex: "/slam_odom /slam_confidence")
--record-storage [sqlite3|mcap]   Plugin d’écriture rosbag2
--play-args "..."                 Arguments passés à 'ros2 bag play'

--node-name "/lidar_slam"         Nom complet du nœud lidar à interroger
```

---

## 🔎 Logs & rapports

* `logs/launch.log` : sortie du launch
* `logs/record.log` : sortie du recorder
* `logs/play.log` : sortie du player
* `logs/node_list.txt`, `topic_list.txt`, `param_dump.yaml` : snapshot du graphe ROS au moment de l’exécution
* `report.html` : tableau par config (timings, args, node)
* `report.json` : version machine du rapport

---



## 📞 Support

* Ouvrez une issue interne avec :

  * la commande complète,
  * le `plan.yaml`,
  * l’archive du dossier `<base-dir>/<experiment>/` (incluant `report.json`, logs/).

Bon tests ! 🧭
