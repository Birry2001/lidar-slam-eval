# scripts_de_test — Évaluation LiDAR‑SLAM (Kitware)

Ce répertoire regroupe les **outils d’orchestration, d’extraction et d’analyse** utilisés pour évaluer la librairie **LiDAR‑SLAM de Kitware**.  
Les scripts couvrent : le lancement d’expériences (ROS 2 + bags), l’extraction de trajectoires, le calcul de métriques (ATE/RPE via `evo`, métriques de confiance), et la génération de résumés / figures.

> **Convention d’arborescence (générée par l’orchestrateur)**
>
> ```
> <exp_root>/<test>/<config>/
>   ├─ evo/                     # .tum (trajectoires)
>   ├─ fichiers_csv/            # CSV de confiance (overlap, nb_matches, …)
>   ├─ maps/                    # cartes éventuelles
>   └─ ros_bag/                 # bags (ignorés par Git)
> <exp_root>/<test>/{
>   graphe_metriques_confiance, metriques_evo, summary, trajectoire, metriques_confidence
> }
> ```

---

## Dépendances

### Système / ROS 2
- **ROS 2** (testé avec un environnement type *Jazzy*), `colcon`, `ros2 bag`.
- Accès aux launch files LiDAR (ex. `slam_ouster.launch.py`).

### Python (>= 3.10 recommandé)
- Standard : `argparse`, `subprocess`, `pathlib`, `sys`, `time`, `re`, `json`, `shutil`.
- Analyse : `pandas`, `numpy` (si utilisé par vos notebooks/plots externes), `matplotlib` (si `plot_configs.py` l’utilise).
- CLI & templating : `typer`, `jinja2`, `pyyaml`.
- ROS 2 bag I/O : `rosbag2_py`, `rclpy`, `nav_msgs.msg`, `lidar_slam.msg` (pour `odom_csv_extractor`).

### Outils externes
- **evo** (CLI) : `evo_traj`, `evo_ape`, `evo_rpe` — https://github.com/MichaelGrupp/evo  
  (doivent être dans le `PATH`).

---

# 1) `slam_orchestrator.py` — Orchestrateur de campagnes

**Modules** : `typer`, `yaml`, `jinja2`, `subprocess`, `signal`, `time`, `pathlib`, `json`, `os`, `re`, `shutil`.

**Entrées**
- Un **plan d’expérience YAML** (le script le demande au démarrage). Deux styles acceptés :
  - **Style A (compact)** :
    ```yaml
    experiment: <nom>
    lidar: ouster|hesai|livox|velodyne
    mode: indoor|outdoor
    bag_path: /chemin/vers/bags
    ref_tum:  /chemin/vers/reference.tum
    tests:
      <test_name>:
        configs:
          <cfg1>:
            lidar_slam.ros__parameters.paramX: 1.0
            ...
          <cfg2>: { ... }
    ```
  - **Style B**
    ```yaml
    orchestrator: { experiment, lidar, mode, bag_path|bag, ref_tum }
    tests:
      - name: <test_name>
        params: [optionnel]
        configs:
          - name: <cfg1>
            values: { param: val, ... }
          - name: <cfg2>
            values: { ... }
    ```

**Validation**
- Champs obligatoires de l’orchestrateur (`experiment`, `lidar`, `mode`, `bag_path`, `ref_tum`).
- `lidar` ∈ `{"hesai","livox","ouster","velodyne"}` (via table `LAUNCHES`).
- `mode` ∈ `{"indoor","outdoor"}` → sélection du YAML actif (`slam_config_*.yaml`).
- Noms de **paramètres** conformes au schéma
  `lidar_slam.ros__parameters.*` (ou avec `/` en racine).
- Noms de **configs** : `[a-z0-9][a-z0-9.-]*` et présence d’une config **par‑defaut**.
- **Typage** : lors de l’override, chaque valeur est **coercée** sur le type par défaut
  détecté dans le YAML de référence (bool/int/float/str).

**Algorithme (boucle de campagne)**
1. **Préparation des dossiers** sous `<exp_root>/<test>/<config>/…` et des répertoires de sortie du test.
2. **Override YAML** : écrit un YAML temporaire où les chemins de paramètres
   `lidar_slam.ros__parameters.*` reçoivent les valeurs de la config.
3. **Démarrage** (process group, `start_new_session=True`) :
   - `ros2 bag record …` (si configuré) — enregistrement des topics utiles.
   - `ros2 launch <launch_file>` en fonction du LiDAR choisi.
4. **Lecture LiDAR** : `ros2 bag play <bag_path>`.
5. **Arrêt propre** de tous les processus (SIGINT ➝ SIGTERM ➝ SIGKILL progressifs).
6. **Restauration** du YAML d’origine.
7. **Post‑traitements** automatiques (exécutés par `subprocess.run`) :
   - `collect_confidence_metrics.py -d <index.txt>`  
   - `extract_trajectories.py -d <index.txt>`  
   - `extract_trajectory_plots.py -d <index.txt>`  
   - `summarize_by_metric.py <exp_dir> <summary_output>`  
   - `compute_evo_metrics_no_align.py -d <index.txt> -r <ref_tum>`  
     > **NB** : dans ce dépôt, le script correspondant s’appelle
     `compute_evo_metrics_no_align_test.py`. Adaptez le nom si besoin.
8. **Rapport** : génération d’un **JSON** et d’un **HTML** (via `jinja2`) récapitulant l’exécution.

**Lancer**
```bash
./slam_orchestrator.py   # un prompt demandera le chemin du plan YAML
# ou
python3 slam_orchestrator.py
```

---

# 2) `extract_trajectories.py` — Extraction .TUM depuis les bags

**Modules** : `subprocess`, `argparse`, `time`, `pathlib`.

**Entrées**
- Fichier `-d <dirs.txt>` contenant une liste de **dossiers de paramètres** (un par ligne).
- Chaque dossier doit contenir `ros_bag/all_bag` avec le bag ROS 2.

**Algorithme**
1. Pour chaque dossier **<config>** listé :
   - crée `<config>/evo/`,
   - lance `evo_traj bag2 <config>/ros_bag/all_bag /slam_odom --save_as_tum` (timeout 120 s),
   - renomme/cop ie le `.tum` en `<config>/evo/<config>.tum`.
2. Journalise succès/échecs.

**Sorties**
- `<config>/evo/<config>.tum` (format TUM compatible `evo`).

**Usage**
```bash
./extract_trajectories.py -d dirs.txt
```

---

# 3) `extract_trajectory_plots.py` — Figures de trajectoires

**Modules** : `subprocess`, `argparse`, `pathlib`, `time`.

**Entrées**
- `-d <dirs.txt>` listant les **dossiers de paramètres**.

**Algorithme**
1. Pour chaque dossier de test (parent commun des configs), collecte tous les `.tum` dans `*/evo/*.tum`.
2. **Sanitization** : chaque `.tum` est nettoyé (conserve les lignes à **8 champs**, trim des espaces).
3. Appelle `evo_traj tum <f1.tum> <f2.tum> … --save_plot <test>/trajectoire/<config>.png`.
4. Journalise, avec timeout 60 s.

**Sorties**
- `<test>/trajectoire/<config>.png` (comparaison multi‑trajectoires).

**Usage**
```bash
./extract_trajectory_plots.py -d dirs.txt
```

---

# 4) `compute_evo_metrics_no_align_test.py` — APE/RPE (sans alignement)

**Modules** : `argparse`, `subprocess`, `glob`, `os`, `re`, `pandas`.

**Entrées**
- `-d <dirs.txt>` : liste de **dossiers de paramètres**.
- **Constantes** dans le script : `REF_TUM` (chemin de la **référence**), `DELTA`/`DELTA_UNIT` pour RPE.

**Algorithme**
1. Recherche récursivement tous les sous‑dossiers `evo/` sous chaque paramètre.
2. Pour chaque `evo/` : identifie `<config>.tum`.
3. Exécute :
   - `evo_ape tum <REF_TUM> <cfg.tum> --save_plot <evo>/<cfg>_ape.png`
   - `evo_rpe tum <REF_TUM> <cfg.tum> --delta <DELTA> --delta_unit <DELTA_UNIT> --save_plot <evo>/<cfg>_rpe.png`
4. **Parsing** : lit la sortie texte d’`evo_*` et extrait `RMSE, mean, median, std, min, max`
   via regex.
5. Agrège les résultats **par paramètre** et **globalement**.

**Sorties**
- `<param>/metriques_evo/metrics_local.xlsx` (par dossier),  
- `all_metrics.csv` et `all_metrics.xlsx` (globaux).

**Usage**
```bash
./compute_evo_metrics_no_align_test.py -d dirs.txt
```

---

# 5) `collect_confidence_metrics.py` — Agrégation des CSV de confiance

**Modules** : `argparse`, `pathlib`, `shutil`, `sys`.

**Entrées**
- `-d <dirs.txt>` listant des **dossiers de configuration**.
- Les CSV à collecter doivent se trouver dans `**/fichiers_csv/*.csv` sous chaque config.

**Algorithme**
1. Pour chaque **config** :
   - recherche tous les CSV sous `**/fichiers_csv/`,
   - crée/vid e `<config>/metriques_confidence/`,
   - copie les CSV dedans.
2. Affiche un bilan (copiés / manquants).

**Sorties**
- `<config>/metriques_confidence/*.csv` (copiés).

**Usage**
```bash
./collect_confidence_metrics.py -d dirs.txt
```

---

# 6) `summarize_by_metric.py` — Résumés par métrique

**Modules** : `pandas`, `argparse`, `pathlib`, `sys`.

**Entrées**
- `input_dir` : dossier contenant des CSV nommés **`<metric>_<config>.csv`**.
- `output_path` : dossier cible ou chemin de fichier `.xlsx`.

**Algorithme**
1. Détecte les métriques cibles : `["computation_time","overlap","nb_matches","std_position_error"]`.
2. Pour chaque métrique, parcourt `input_dir` et charge tous les CSV **<metric>_*.csv**.
3. Calcule **min / max / mean** par configuration et construit un DataFrame.
4. Écrit un **Excel** multi‑feuilles (une feuille par métrique), avec **largeurs de colonnes**
   ajustées d’après les valeurs formatées ; peut aussi générer des CSV triés si besoin.

**Sorties**
- `summary_by_metric.xlsx` (ou `<output_path>.xlsx`).

**Usage**
```bash
./summarize_by_metric.py <input_dir> <output_path>
# Ex. : ./summarize_by_metric.py metriques_confidence summary/summary
```

---

# 7) `collect_summaries.py` — Orchestration des résumés

**Modules** : `argparse`, `pathlib`, `subprocess`, `sys`.

**Entrées**
- `-d <dirs.txt>` : liste de **dossiers de configuration**.

**Algorithme**
1. Pour chaque config : vérifie la présence de `metriques_confidence/` non vide.
2. Crée (ou purge) `<config>/summary/`.
3. Lance `summarize_by_metric.py <config>/metriques_confidence <config>/summary`.
4. Arrêt immédiat en cas d’erreur d’une étape (fail‑fast).

**Sorties**
- `<config>/summary/summary_by_metric.xlsx` (résumé par config).

**Usage**
```bash
./collect_summaries.py -d dirs.txt
```

---

# 8) `run_plots_confiance.sh` — Graphes comparatifs (confiance)

**Modules / outils** : bash, `python3`, (dépend d’un script `plot_configs.py` non inclus ici).

**Entrées**
- Un fichier `<dirs.txt>` listant des dossiers de **test** (parents des configs).

**Algorithme**
1. Pour chaque dossier listé :
   - construit les listes de CSV par métrique (depuis `*/fichiers_csv/`),
   - appelle `plot_configs.py --metric <m> --csv <cfg1.csv> --csv <cfg2.csv> … --output <png>`,
   - écrit les PNG sous `<test>/graphe_metriques_confiance/`.
2. Métriques tra cées : `computation_time`, `nb_matches`, `overlap`, `std_position_error`.

**Sorties**
- `<test>/graphe_metriques_confiance/comparaison_<metric>.png`.

**Remarque**
- Le script suppose la présence d’un **`plot_configs.py`** compatible (CLI `--metric/--csv/--output`).

---

# 9) Package ROS 2 `odom_csv_extractor` — Extraction CSV depuis rosbag2

**Modules** : `rosbag2_py`, `rclpy.serialization`, `nav_msgs.msg.Odometry`, `lidar_slam.msg.Confidence`, `argparse`, `csv`, `pathlib`, `os`, `sys`.

**Topics supportés** (extrait du mapping interne `TOPICS_CFG`) :
- `/slam_confidence` → champs: `overlap`, `nb_matches`, `std_position_error`, `computation_time`.
- (Extensible pour `Odometry` si besoin via un mapping similaire).

**Algorithme (fonction `extract_to_csv`)**
1. Ouvre le **bag** via `rosbag2_py.SequentialReader` (`StorageOptions`, `ConverterOptions`).
2. Itère les messages, **filtre** par `--topic` demandé.
3. **Désérialise** chaque message (`deserialize_message`) vers le type ROS attendu.
4. Extrait les **champs** selon un **chemin d’attributs** (ex. `('overlap',)`), via un utilitaire
   générique « get_attr_by_path ».
5. Écrit un CSV horodaté (`stamp`) avec colonnes par champ.
6. Deux modes :
   - **Unitaire** : `--bag /path/to/bagdir --topic /slam_confidence [--outdir out/]`.
   - **Batch** : `-d dirs.txt` (chaque ligne est un **dossier de paramètres** ;
     pour chaque config : lit `ros_bag/all_bag` et écrit `fichiers_csv/<metric>_<config>.csv`).

**Usage ROS 2**
```bash
# Build et source
colcon build --packages-select odom_csv_extractor
source install/setup.bash

# Mode unitaire
ros2 run odom_csv_extractor extract_to_csv --bag <bagdir> --topic /slam_confidence --outdir csv_output

# Mode batch
ros2 run odom_csv_extractor extract_to_csv -d dirs.txt
```

---

## Bonnes pratiques & conseils

- Les dossiers **`ros_bag/`** sont **ignorés** par Git (`**/ros_bag/`), mais nécessaires à l’exécution locale.
- Si vous changez le nom de `compute_evo_metrics_no_align_test.py`, mettez‑le en cohérence avec l’orchestrateur.
- Vérifiez que `evo_*` est dans votre `PATH` (ex. `pipx install evo` ou installation via conda).
- Les chemins YAML par défaut dans l’orchestrateur pointent vers
  `~/test_ws/src/ros2_wrapping/lidar_slam/params/slam_config_*.yaml` et des « originaux » dans `~/Téléchargements/` :
  adaptez‑les si nécessaire.

---

## Aide rapide (cheat‑sheet)

```bash
# 1) Orchestration complète (demande un plan YAML)
./slam_orchestrator.py

# 2) Extraction des TUM depuis les bags
./extract_trajectories.py -d dirs.txt

# 3) Tracés des trajectoires
./extract_trajectory_plots.py -d dirs.txt

# 4) Métriques APE/RPE (réf. TUM configurée dans le script)
./compute_evo_metrics_no_align_test.py -d dirs.txt

# 5) Copie des CSV de confiance vers metriques_confidence/
./collect_confidence_metrics.py -d dirs.txt

# 6) Résumés par métrique vers summary/
./collect_summaries.py -d dirs.txt
```

---

**Contact / Maintenance** : pour toute question sur la structure des plans YAML, l’ajout d’un nouveau lidar/launch ou l’extension des métriques, ouvrez une issue dans le dépôt.
