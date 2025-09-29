Voici une **mise à jour du README** intégrant les 3 scripts (dont *deux nouveaux* : `detect_failures.py` et `viz_all.py`) et en harmonisant les dépendances / usages.

---

# scripts_de_test — Évaluation LiDAR-SLAM (Kitware)

Ce répertoire regroupe les **outils d’orchestration, d’extraction, d’analyse et de visualisation** utilisés pour évaluer la librairie **LiDAR-SLAM de Kitware**.
Les scripts couvrent : le lancement d’expériences (ROS 2 + bags), l’extraction de trajectoires, le calcul de métriques (ATE/RPE via `evo`, métriques de confiance), la **détection automatique d’échecs**, la consolidation d’impacts et la **génération de figures**.

> **Convention d’arborescence (générée par l’orchestrateur)**
>
> ```
> <exp_root>/<test>/<config>/
>   ├─ evo/                     # .tum (trajectoires) + plots APE/RPE
>   ├─ fichiers_csv/            # CSV de confiance (overlap, nb_matches, …)
>   ├─ metriques_confidence/    # CSV de confiance collectés (après collect)
>   ├─ metriques_evo/           # xlsx/plots EVO (locaux à la config)
>   ├─ maps/                    # cartes éventuelles
>   └─ ros_bag/                 # bags (ignorés par Git)
> <exp_root>/<test>/{
>   graphe_metriques_confiance, metriques_evo, summary, trajectoire
> }
> ```

---

## Dépendances

### Système / ROS 2

* **ROS 2** (testé avec un environnement type *Jazzy*), `colcon`, `ros2 bag`.
* Accès aux launch files LiDAR (ex. `slam_ouster.launch.py`).

### Python (>= 3.10 recommandé)

* Standard : `argparse`, `subprocess`, `pathlib`, `sys`, `time`, `re`, `json`, `shutil`.
* Analyse : `pandas`, `numpy`, `matplotlib`, `pyyaml`.
* CLI & templating : `typer`, `jinja2`.
* ROS 2 bag I/O (package optionnel) : `rosbag2_py`, `rclpy`, `nav_msgs.msg`, `lidar_slam.msg` (pour `odom_csv_extractor`).

### Outils externes

* **evo** (CLI) : `evo_traj`, `evo_ape`, `evo_rpe` — [https://github.com/MichaelGrupp/evo](https://github.com/MichaelGrupp/evo)
  (doivent être dans le `PATH`).

---

# 1) `slam_orchestrator.py` — Orchestrateur de campagnes

Lit un **plan YAML** (styles A/B), valide les champs, **override** les paramètres du YAML actif `slam_config_{indoor|outdoor}.yaml`, lance **bag record + launch + bag play**, lit les paramètres runtime, **arrête proprement** et enchaîne les **post-traitements** (collecte CSV, TUM, EVO, résumés, plots). Rapports **JSON/HTML** générés.

---

# 2) `extract_trajectories.py` — Extraction .TUM depuis les bags

Extrait des `.tum` depuis `ros_bag/all_bag` (topic `/slam_odom`) et les place dans `<config>/evo/<config>.tum`.
**Usage** :

```bash
./extract_trajectories.py -d dirs.txt
```

---

# 3) `extract_trajectory_plots.py` — Figures de trajectoires

Agrège les `.tum` par **test** et génère des PNG comparatifs sous `<test>/trajectoire/`.
**Usage** :

```bash
./extract_trajectory_plots.py -d dirs.txt
```

---

# 4) `compute_evo_metrics_no_align_test.py` — APE/RPE (sans alignement)

Parcourt les dossiers `evo/`, lance `evo_ape` / `evo_rpe`, parse les sorties, **agrège** et exporte `metrics_local.xlsx` (par dossier) + `all_metrics.(csv|xlsx)` globaux.
**Usage** :

```bash
./compute_evo_metrics_no_align_test.py -d dirs.txt
```

---

# 5) `collect_confidence_metrics.py` — Agrégation des CSV de confiance

Copie tous les CSV trouvés sous `**/fichiers_csv/*.csv` vers `<config>/metriques_confidence/`, pour chaque **config** listée dans `-d`.
**Détails script** : recherche récursive, création du dossier cible, copie avec journalisation 
**Usage** :

```bash
./collect_confidence_metrics.py -d dirs.txt
```

---

# 6) `summarize_by_metric.py` — Résumés par métrique

Charge les CSV **<metric>_*.csv**, calcule **min / max / mean** par config, et écrit un **Excel** multi-feuilles `summary_by_metric.xlsx` (une feuille par métrique).
**Usage** :

```bash
./summarize_by_metric.py <input_dir> <output_path>
# Ex. : ./summarize_by_metric.py metriques_confidence summary/summary
```

---

# 7) `collect_summaries.py` — Orchestration des résumés

Pour chaque **config**, vérifie `metriques_confidence/`, crée `<config>/summary/` et y écrit `summary_by_metric.xlsx` via `summarize_by_metric.py`.
**Usage** :

```bash
./collect_summaries.py -d dirs.txt
```

---

# 8) `run_plots_confiance.sh` — Graphes comparatifs (confiance)

Appelle un script plotting (`plot_configs.py`) pour tracer `computation_time`, `nb_matches`, `overlap`, `std_position_error`, et sauvegarde sous `<test>/graphe_metriques_confiance/`.

---

# 9) Package ROS 2 `odom_csv_extractor` — Extraction CSV depuis rosbag2

`ros2 run odom_csv_extractor extract_to_csv` — modes **unitaire** (`--bag`, `--topic`) ou **batch** (`-d dirs.txt`).
Génère les CSV de confiance (et éventuellement odom) dans `fichiers_csv/`.

---

# 10) `detect_failures.py` — Détection automatique d’échecs SLAM

Détecte, par **test/config**, des cas d’échec probables, en combinant seuils **robustes** (Médiane + *k*·MAD) et **règles** sur les indicateurs :

* `std_position_error_max` au-dessus d’un seuil robuste (avec *cap* dur minimal `--std-cap`).
* `APE`/`RPE` issus d’EVO (priorité *max*, sinon *RMSE* si `--use-rmse`).
* Cas *impossibles* depuis `summary_by_metric.xlsx` : `overlap_mean/min ≥ 0.999`, `position_error_mean == 0`, `std_position_error == 0`.
* **Trajectoire constante** dans les `.tum` (toutes positions ≈ 0 ou très faible variance).
  Le script produit `failures.csv` et `failures.json` **dans chaque dossier** de test, et un **`global_failures.csv` agrégé** (si `--out` fourni, sinon à la racine de travail). 

**Options principales** :

* `-f, --dirs-file` : fichier listant les dossiers de tests.
* `--std-cap <float>` : cap dur minimal pour `std_position_error_max` (défaut 0.15). 
* `--k-mad <float>` : facteur *k* pour le seuil robuste (défaut 3.0). 
* `--use-rmse` : bascule sur RMSE si les colonnes `*_max` EVO sont absentes. 
* `--out <PATH>` : chemin du CSV global agrégé (optionnel). 

**Usage** :

```bash
./detect_failures.py -f /chemin/EXP.txt --std-cap 0.15 --k-mad 3.0 --use-rmse --out /chemin/global_failures.csv
```

---

# 11) `viz_all.py` — Visualisations globales (impacts Irel) + pipeline optionnel

Script *tout-en-un* pour :

1. **(Optionnel)** Lancer `detect_failures.py` et récupérer `global_failures.csv`.
2. **(Optionnel)** Lancer `analyse_impacts.py` pour produire `impacts_*.csv` (avec exclusion/étiquetage des configs en échec).
3. **Charger** les `impacts_*.csv` et **générer des figures** :

   * **Barres horizontales** (|Irel| par paramètre et par métrique),
   * **Radar** (axes = métriques, toiles = paramètres),
   * **Pairs** *Best vs Worst* (par métrique/paramètre),
   * **Heatmap** (paramètres × métriques),
   * **Mindmap** (catégories → paramètres ; taille ~ impact moyen |Irel|).
     Contraintes graphiques : **matplotlib uniquement**, **1 figure par plot**, **pas de couleurs imposées** (hérite du style par défaut). 

### Fichier YAML de configuration (exemple minimal)

```yaml
out_dir:  /chemin/TRAITEMENT_RESULTATS/figures
csv_dir:  /chemin/TRAITEMENT_RESULTATS
overwrite_figs: true

failures:
  enabled: true
  detect_script: detect_failures.py
  dirs_file: /chemin/dirs/EXP.txt
  out_csv:   /chemin/TRAITEMENT_RESULTATS/global_failures.csv
  std_cap: 0.15
  k_mad: 3.0
  use_rmse: false

extraction:
  enabled: true
  index_file: /chemin/dirs/EXP.txt
  out_xlsx:  /chemin/TRAITEMENT_RESULTATS/impacts_consolides.xlsx
  no_overlap: false
  overwrite:  true

analyse_args:
  failures_mode: exclude   # exclude | tag | ignore

plots: [bar, radar, heatmap, pair, mindmap]
topk: 12
pairs:
  - { metric: computation_time, param: KE_blob }
  - { metric: ATE_RMSE,        param: EM_mode }
include_metrics: [overlap, std_position_error, computation_time, ATE_RMSE, RPE_RMSE]
exclude_metrics: []
auto_categories: true
categories_merge: true
categories: /chemin/categories.json   # optionnel
```

**Remarques d’implémentation** :

* Si `failures.enabled: true`, le script rend exécutable `detect_failures.py`, valide `dirs_file`, *exécute* le script puis **localise** `global_failures.csv` (ordre : `failures.out_csv` > parent de `extraction.out_xlsx` > `dirs_file.parent` > `csv_dir` > `cwd`). 
* Si `extraction.enabled: true`, il **nettoie** (si `overwrite`) les anciens `impacts_*.csv`/xlsx, puis lance `analyse_impacts.py` avec `--out`, `--no-overlap` éventuel et **passe** `--failures-file` + `--failures-mode` quand un CSV d’échecs est présent. 
* Les figures sont écrites dans `out_dir` ; les impacts sont lus depuis `csv_dir` (ou, à défaut, le parent de `out_xlsx`). 

**Usage** :

```bash
python3 viz_all.py config.yaml
```

---

## Bonnes pratiques & conseils

* Les dossiers **`ros_bag/`** sont **ignorés** par Git (`**/ros_bag/`), mais nécessaires à l’exécution locale.
* Synchronisez les noms de scripts appelés par l’orchestrateur si vous renommez `compute_evo_metrics_no_align_test.py`.
* Vérifiez que `evo_*` est dans votre `PATH`.
* Pour des séries longues de runs, utilisez `detect_failures.py` pour **écrémer** les configs douteuses, puis `viz_all.py` pour **visualiser** les impacts nettoyés.

---

## Aide rapide (cheat-sheet)

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

# 7) Détection d’échecs (global + par test)
./detect_failures.py -f EXP.txt --std-cap 0.15 --k-mad 3.0 --use-rmse --out /chemin/global_failures.csv

# 8) Visualisations globales (pipeline complet optionnel)
python3 viz_all.py config.yaml
```

---

**Contact / Maintenance** : pour toute question (structure des plans YAML, ajout d’un nouveau LiDAR/launch, métriques, pipeline d’échecs/impacts/figures), ouvrez une issue dans le dépôt.
