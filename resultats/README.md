# Résultats d’évaluation — Kitware LiDAR SLAM

Ce dossier contient l’ensemble des **résultats expérimentaux** obtenus lors de l’évaluation de la librairie **LiDAR-SLAM de Kitware** (bag indoor et outdoor).  
Chaque sous-dossier correspond à une famille de tests définie dans les plans YAML (`plan_*.yaml`).

---

## Organisation générale

Chaque campagne (`indoor/`, `Outdoor/`) est subdivisée en répertoires thématiques reflétant les paramètres étudiés :

- `ego_motion/`, `ego_motion_registration/` : variantes des paramètres liés à l’odométrie initiale (ICP/LM, saturation, distances voisines, profils matching edge/plane, etc.).
- `ke/` : tests sur l’extraction de keypoints (activation des blobs, azimuth, distances min/max au LiDAR, seuils d’angles, downsampling).
- `LOC_*` : paramètres de localisation (miroirs des familles ego_motion_registration, appliqués lors de l’optimisation pose-map).
- `SLAM_*` : réglages globaux (interpolation modèle, undistortion, keyframe thresholds).
- `KF_thresholds/` : fréquence de création de keyframes.
- `summary/`, `metriques_*`, `graphe_metriques_confiance/`, `trajectoires/` : sorties synthétiques.

Chaque **valeur de paramètre testée** (ex. `par-defaut-4-15`, `strict-1.5-0.4`, `0.5-200`, `enabled`, etc.) correspond à un sous-dossier contenant les résultats bruts et les métriques associées.

---

## Contenu d’un dossier de test

Un dossier de configuration contient généralement les sous-répertoires suivants :

- **`ros_bag/`** : enregistrements ROS 2 utilisés (ignores dans Git via `.gitignore`).
- **`maps/`** : cartes générées par le SLAM (nuages de points PCD/PLY).
- **`fichiers_csv/`** : métriques extraites en CSV (temps de calcul, confiance, recouvrement…).
- **`evo/`** : résultats produits avec [evo](https://github.com/MichaelGrupp/evo) (ATE, RPE, comparaisons de trajectoires).
- **`graphes_confidence/`** et **`graphes_pose/`** (surtout en Outdoor) : figures illustrant l’évolution des métriques de confiance et de la pose.
- **`trajectoire/` / `trajectoires/`** : trajectoires générées et comparées.
- **`summary/`** : synthèses globales par groupe d’expériences.

---

## Indoor vs Outdoor

- **`indoor/`** : résultats sur le bag intérieur, avec une arborescence déjà normalisée (KE\_, ER\_, LOC\_, SLAM\_).
- **`Outdoor/`** : résultats sur le bag extérieur, organisation similaire mais avec quelques variantes :
  - sous-dossiers supplémentaires `graphes_confidence/` et `graphes_pose/` dans chaque test,
  - jeux de paramètres plus larges pour ICP/LM (High/Low, extrêmes, légers),
  - noms parfois abrégés (`HI-HP`, `LI-LP` pour high/low ICP/LM).

---

## Lien avec les scripts

Les scripts du dossier `scripts_de_test/` automatisent la production de ces résultats :

- `slam_orchestrator.py` lit les plans YAML et orchestre les tests.
- `extract_trajectories.py`, `extract_trajectory_plots.py` : extractions et tracés des trajectoires.
- `collect_confidence_metrics.py`, `collect_summaries.py`, `summarize_by_metric.py` : consolidation des CSV et métriques.
- `compute_evo_metrics_no_align_test.py` : calcul des ATE/RPE avec **evo**.
- `run_plots_confiance.sh` : génération des graphes de confiance.

---

## Utilisation

1. Choisir une campagne (`indoor/` ou `Outdoor/`).
2. Explorer les sous-dossiers correspondant au paramètre étudié (KE\_, ER\_, etc.).
3. Ouvrir :
   - les **CSV** (`fichiers_csv/`) pour les valeurs numériques,
   - les **figures** (`evo/`, `graphes_*`) pour les visualisations,
   - les **maps** (`maps/`) pour les reconstructions 3D.
4. Se référer aux fichiers `summary/` et `metriques_*` pour les synthèses inter-tests.

---

## Notes

- Les dossiers `ros_bag/` contiennent des enregistrements lourds et sont exclus du suivi Git.
- Les chemins et valeurs par défaut des tests sont documentés dans les fichiers `plan_*.yaml`.
- Les résultats sont reproductibles via `slam_orchestrator.py` et les scripts annexes.
