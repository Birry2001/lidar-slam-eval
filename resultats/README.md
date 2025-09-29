# Résultats d’évaluation — Kitware LiDAR SLAM

Ce dossier contient l’ensemble des **résultats expérimentaux** obtenus lors de l’évaluation de la librairie **LiDAR-SLAM de Kitware** .  
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

- **`indoor/`** : résultats des tests avec des jeux de données indoor 
- **`Outdoor/`** : résultats des tests avec des jeux de données indoor
