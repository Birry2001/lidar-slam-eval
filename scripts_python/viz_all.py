#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viz_all.py — Générateur global de visualisations d'impacts (Irel) + détection d'échecs optionnelle
--------------------------------------------------------------------------------------------------
Ce script peut :
  1) (optionnel) LANCER detect_failures.py pour produire un CSV d'échecs (global_failures.csv)
  2) (optionnel) LANCER analyse_impacts.py en lui passant --failures-file (pour exclure/étiqueter)
  3) Charger les impacts_*.csv et produire les figures :
     - Barres horizontales (|Irel| par paramètre et par métrique)
     - Radar (axes = métriques, une toile par paramètre)
     - Paires Best vs Worst
     - Heatmap (paramètres × métriques)
     - Mindmap (catégories → paramètres ; taille ~ impact moyen |Irel|)

Contraintes plots : matplotlib uniquement, 1 figure par plot, pas de couleurs imposées.

YAML attendu (exemple minimal) :
--------------------------------
out_dir:  /chemin/TRAITEMENT_RESULTATS/figures
csv_dir:  /chemin/TRAITEMENT_RESULTATS
overwrite_figs: true

# 1) Détection d'échecs (optionnelle)
failures:
  enabled: true
  detect_script: detect_failures.py           # chemin absolu ou relatif (sinon pris à côté de ce script)
  dirs_file: /chemin/dirs/EXP.txt
  out_csv:   /chemin/TRAITEMENT_RESULTATS/global_failures.csv   # optionnel (par défaut: parent de out_xlsx/extraction ou csv_dir)
  std_cap: 0.15
  k_mad: 3.0
  use_rmse: false
  evo_cap: null                               # optionnel

# 2) Extraction des impacts (optionnelle)
extraction:
  enabled: true
  index_file: /chemin/dirs/EXP.txt
  out_xlsx:  /chemin/TRAITEMENT_RESULTATS/impacts_consolides.xlsx
  no_overlap: false
  overwrite:  true

# 3) Paramètres d'analyse (passés à analyse_impacts.py)
analyse_args:
  failures_mode: exclude   # exclude | tag | ignore

# 4) Plots
plots: [bar, radar, heatmap, pair, mindmap]
topk: 12
pairs:
  - { metric: computation_time, param: KE_blob }
  - { metric: ATE_RMSE,        param: EM_mode }
categories: /chemin/categories.json
include_metrics: [overlap, std_position_error, computation_time, ATE_RMSE, RPE_RMSE]
exclude_metrics: []
pairs_auto_topk: 5
auto_categories: true
categories_merge: true

Usage :
  python3 viz_all.py config.yaml
"""

import argparse
import json
import re
import subprocess
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
import shutil


# ---------------------------- Détection d'échecs (optionnelle) ----------------------------

def run_failures_detection_if_requested(cfg: dict, default_base_dir: Optional[Path]) -> Optional[Path]:
    """
    Si cfg['failures']['enabled'] est vrai :
      - résout un chemin exécutable vers detect_failures.py
      - exécute le script directement (sans 'python3')
      - tente de localiser le CSV produit ('global_failures.csv')
    Retourne le Path du CSV produit ou None si échec.
    """
    import shutil
    import stat

    bloc = cfg.get("failures")
    if not isinstance(bloc, dict) or not bloc.get("enabled", False):
        return None

    # 1) Résolution du script
    detect_script_cfg = bloc.get("detect_script")
    candidates: List[Path] = []
    if detect_script_cfg:
        p = Path(detect_script_cfg).expanduser()
        candidates.append(p if p.is_absolute() else Path(__file__).parent / p)
        candidates.append(Path.cwd() / p)
    else:
        candidates.append(Path(__file__).parent / "detect_failures.py")
    # Essai via PATH en dernier
    which = shutil.which(detect_script_cfg or "detect_failures.py")
    if which:
        candidates.append(Path(which))

    detect_script: Optional[Path] = None
    for c in candidates:
        if c.exists():
            detect_script = c.resolve()
            break
    if detect_script is None:
        print(f"⚠️  detect_failures.py introuvable (essayé: {', '.join(str(x) for x in candidates)}) — détection ignorée.")
        return None

    # 2) Assure l'exécutabilité (+x)
    try:
        st = detect_script.stat()
        if not (st.st_mode & stat.S_IXUSR):
            detect_script.chmod(st.st_mode | stat.S_IXUSR)
    except Exception as e:
        print(f"⚠️  Impossible de rendre exécutable {detect_script}: {e}")

    # 3) Vérifications d'entrées
    dirs_file = bloc.get("dirs_file") or cfg.get("dirs_file")
    if not dirs_file:
        print("⚠️  failures.dirs_file absent — détection ignorée.")
        return None
    dirs_file = Path(dirs_file).expanduser()
    if not dirs_file.exists():
        print(f"⚠️  dirs_file introuvable: {dirs_file} — détection ignorée.")
        return None

    # 4) Commande (pas d'option --out car ton detect_failures.py ne la supporte pas)
    cmd = [str(detect_script), "-f", str(dirs_file)]
    if "std_cap" in bloc and bloc["std_cap"] is not None:
        cmd += ["--std-cap", str(bloc["std_cap"])]
    if "k_mad" in bloc and bloc["k_mad"] is not None:
        cmd += ["--k-mad", str(bloc["k_mad"])]
    if bool(bloc.get("use_rmse", False)):
        cmd += ["--use-rmse"]
    if "evo_cap" in bloc and bloc["evo_cap"] is not None:
        cmd += ["--evo-cap", str(bloc["evo_cap"])]

    print("▶ Détection d'échecs :", " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print("⚠️  detect_failures.py a échoué :", e)
        return None

    # 5) Localisation du CSV de sortie (pas d'argument --out -> on cherche)
    #    Ordre de recherche : bloc['out_csv'] (si fourni) -> default_base_dir/global_failures.csv
    #    -> dirs_file.parent/global_failures.csv -> cfg['csv_dir']/global_failures.csv -> CWD/global_failures.csv
    candidates_csv: List[Path] = []
    out_csv_cfg = bloc.get("out_csv")
    if out_csv_cfg:
        candidates_csv.append(Path(out_csv_cfg).expanduser())
    if default_base_dir:
        candidates_csv.append(Path(default_base_dir) / "global_failures.csv")
    candidates_csv.append(dirs_file.parent / "global_failures.csv")
    if cfg.get("csv_dir"):
        candidates_csv.append(Path(cfg["csv_dir"]).expanduser() / "global_failures.csv")
    candidates_csv.append(Path.cwd() / "global_failures.csv")

    for c in candidates_csv:
        if c.exists():
            print(f"✅ Échecs détectés -> {c}")
            return c

    print("⚠️  Le CSV d'échecs 'global_failures.csv' n'a pas été trouvé dans les emplacements attendus.")
    return None


# ---------------------------- Extraction des impacts (optionnelle) ----------------------------

def run_extraction_if_requested(cfg: dict, failures_csv: Optional[Path]) -> Optional[Path]:
    """
    Si cfg['extraction']['enabled'] est vrai :
      - si overwrite: supprime impacts_*.csv + ancien xlsx
      - lance analyse_impacts.py avec --out, (optionnel) --no-overlap,
        et si failures_csv n'est pas None: --failures-file <csv> (+ --failures-mode si fourni)
    Retourne le dossier où sont écrits impacts_*.csv (parent de out_xlsx), ou None.
    """
    ex = cfg.get("extraction")
    if not isinstance(ex, dict) or not ex.get("enabled", False):
        print("ℹ️  Extraction désactivée (pas de bloc 'extraction' ou enabled=false).")
        return None

    index_file = Path(ex["index_file"]).expanduser()
    out_xlsx   = Path(ex["out_xlsx"]).expanduser()
    no_overlap = bool(ex.get("no_overlap", False))
    overwrite  = bool(ex.get("overwrite", False))

    if not index_file.exists():
        raise SystemExit(f"❌ index_file introuvable: {index_file}")

    target_dir = out_xlsx.parent
    target_dir.mkdir(parents=True, exist_ok=True)

    # Nettoyage si demandé
    if overwrite:
        removed = 0
        for p in target_dir.glob("impacts_*.csv"):
            try:
                p.unlink(); removed += 1
            except Exception:
                pass
        if out_xlsx.exists():
            try:
                out_xlsx.unlink(); removed += 1
            except Exception:
                pass
        if removed:
            print(f"🧹 overwrite: {removed} fichier(s) retiré(s) dans {target_dir}")

    # Lancer analyse_impacts.py
    analyse_script = Path(__file__).parent / "analyse_impacts.py"
    cmd = ["python3", str(analyse_script), str(index_file), "--out", str(out_xlsx)]
    if no_overlap:
        cmd.append("--no-overlap")

    # Paramètres d'analyse additionnels (dont failures_mode)
    analyse_args = cfg.get("analyse_args", {}) or {}
    failures_mode = analyse_args.get("failures_mode")
    if failures_csv:
        cmd += ["--failures-file", str(failures_csv)]
        if failures_mode in {"exclude", "tag", "ignore"}:
            cmd += ["--failures-mode", failures_mode]
        else:
            # défaut raisonnable si non précisé
            cmd += ["--failures-mode", "exclude"]

    print("▶ Extraction (analyse_impacts.py) :", " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print("⚠️  Extraction a échoué :", e)

    return target_dir


# ---------------------------- Chargement des données ----------------------------

def load_impacts(csv_dir: Path) -> Dict[str, pd.DataFrame]:
    """Charge tous les impacts_*.csv et renvoie metric -> DataFrame nettoyé."""
    data: Dict[str, pd.DataFrame] = {}
    for csv in sorted(csv_dir.glob("impacts_*.csv")):
        metric = csv.stem.replace("impacts_", "")
        try:
            df = pd.read_csv(csv)
        except Exception as e:
            print(f"⚠️  Lecture échouée pour {csv}: {e}")
            continue
        # Supprimer la ligne de tête "MOYENNE_metric" si présente
        if df.shape[0] and isinstance(df.loc[0, "test"], str) and df.loc[0, "test"].startswith("MOYENNE_"):
            df = df.iloc[1:].copy()
        # Irel numérique + absolu
        if "Irel(%)" in df.columns:
            I = df["Irel(%)"].astype(str).str.replace(",", ".", regex=False)
            df["Irel(%)"] = pd.to_numeric(I, errors="coerce")
            df["absIrel"] = df["Irel(%)"].abs()
        else:
            df["Irel(%)"] = np.nan
            df["absIrel"] = np.nan
        data[metric] = df.reset_index(drop=True)
    return data


def filter_metrics(data: Dict[str, pd.DataFrame],
                   include: Optional[List[str]] = None,
                   exclude: Optional[List[str]] = None) -> Dict[str, pd.DataFrame]:
    if include:
        data = {m: df for m, df in data.items() if m in include}
    if exclude:
        data = {m: df for m, df in data.items() if m not in exclude}
    return data


def ensure_outdir(out_dir: Path, overwrite_figs: bool = False) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    if overwrite_figs:
        removed = 0
        for p in out_dir.glob("*.png"):
            try:
                p.unlink(); removed += 1
            except Exception:
                pass
        if removed:
            print(f"🧹 figures: {removed} ancien(s) PNG retiré(s) dans {out_dir}")


# ---------------------------- Plots ----------------------------

def plot_bars_per_metric(data: Dict[str, pd.DataFrame], out_dir: Path, topk: Optional[int] = None) -> None:
    for metric, df in data.items():
        if df.empty:
            continue
        sdf = df.sort_values("absIrel", ascending=False)
        if topk:
            sdf = sdf.head(topk)
        fig = plt.figure()
        plt.barh(sdf["test"], sdf["absIrel"])
        plt.xlabel("|Irel|(%)")
        plt.ylabel("Paramètre (test)")
        plt.title(f"Impact relatif par paramètre — {metric}")
        plt.gca().invert_yaxis()
        fig.savefig(out_dir / f"bars_{metric}.png", bbox_inches="tight")
        plt.close(fig)


def _radar_angles(n_axes: int) -> List[float]:
    angles = np.linspace(0, 2*np.pi, n_axes, endpoint=False).tolist()
    angles += angles[:1]
    return angles


def plot_radar_parameters_across_metrics(data: Dict[str, pd.DataFrame], out_dir: Path, topk: int = 8) -> None:
    frames = []
    for metric, df in data.items():
        if df.empty:
            continue
        tmp = df[["test", "absIrel"]].copy()
        tmp = tmp.groupby("test", as_index=False)["absIrel"].mean()
        tmp["metric"] = metric
        frames.append(tmp)
    if not frames:
        return
    big = pd.concat(frames, ignore_index=True)
    pivot = big.pivot_table(index="test", columns="metric", values="absIrel", aggfunc="mean").fillna(0.0)
    pivot["avg"] = pivot.mean(axis=1)
    pivot = pivot.sort_values("avg", ascending=False)
    if topk:
        pivot = pivot.head(topk)

    metrics = list(pivot.columns.drop("avg"))
    angles = _radar_angles(len(metrics))

    fig = plt.figure()
    ax = plt.subplot(111, polar=True)
    plt.title("Radar |Irel| — top paramètres")
    for _, row in pivot.drop(columns=["avg"]).iterrows():
        values = row.values.tolist()
        values += values[:1]
        ax.plot(angles, values)
        ax.fill(angles, values, alpha=0.1)
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(metrics, rotation=0)
    fig.savefig(out_dir / "radar_top_params.png", bbox_inches="tight")
    plt.close(fig)


def _parse_paren_value(s: str) -> float:
    if not isinstance(s, str):
        return float("nan")
    m = re.search(r"\(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\)", s)
    return float(m.group(1)) if m else float("nan")


def plot_best_worst_pair(data: Dict[str, pd.DataFrame], out_dir: Path, metric: str, param: str) -> None:
    if metric not in data or data[metric].empty:
        return
    df = data[metric]
    row = df[df["test"] == param].head(1)
    if row.empty:
        return
    best_s = row["best_config"].iloc[0]
    worst_s = row["worst_config"].iloc[0]
    best_v = _parse_paren_value(best_s)
    worst_v = _parse_paren_value(worst_s)
    labels = ["Best", "Worst"]
    values = [best_v, worst_v]

    fig = plt.figure()
    plt.bar(labels, values)
    plt.ylabel("Valeur de la métrique")
    plt.title(f"{metric} — {param}\nBest vs Worst")
    fig.savefig(out_dir / f"pair_{metric}_{param}.png", bbox_inches="tight")
    plt.close(fig)


def plot_heatmap(data: Dict[str, pd.DataFrame], out_dir: Path, topk_params: Optional[int] = None) -> None:
    frames = []
    for metric, df in data.items():
        if df.empty:
            continue
        tmp = df[["test", "absIrel"]].copy()
        tmp = tmp.groupby("test", as_index=False)["absIrel"].mean()
        tmp["metric"] = metric
        frames.append(tmp)
    if not frames:
        return
    big = pd.concat(frames, ignore_index=True)
    pivot = big.pivot_table(index="test", columns="metric", values="absIrel", aggfunc="mean").fillna(0.0)
    if topk_params:
        pivot = pivot.assign(_avg=pivot.mean(axis=1)).sort_values("_avg", ascending=False).drop(columns=["_avg"]).head(topk_params)

    fig = plt.figure()
    mat = plt.imshow(pivot.values, aspect="auto")
    plt.colorbar(mat, fraction=0.046, pad=0.04)
    plt.yticks(range(pivot.shape[0]), pivot.index.tolist())
    plt.xticks(range(pivot.shape[1]), pivot.columns.tolist(), rotation=45, ha="right")
    plt.title("Heatmap |Irel| (paramètres x métriques)")
    fig.savefig(out_dir / "heatmap_params_metrics.png", bbox_inches="tight")
    plt.close(fig)


def build_auto_categories(data: Dict[str, pd.DataFrame]) -> dict:
    rules = [
        ("Keypoints",      [r"\bKE[_-]", r"keypoint", r"\bkp\b"]),
        ("ICP/Matching",   [r"\bICP\b", r"match", r"registration"]),
        ("Keyframes",      [r"\bKF[_-]", r"keyframe"]),
        ("Localization",   [r"\bLOC[_-]", r"localiz"]),
        ("Pré-traitement", [r"SLAM", r"undistort", r"preproc", r"filter"]),
        ("Mapping",        [r"map", r"voxel", r"grid"]),
        ("Loop closure",   [r"loop"]),
        ("Odometry",       [r"odom"]),
    ]
    def assign_category(name: str) -> str:
        lname = name.lower()
        for cat, pats in rules:
            for pat in pats:
                if re.search(pat, lname, re.IGNORECASE):
                    return cat
        return "Autres"

    params = set()
    for df in data.values():
        if df is None or df.empty:
            continue
        for t in df["test"].dropna().astype(str).tolist():
            params.add(t)
    return {t: assign_category(t) for t in sorted(params)}


def plot_mindmap(data: Dict[str, pd.DataFrame], out_dir: Path, categories_path: Optional[Path] = None) -> None:
    frames = []
    for metric, df in data.items():
        if df.empty:
            continue
        tmp = df[["test", "absIrel"]].copy()
        frames.append(tmp)
    if not frames:
        return
    big = pd.concat(frames, ignore_index=True)
    stats = big.groupby("test", as_index=False)["absIrel"].mean().rename(columns={"absIrel": "avg_abs_irel"})

    cat_map = {}
    if categories_path and categories_path.exists():
        cat_map = json.loads(categories_path.read_text())
    stats["category"] = stats["test"].map(lambda t: cat_map.get(t, "Autres"))

    cats = stats["category"].unique().tolist()
    x_positions = {c: i for i, c in enumerate(cats)}
    max_abs = stats["avg_abs_irel"].max() or 1.0

    fig = plt.figure()
    ax = plt.gca()
    for c in cats:
        x = x_positions[c]; y = 0.0
        ax.scatter([x], [y], s=300)
        ax.text(x, y+0.1, c, ha="center", va="bottom")

    y_offsets = {c: 1}
    for _, r in stats.sort_values(["category", "avg_abs_irel"], ascending=[True, False]).iterrows():
        c = r["category"]; p = r["test"]; s = r["avg_abs_irel"]
        x = x_positions[c]
        y = y_offsets[c]
        size = 100 + 900 * (s / max_abs)
        ax.scatter([x], [y], s=size)
        ax.text(x, y, p, ha="center", va="center")
        ax.plot([x, x], [0.0, y])
        y_offsets[c] += 1

    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(-1, len(cats))
    ax.set_ylim(-0.5, max(y_offsets.values()) + 0.5)
    plt.title("Carte hiérarchique des paramètres (taille ~ impact moyen |Irel|)")
    fig.savefig(out_dir / "mindmap_params.png", bbox_inches="tight")
    plt.close(fig)


# ---------------------------- Main ----------------------------

def main():
    parser = argparse.ArgumentParser(description="Détection d'échecs + extraction + visualisations d'impacts (Irel)")
    parser.add_argument("config", type=str, help="Fichier YAML de configuration")
    args = parser.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())

    # Prépare un "base_dir" par défaut pour placer global_failures.csv si rien n'est fourni :
    # on privilégie le parent de extraction.out_xlsx, sinon csv_dir, sinon cwd.
    base_dir_for_failures = None
    if isinstance(cfg.get("extraction"), dict) and cfg["extraction"].get("out_xlsx"):
        base_dir_for_failures = Path(cfg["extraction"]["out_xlsx"]).expanduser().parent
    elif cfg.get("csv_dir"):
        base_dir_for_failures = Path(cfg["csv_dir"]).expanduser()
    else:
        base_dir_for_failures = Path.cwd()

    # 1) Détection d'échecs (optionnelle)
    failures_csv = run_failures_detection_if_requested(cfg, base_dir_for_failures)

    # 2) Extraction (optionnelle) en passant le failures_file si présent
    csv_dir_from_extraction = run_extraction_if_requested(cfg, failures_csv)

    # 3) Paramètres de plotting
    out_dir = Path(cfg.get("out_dir") or cfg.get("output_dir") or ".").expanduser()
    overwrite_figs = bool(cfg.get("overwrite_figs", False))
    ensure_outdir(out_dir, overwrite_figs=overwrite_figs)

    # Où lire les impacts_*.csv
    if cfg.get("csv_dir"):
        csv_dir = Path(cfg["csv_dir"]).expanduser()
    elif csv_dir_from_extraction:
        csv_dir = csv_dir_from_extraction
    else:
        csv_dir = out_dir.parent

    plots = cfg.get("plots", ["bar", "radar", "heatmap"])
    topk = cfg.get("topk", None)
    pairs = cfg.get("pairs", [])
    pairs_auto_topk = cfg.get("pairs_auto_topk", None)
    categories = Path(cfg["categories"]).expanduser() if cfg.get("categories") else None
    auto_categories = bool(cfg.get("auto_categories", False))
    categories_merge = bool(cfg.get("categories_merge", True))
    include_metrics = cfg.get("include_metrics")
    exclude_metrics = cfg.get("exclude_metrics")

    # 4) Chargement impacts
    if not csv_dir.exists():
        print(f"⚠️  csv_dir introuvable: {csv_dir}")
    data = load_impacts(csv_dir)
    if not data:
        print(f"⚠️  Aucun impacts_*.csv trouvé dans {csv_dir}")
        print("✅ Terminé.")
        return

    data = filter_metrics(data, include_metrics, exclude_metrics)

    # 5) Plots
    if "bar" in plots:
        plot_bars_per_metric(data, out_dir, topk=topk)
    if "radar" in plots:
        plot_radar_parameters_across_metrics(data, out_dir, topk=topk or 8)
    if "pair" in plots and pairs:
        for pair in pairs:
            m = pair.get("metric"); p = pair.get("param")
            if m and p:
                plot_best_worst_pair(data, out_dir, m, p)
    if "pair" in plots and pairs_auto_topk:
        try:
            k = int(pairs_auto_topk)
        except Exception:
            k = None
        if k and k > 0:
            for metric, df in data.items():
                if df.empty:
                    continue
                sdf = df.sort_values("absIrel", ascending=False).head(k)
                for param in sdf["test"].tolist():
                    plot_best_worst_pair(data, out_dir, metric, param)
    if "heatmap" in plots:
        plot_heatmap(data, out_dir, topk_params=topk)

    if "mindmap" in plots:
        auto_path = None
        if auto_categories:
            auto_map = build_auto_categories(data)
            auto_path = out_dir / "categories_auto.json"
            auto_path.write_text(json.dumps(auto_map, indent=2, ensure_ascii=False))
        use_path = categories
        if auto_categories and categories_merge and categories and categories.exists():
            try:
                manual = json.loads(categories.read_text())
            except Exception:
                manual = {}
            merged = dict(auto_map)
            merged.update(manual)  # manual > auto
            merged_path = out_dir / "categories_merged.json"
            merged_path.write_text(json.dumps(merged, indent=2, ensure_ascii=False))
            use_path = merged_path
        elif auto_categories and not categories:
            use_path = auto_path
        plot_mindmap(data, out_dir, use_path)

    print(f"✅ Terminé. Figures dans: {out_dir}")


if __name__ == "__main__":
    main()
