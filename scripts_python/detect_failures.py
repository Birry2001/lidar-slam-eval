#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_failures.py — Détection automatique d'échecs SLAM (par test/config)

Entrées :
  -f / --dirs-file : chemin vers un fichier listant les dossiers de tests (un par ligne)
Options seuils :
  --std-cap 0.15           # cap dur pour std_position_error_max (si dispo)
  --k-mad 3.0              # facteur pour seuil robuste: median + k * MAD
  --use-rmse               # si pas de *_max en EVO, basculer sur RMSE
  --out PATH               # (optionnel) CSV agrégé (défaut: ./global_failures.csv)

Critères d'échec :
  - std_position_error_max au-dessus d'un seuil robuste (ou cap dur)
  - APE/RPE (max ou RMSE) au-dessus d'un seuil robuste
  - Cas “impossibles” (depuis summary_by_metric.xlsx) :
      * overlap_mean/min ≥ 0.999
      * position_error_mean/min == 0
      * std_position_error mean==0 ou max==0
  - Trajectoire constante (dans les .tum) : toutes les positions ~0 ou variance très faible
"""

import argparse
from pathlib import Path
from typing import List, Optional

import numpy as np
import pandas as pd

# ---------- Utils robustes ----------
def median_abs_deviation(x: pd.Series) -> float:
    x = pd.to_numeric(x, errors="coerce").dropna()
    if x.empty:
        return float("nan")
    med = x.median()
    return (x - med).abs().median()

def robust_threshold(x: pd.Series, k: float, min_cap: Optional[float] = None) -> float:
    x = pd.to_numeric(x, errors="coerce").dropna()
    if x.empty:
        return float("inf")
    med = x.median()
    mad = median_abs_deviation(x)
    thr = med + k * (mad if mad > 0 else x.std(ddof=0))
    if min_cap is not None and np.isfinite(min_cap):
        thr = max(thr, float(min_cap))
    return float(thr)

# ---------- Lecture EVO ----------
def read_evo_metrics_local(xlsx_path: Path, prefer_max: bool = True) -> pd.DataFrame:
    if not xlsx_path.exists():
        return pd.DataFrame(columns=["config"])
    try:
        df = pd.read_excel(xlsx_path)
    except Exception:
        return pd.DataFrame(columns=["config"])
    df.columns = [str(c) for c in df.columns]
    keep = ["config"]
    for base in ("APE", "RPE"):
        if prefer_max and f"{base}_max" in df.columns:
            keep.append(f"{base}_max")
        elif f"{base}_RMSE" in df.columns:
            keep.append(f"{base}_RMSE")
    cols = [c for c in keep if c in df.columns]
    if "config" not in cols:
        return pd.DataFrame(columns=["config"])
    out = df[cols].copy()
    # normalise
    ren = {}
    for c in out.columns:
        lc = c.lower()
        if "ape" in lc: ren[c] = "evo_ape"
        if "rpe" in lc: ren[c] = "evo_rpe"
    out = out.rename(columns=ren)
    out["config"] = out["config"].astype(str)
    return out

# ---------- summary_by_metric.xlsx (niveau TEST) ----------
CONFIG_COL_CANDIDATES = ["config","configuration","name","nom","run","Config","Name","Nom","Run"]
MEAN_COL_CANDIDATES   = ["mean","moyenne","avg","value","Mean","Moyenne","Avg","Value","mean [m]","moy","moy (%)","moy[%]"]
MIN_COL_CANDIDATES    = ["min","Min","MIN","min [m]","min(m)","minimum"]

def _norm(s: str) -> str:
    return "".join(ch for ch in str(s).lower() if ch.isalnum())

def _find_col(df: pd.DataFrame, cands: List[str]) -> Optional[str]:
    cols = list(df.columns)
    for c in cands:
        if c in cols: return c
    lower_map = {c.lower(): c for c in cols}
    for c in cands:
        if c.lower() in lower_map: return lower_map[c.lower()]
    # contains
    norm_map = {_norm(c): c for c in cols}
    for c in cands:
        for k,v in norm_map.items():
            if _norm(c) in k:
                return v
    return None

def read_metric_sheet_prefixed(test_dir: Path, metric_name: str, prefix: str) -> pd.DataFrame:
    xl = test_dir / "summary" / "summary_by_metric.xlsx"
    if not xl.exists():
        return pd.DataFrame(columns=["config"])
    try:
        x = pd.ExcelFile(xl)
    except Exception:
        return pd.DataFrame(columns=["config"])
    want = _norm(metric_name)
    target = None
    for s in x.sheet_names:
        if metric_name.lower() in s.lower():
            target = s; break
    if target is None:
        for s in x.sheet_names:
            if want in _norm(s):
                target = s; break
    if target is None:
        return pd.DataFrame(columns=["config"])
    try:
        df = x.parse(target)
    except Exception:
        return pd.DataFrame(columns=["config"])
    cfg = _find_col(df, CONFIG_COL_CANDIDATES) or df.columns[0]
    mean = _find_col(df, MEAN_COL_CANDIDATES)
    minc = _find_col(df, MIN_COL_CANDIDATES)
    out = pd.DataFrame()
    out["config"] = df[cfg].astype(str)
    if mean and mean in df.columns:
        out[f"{prefix}_mean"] = pd.to_numeric(df[mean], errors="coerce")
    if minc and minc in df.columns:
        out[f"{prefix}_min"] = pd.to_numeric(df[minc], errors="coerce")
    return out

# ---------- std_position_error depuis CSV ----------
def find_std_position_error_stats(test_dir: Path) -> pd.DataFrame:
    rows = []
    for csv in test_dir.rglob("*.csv"):
        try:
            df = pd.read_csv(csv)
        except Exception:
            continue
        cols_lower = [c.lower() for c in df.columns]
        if "std_position_error" not in cols_lower:
            continue
        col = df.columns[cols_lower.index("std_position_error")]
        cfg = csv.parent.name
        serie = pd.to_numeric(df[col], errors="coerce").dropna()
        if not serie.empty:
            rows.append({
                "config": cfg,
                "std_pos_err_max": float(serie.max()),
                "std_pos_err_mean": float(serie.mean())
            })
    if not rows:
        return pd.DataFrame(columns=["config","std_pos_err_max","std_pos_err_mean"])
    g = pd.DataFrame(rows).groupby("config", as_index=False).agg({
        "std_pos_err_max": "max",
        "std_pos_err_mean": "mean"
    })
    g["config"] = g["config"].astype(str)
    return g

# ---------- Trajectoires constantes (.tum) ----------
def detect_constant_trajectories(test_dir: Path,
                                 zero_eps: float = 1e-9,
                                 var_eps: float = 1e-6) -> pd.DataFrame:
    """
    Parcourt **/evo/*.tum. Si toutes les positions ~0, ou variances très faibles,
    marque la config comme échec.
    """
    rows = []
    for tum in test_dir.rglob("evo/*.tum"):
        cfg = tum.parent.name  # le dossier parent du .tum est la config
        try:
            arr = []
            with open(tum, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"): 
                        continue
                    parts = line.split()
                    if len(parts) < 8: 
                        continue
                    x,y,z = float(parts[1]), float(parts[2]), float(parts[3])
                    arr.append((x,y,z))
            if not arr:
                continue
            A = np.array(arr, dtype=float)
            xs, ys, zs = A[:,0], A[:,1], A[:,2]
            all_zero = (np.all(np.abs(xs) <= zero_eps) and
                        np.all(np.abs(ys) <= zero_eps) and
                        np.all(np.abs(zs) <= zero_eps))
            very_low_var = (np.var(xs) <= var_eps and np.var(ys) <= var_eps and np.var(zs) <= var_eps)
            if all_zero or very_low_var:
                reason = "trajectory_zero" if all_zero else "trajectory_constant"
                rows.append({
                    "config": cfg, "reason": reason, "metric": "trajectory",
                    "value": 0.0, "threshold": max(zero_eps, var_eps)
                })
        except Exception:
            continue
    if not rows:
        return pd.DataFrame(columns=["config","reason","metric","value","threshold"])
    return pd.DataFrame(rows)

# ---------- Détection par dossier de test ----------
def detect_in_test_dir(test_dir: Path,
                       std_cap: float,
                       k_mad: float,
                       use_rmse_fallback: bool) -> pd.DataFrame:
    failures = []

    # EVO (max prioritaire sinon RMSE)
    evo_df = read_evo_metrics_local(test_dir / "metriques_evo" / "metrics_local.xlsx",
                                    prefer_max=not use_rmse_fallback)

    # std_position_error (depuis CSV)
    std_df = find_std_position_error_stats(test_dir)

    # Feuilles summary_by_metric.xlsx (colonnes préfixées)
    overlap_df   = read_metric_sheet_prefixed(test_dir, "overlap", "overlap")
    pos_err_df   = read_metric_sheet_prefixed(test_dir, "position_error", "position_error")

    # Trajectoires constantes
    traj_df = detect_constant_trajectories(test_dir)

    # Ensemble des configs vues
    configs = set()
    for df in (evo_df, std_df, overlap_df, pos_err_df, traj_df):
        if df is not None and "config" in df.columns:
            configs.update(df["config"].dropna().astype(str).tolist())
    if not configs:
        return pd.DataFrame(columns=["test","config","reason","metric","value","threshold"])

    all_df = pd.DataFrame({"config": sorted(configs)})
    for df in (evo_df, std_df, overlap_df, pos_err_df):
        if df is not None and not df.empty:
            all_df = all_df.merge(df, on="config", how="left")

    # ---- Cas impossibles ----
    # overlap mean/min ~ 1.0 (impossible en pratique)
    for col in ("overlap_mean", "overlap_min"):
        if col in all_df.columns:
            for _, r in all_df.dropna(subset=[col]).iterrows():
                v = float(r[col])
                if v >= 0.999:
                    failures.append({
                        "test": test_dir.name,
                        "config": r["config"],
                        "reason": "overlap_unrealistic",
                        "metric": col,
                        "value": v,
                        "threshold": 0.999
                    })

    # position_error == 0 : SEULEMENT mean==0 (min==0 est permis)
    if "position_error_mean" in all_df.columns:
        for _, r in all_df.dropna(subset=["position_error_mean"]).iterrows():
            v = float(r["position_error_mean"])
            if v == 0.0:
                failures.append({
                    "test": test_dir.name,
                    "config": r["config"],
                    "reason": "position_error_zero_mean",
                    "metric": "position_error_mean",
                    "value": v,
                    "threshold": 0.0
                })

    # (facultatif) si tu disposes d'un "position_error_max" quelque part, tu peux ajouter :
    # if "position_error_max" in all_df.columns:
    #     for _, r in all_df.dropna(subset=["position_error_max"]).iterrows():
    #         v = float(r["position_error_max"])
    #         if v == 0.0:
    #             failures.append({...})


    # std_position_error == 0
    if "std_pos_err_mean" in all_df.columns or "std_pos_err_max" in all_df.columns:
        for _, r in all_df.iterrows():
            vmean = r.get("std_pos_err_mean")
            vmax  = r.get("std_pos_err_max")
            if (pd.notna(vmean) and float(vmean) == 0.0) or (pd.notna(vmax) and float(vmax) == 0.0):
                failures.append({
                    "test": test_dir.name,
                    "config": r["config"],
                    "reason": "std_pos_err_allzero",
                    "metric": "std_position_error",
                    "value": float(0.0 if pd.isna(vmax) else vmax),
                    "threshold": 0.0
                })

    # ---- Seuils robustes ----
    # std_position_error_max
    if "std_pos_err_max" in all_df.columns:
        thr_std = robust_threshold(all_df["std_pos_err_max"], k=k_mad, min_cap=std_cap)
        for _, r in all_df.dropna(subset=["std_pos_err_max"]).iterrows():
            if float(r["std_pos_err_max"]) > thr_std:
                failures.append({
                    "test": test_dir.name,
                    "config": r["config"],
                    "reason": "std_position_error_high",
                    "metric": "std_pos_err_max",
                    "value": float(r["std_pos_err_max"]),
                    "threshold": float(thr_std),
                })

    # EVO threshold — APE uniquement
    if "evo_ape" in all_df.columns:
        thr = robust_threshold(all_df["evo_ape"], k=k_mad, min_cap=None)
        for _, r in all_df.dropna(subset=["evo_ape"]).iterrows():
            if float(r["evo_ape"]) > thr:
                failures.append({
                    "test": test_dir.name,
                    "config": r["config"],
                    "reason": "APE_too_high",
                    "metric": "APE",
                    "value": float(r["evo_ape"]),
                    "threshold": float(thr),
                })

    return pd.DataFrame(failures)

    # Trajectoires constantes (.tum)
    if traj_df is not None and not traj_df.empty:
        for _, r in traj_df.iterrows():
            failures.append({
                "test": test_dir.name,
                "config": str(r["config"]),
                "reason": str(r["reason"]),
                "metric": str(r["metric"]),
                "value": float(r["value"]),
                "threshold": float(r["threshold"]),
            })

    return pd.DataFrame(failures)

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Détection d'échecs SLAM (robuste, par test/config)")
    ap.add_argument("-f","--dirs-file", required=True, help="Fichier listant les dossiers de tests (un par ligne).")
    ap.add_argument("--std-cap", type=float, default=0.15, help="Cap dur minimal pour std_position_error_max.")
    ap.add_argument("--k-mad", type=float, default=3.0, help="Facteur k pour seuil robuste: median + k*MAD.")
    ap.add_argument("--use-rmse", action="store_true", help="Si *_max absent en EVO, utiliser *_RMSE.")
    ap.add_argument("--out", type=str, default=None, help="Chemin du CSV global agrégé (optionnel).")
    args = ap.parse_args()

    dirs = [Path(l.strip()).expanduser() for l in Path(args.dirs_file).read_text().splitlines() if l.strip()]
    all_rows = []
    for d in dirs:
        if not d.exists():
            print(f"⚠️  Dossier introuvable: {d}")
            continue
        df = detect_in_test_dir(d, std_cap=args.std_cap, k_mad=args.k_mad, use_rmse_fallback=args.use_rmse)
        if df.empty:
            (d / "failures.csv").write_text("test,config,reason,metric,value,threshold\n")
            (d / "failures.json").write_text("[]")
            continue
        df.to_csv(d / "failures.csv", index=False)
        (d / "failures.json").write_text(df.to_json(orient="records", indent=2))
        all_rows.append(df)

    # Agrégé global
    if all_rows:
        global_df = pd.concat(all_rows, ignore_index=True)
        out_path = Path(args.out).expanduser() if args.out else Path.cwd() / "global_failures.csv"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        global_df.to_csv(out_path, index=False)
        print(f"✅ global_failures.csv écrit: {out_path} ({global_df.shape[0]} ligne(s))")
    else:
        print("✅ Aucun échec détecté (ou sources manquantes).")

if __name__ == "__main__":
    main()
