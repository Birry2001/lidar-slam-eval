#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compute_evo_metrics_auto_align.py

- Auto t_offset (à partir de la 1re timestamp de REF et EST)
- Alignement (Umeyama) activé
- Nettoyage des anciens fichiers (plots/logs/metrics) avant chaque run
- Backend matplotlib non-GUI (MPLBACKEND=Agg)
- Retry sans plots si timeout
"""

from __future__ import annotations
import argparse
import glob
import os
import re
import shutil
import subprocess
from typing import Dict, List
import pandas as pd

# ================== REGLAGES ==================
DIRS_FILE_DEFAULT = "dirs.txt"
DELTA = "0.5"
DELTA_UNIT = "m"
TMAX_DIFF = "0.25"
TIMEOUT_S = 180  # timeout evo_* (s)

# ==============================================
STAT_RE = re.compile(r"^\s*(rmse|mean|median|std|min|max)\s*[:=]?\s*([0-9.+\-eE]+)", re.I)
KEY_MAP = {'rmse': 'RMSE', 'mean': 'mean', 'median': 'median', 'std': 'std', 'min': 'min', 'max': 'max'}


def read_first_timestamp(tum_path: str) -> float | None:
    try:
        with open(tum_path, "r") as f:
            for line in f:
                if not line or line[0] == '#':
                    continue
                parts = line.strip().split()
                if len(parts) >= 8:
                    return float(parts[0])
    except Exception:
        return None
    return None


def auto_time_offset(ref_tum: str, est_tum: str) -> float:
    tref = read_first_timestamp(ref_tum)
    test = read_first_timestamp(est_tum)
    if tref is None or test is None:
        return 0.0
    return round(tref - test, 6)


def ensure_dir(p: str) -> None:
    os.makedirs(p, exist_ok=True)


def clean_outputs_in_evo_dir(evo_dir: str) -> None:
    """Supprime tout ce qui peut faire bloquer evo (plots/logs/metrics) mais conserve *.tum."""
    patterns = [
        "*.png", "*.pdf", "*.svg",
        "*.log", "*.txt",
        "*.csv", "*.xlsx", "*.json",
    ]
    for pat in patterns:
        for fp in glob.glob(os.path.join(evo_dir, pat)):
            # Ne jamais supprimer les trajectoires .tum
            if fp.lower().endswith(".tum"):
                continue
            try:
                os.remove(fp)
            except Exception:
                pass


def run_and_parse(cmd: List[str], name: str, kind: str, timeout_s: int, log_path: str) -> Dict[str, float]:
    """Exécute evo_* avec timeout, sauvegarde stdout, parse stats.
       Force un backend non-GUI et retry sans plots si ça bloque."""
    print(f"▶ [{kind}] {name} :: {' '.join(cmd)}")

    env = os.environ.copy()
    env.setdefault("MPLBACKEND", "Agg")
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    env.setdefault("LC_ALL", "C")
    env.setdefault("LANG", "C")

    def _run(the_cmd: List[str], note: str) -> str:
        try:
            res = subprocess.run(
                the_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_s,
                check=False,
                env=env
            )
            out = res.stdout or ""
            if res.returncode != 0:
                print(f"⚠️ evo_{kind} code={res.returncode} pour {name} ({note})")
            return out
        except subprocess.TimeoutExpired:
            print(f"⏳ [{kind}] {name} a dépassé {timeout_s}s -> on passe.")
            return ""

    # premier essai tel quel
    out = _run(cmd, "avec plots")

    # si vide et qu'on avait des plots, on retente sans --save_plot <path>
    if (not out.strip()) and "--save_plot" in cmd:
        cmd_no_plot: List[str] = []
        skip_next = False
        for c in cmd:
            if skip_next:
                skip_next = False
                continue
            if c == "--save_plot":
                skip_next = True
                continue
            cmd_no_plot.append(c)
        print(f"↻ [{kind}] {name} :: retry sans plots")
        out = _run(cmd_no_plot, "sans plots")

    # log
    try:
        with open(log_path, "w") as f:
            f.write(out)
    except Exception:
        pass

    # parse stats
    stats: Dict[str, float] = {}
    for line in out.splitlines():
        m = STAT_RE.match(line)
        if m:
            label = m.group(1).lower()
            stats[KEY_MAP[label]] = float(m.group(2))
    return stats


def process_param_dir(param_dir: str, ref_tum: str, timeout_s: int) -> List[Dict]:
    """Traite un dossier de paramètre : pour chaque evo/ contenant un .tum, lance APE/RPE avec auto t_offset & align."""
    records: List[Dict] = []

    # Nettoyage global de metriques_evo au niveau param_dir (mais garder les evo/.tum)
    met_dir = os.path.join(param_dir, "metriques_evo")
    if os.path.isdir(met_dir):
        # on supprime le dossier complet (il ne contient que des sorties)
        try:
            shutil.rmtree(met_dir)
        except Exception:
            pass
    ensure_dir(met_dir)

    evo_dirs = [
        p for p in glob.glob(os.path.join(param_dir, "**", "evo"), recursive=True)
        if os.path.isdir(p)
    ]

    for evo in sorted(evo_dirs):
        cfg = os.path.basename(os.path.dirname(evo))
        tum_files = [t for t in glob.glob(os.path.join(evo, "*.tum")) if os.path.isfile(t)]
        if not tum_files:
            print(f"⚠️ Pas de .tum dans {evo}, on skip.")
            continue
        tum = tum_files[0]

        # Nettoyer ce evo/ (plots/logs/metrics), conserver *.tum
        clean_outputs_in_evo_dir(evo)

        # calcul t_offset
        t_off = auto_time_offset(ref_tum, tum)
        print(f"⏱  {cfg}: t_offset auto = {t_off:.6f}s")

        # chemins de plots & logs (qui seront recréés)
        ape_plot = os.path.join(evo, f"{cfg}_ape.png")
        rpe_plot = os.path.join(evo, f"{cfg}_rpe.png")
        ape_log = os.path.join(evo, f"{cfg}_ape.log")
        rpe_log = os.path.join(evo, f"{cfg}_rpe.log")

        # par sécurité, s’ils existent encore, on les enlève
        for fp in (ape_plot, rpe_plot, ape_log, rpe_log):
            try:
                if os.path.exists(fp):
                    os.remove(fp)
            except Exception:
                pass

        # APE (align + t_offset + t_max_diff)
        ape_cmd = [
            "evo_ape", "tum", ref_tum, tum,
            "--t_offset", str(t_off),
            "--t_max_diff", TMAX_DIFF,
            "--align",
            "--save_plot", ape_plot
        ]
        ape_stats = run_and_parse(ape_cmd, cfg, "ape", timeout_s, ape_log)

        # RPE
        rpe_cmd = [
            "evo_rpe", "tum", ref_tum, tum,
            "--t_offset", str(t_off),
            "--t_max_diff", TMAX_DIFF,
            "--delta", DELTA, "--delta_unit", DELTA_UNIT,
            "--align",
            "--save_plot", rpe_plot
        ]
        rpe_stats = run_and_parse(rpe_cmd, cfg, "rpe", timeout_s, rpe_log)

        if not ape_stats and not rpe_stats:
            print(f"⚠️ Pas de stats récupérées pour {cfg}, ignoré.")
            continue

        rec = {"param_dir": param_dir, "config": cfg, "t_offset": t_off}
        rec.update({f"APE_{k}": v for k, v in ape_stats.items()})
        rec.update({f"RPE_{k}": v for k, v in rpe_stats.items()})
        records.append(rec)

    # Écriture locale
    if records:
        df_local = pd.DataFrame(records)
        local_xlsx = os.path.join(met_dir, "metrics_local.xlsx")
        try:
            if os.path.exists(local_xlsx):
                os.remove(local_xlsx)
        except Exception:
            pass
        df_local.to_excel(local_xlsx, index=False)
        print(f"✅ metrics_local.xlsx généré dans {met_dir}")

    return records


def main():
    parser = argparse.ArgumentParser(description="Compute EVO metrics (auto time offset + alignment) sur des dossiers.")
    parser.add_argument("-d", "--dirs-file", default=DIRS_FILE_DEFAULT,
                        help="Fichier listant les dossiers param_dir (un par ligne).")
    parser.add_argument("-r", "--ref-tum", required=True, help="Fichier TUM de référence.")
    parser.add_argument("--timeout", type=int, default=TIMEOUT_S, help="Timeout pour chaque evo_* (s).")
    args = parser.parse_args()

    if not os.path.isfile(args.dirs_file):
        print(f"❌ Fichier introuvable : {args.dirs_file}")
        return
    if not os.path.isfile(args.ref_tum):
        print(f"❌ Fichier de référence introuvable : {args.ref_tum}")
        return

    # Lire la liste
    with open(args.dirs_file, "r") as f:
        param_dirs = [l.strip() for l in f if l.strip()]

    all_records: List[Dict] = []
    for d in param_dirs:
        print(f"\n=== Traitement de {d} ===")
        recs = process_param_dir(d, args.ref_tum, args.timeout)
        all_records.extend(recs)

    # exports globaux (écrasent)
    if all_records:
        df = pd.DataFrame(all_records)
        for fp in ("all_metrics.csv", "all_metrics.xlsx"):
            try:
                if os.path.exists(fp):
                    os.remove(fp)
            except Exception:
                pass
        df.to_csv("all_metrics.csv", index=False)
        df.to_excel("all_metrics.xlsx", index=False)
        print("✅ all_metrics.csv et all_metrics.xlsx générés.")


if __name__ == "__main__":
    main()
