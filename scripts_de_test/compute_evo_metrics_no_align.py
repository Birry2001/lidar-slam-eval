#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compute_evo_metrics_no_align.py

- Parcourt les dossiers listés (ou dirs_1.txt par défaut).
- Cherche chaque sous-dossier "evo" contenant un fichier .tum (estimation).
- Aligne automatiquement la GT et l'estimation via un t_offset détecté (diff. des 1ers timestamps).
- Lance evo_ape / evo_rpe avec --t_offset auto et un --t_max_diff plus tolérant.
- Parse les stats (rmse, mean, median, std, min, max) et génère :
    - metriques_evo/metrics_local.xlsx (par dossier de paramètres)
    - all_metrics.csv / all_metrics.xlsx (globaux)
"""

import argparse
import os
import glob
import re
import subprocess
import pandas as pd
from typing import List, Dict, Optional, Tuple

# === CONFIGURATION PAR DÉFAUT ===
DELTA = "0.5"
DELTA_UNIT = "m"
DIRS_FILE_DEFAULT = "dirs_1.txt"

# marge d'association de timestamps (plus souple que 0.01s par défaut d'evo)
T_MAX_DIFF_S = "0.25"   # ajuste si besoin (0.1–0.5 raisonnable)

STAT_RE = re.compile(
    r"^\s*(?P<label>rmse|mean|median|std|min|max)\s*[:=]?\s*(?P<value>[0-9.+\-eE]+)",
    re.IGNORECASE
)
KEY_MAP = {
    'rmse': 'RMSE',
    'mean': 'mean',
    'median': 'median',
    'std': 'std',
    'min': 'min',
    'max': 'max'
}

# Certaines versions d'evo utilisent --t_max_diff ; on standardise ici
EVO_SYNC_ARGS = ["--t_max_diff", T_MAX_DIFF_S]


def first_numeric_timestamp(tum_path: str) -> Optional[float]:
    """
    Retourne le premier timestamp (float) trouvé dans un fichier TUM.
    Ignore commentaires / lignes vides.
    """
    if not os.path.isfile(tum_path):
        return None
    with open(tum_path, "r") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            # On attend la forme TUM: t x y z qx qy qz qw
            # On prend la première valeur si elle ressemble à un float
            part = s.split()
            if not part:
                continue
            try:
                return float(part[0])
            except ValueError:
                continue
    return None


def compute_time_offset(ref_tum: str, est_tum: str) -> Optional[float]:
    """
    Calcule t_offset = t_ref0 - t_est0 (diff. des 1ers timestamps).
    Si un des timestamps manque, retourne None.
    """
    t_ref0 = first_numeric_timestamp(ref_tum)
    t_est0 = first_numeric_timestamp(est_tum)
    if t_ref0 is None or t_est0 is None:
        return None
    return t_ref0 - t_est0


def run_and_parse(cmd: List[str], name: str, typ: str) -> Dict[str, float]:
    """Lance la commande cmd et parse sa sortie pour extraire les statistiques evo."""
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, check=True)
        output = res.stdout
    except subprocess.CalledProcessError as e:
        print(f"⚠️ evo_{typ} a échoué pour {name}")
        output = e.stdout or ""

    stats: Dict[str, float] = {}
    for line in (output or "").splitlines():
        m = STAT_RE.match(line)
        if m:
            label = m.group('label').lower()
            try:
                stats[KEY_MAP[label]] = float(m.group('value'))
            except Exception:
                pass
    return stats


def process_param_dir(param_dir: str, ref_tum: str) -> List[Dict]:
    """
    Traite un dossier de paramètre : recherche les evo/, exécute APE/RPE, retourne la liste des records.
    - Détecte t_offset entre la GT et le .tum estimé.
    - Passe --t_offset et --t_max_diff à evo_*.
    """
    evo_dirs = [
        p for p in glob.glob(os.path.join(param_dir, "**", "evo"), recursive=True)
        if os.path.isdir(p)
    ]
    records: List[Dict] = []
    met_dir = os.path.join(param_dir, "metriques_evo")
    os.makedirs(met_dir, exist_ok=True)

    for evo in sorted(evo_dirs):
        cfg = os.path.basename(os.path.dirname(evo))
        tum_files = glob.glob(os.path.join(evo, "*.tum"))
        if not tum_files:
            print(f"⚠️ Pas de .tum dans {evo}, on skip.")
            continue

        # On prend le premier .tum trouvé
        tum = tum_files[0]

        # Détection du décalage temporel
        t_offset = compute_time_offset(ref_tum, tum)
        offset_args: List[str] = []
        if t_offset is not None:
            offset_args = ["--t_offset", f"{t_offset:.6f}"]
            print(f"⏱  {cfg}: t_offset auto = {t_offset:.6f}s (ref - est)")

        # APE
        ape_plot = os.path.join(evo, f"{cfg}_ape.png")
        ape_cmd = ["evo_ape", "tum", ref_tum, tum, *offset_args, *EVO_SYNC_ARGS, "--save_plot", ape_plot]
        ape_stats = run_and_parse(ape_cmd, cfg, "ape")

        # RPE
        rpe_plot = os.path.join(evo, f"{cfg}_rpe.png")
        rpe_cmd = [
            "evo_rpe", "tum", ref_tum, tum,
            "--delta", DELTA, "--delta_unit", DELTA_UNIT,
            *offset_args, *EVO_SYNC_ARGS,
            "--save_plot", rpe_plot
        ]
        rpe_stats = run_and_parse(rpe_cmd, cfg, "rpe")

        # Si aucune stat, on tente un fallback : ré-essayer SANS offset (rare mais utile si les timestamps sont déjà alignés)
        if not ape_stats and not rpe_stats and offset_args:
            print(f"↻  Fallback {cfg}: retry sans --t_offset")
            ape_stats = run_and_parse(
                ["evo_ape", "tum", ref_tum, tum, *EVO_SYNC_ARGS, "--save_plot", ape_plot],
                cfg, "ape"
            )
            rpe_stats = run_and_parse(
                ["evo_rpe", "tum", ref_tum, tum, "--delta", DELTA, "--delta_unit", DELTA_UNIT,
                 *EVO_SYNC_ARGS, "--save_plot", rpe_plot],
                cfg, "rpe"
            )

        if not ape_stats and not rpe_stats:
            print(f"⚠️ Pas de stats récupérées pour {cfg}, ignoré.")
            continue

        rec = {"config": cfg}
        rec.update({f"APE_{k}": v for k, v in ape_stats.items()})
        rec.update({f"RPE_{k}": v for k, v in rpe_stats.items()})
        records.append(rec)

    # Écriture locale
    if records:
        df_local = pd.DataFrame(records)
        local_xlsx = os.path.join(met_dir, "metrics_local.xlsx")
        df_local.to_excel(local_xlsx, index=False)
        print(f"✅ metrics_local.xlsx généré dans {met_dir}")

    return records


def main():
    parser = argparse.ArgumentParser(
        description="Compute EVO metrics across multiple evo/ subfolders (auto time-offset)."
    )
    parser.add_argument(
        "-d", "--dirs-file",
        default=DIRS_FILE_DEFAULT,
        help="Chemin vers le fichier listant les dossiers à traiter (un par ligne)."
    )
    parser.add_argument(
        "-r", "--ref-tum",
        required=True,
        help="Chemin vers le fichier TUM de référence pour EVO (APE/RPE)."
    )
    args = parser.parse_args()

    if not os.path.isfile(args.dirs_file):
        print(f"❌ Fichier introuvable : {args.dirs_file}")
        return
    if not os.path.isfile(args.ref_tum):
        print(f"❌ Fichier de référence introuvable : {args.ref_tum}")
        return

    with open(args.dirs_file) as f:
        dirs = [l.strip() for l in f if l.strip()]

    all_records: List[Dict] = []
    for d in dirs:
        print(f"\n=== Traitement de {d} ===")
        recs = process_param_dir(d, args.ref_tum)
        all_records.extend(recs)

    if all_records:
        df = pd.DataFrame(all_records)
        df.to_csv("all_metrics.csv", index=False)
        df.to_excel("all_metrics.xlsx", index=False)
        print("✅ all_metrics.csv et all_metrics.xlsx générés.")


if __name__ == "__main__":
    main()
