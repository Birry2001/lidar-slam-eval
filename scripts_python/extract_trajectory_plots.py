#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import subprocess
import sys
import time
from pathlib import Path

EVO_TIMEOUT = 60  # secondes

def log(msg: str):
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | {msg}", flush=True)

def sanitize_tum(path: Path):
    """
    Ne garde que les lignes à 8 champs et enlève les espaces superflus.
    """
    lines = []
    for ln in path.read_text().splitlines():
        parts = ln.strip().split()
        if len(parts) == 8:
            lines.append(" ".join(parts))
    path.write_text("\n".join(lines) + "\n")

def plot_config_trajectories(cfg_dir: Path):
    """
    Pour chaque config, cherche tous les .tum dans */evo, sanitizes-les,
    puis appelle `evo_traj tum <f1> <f2> ... --save_plot trajectoires/<config>.png`.
    """
    tum_files = []
    # recense tous les .tum valides
    for evo_sub in sorted(cfg_dir.glob("*/evo")):
        for tum in evo_sub.glob("*.tum"):
            sanitize_tum(tum)
            tum_files.append(str(tum))
    if not tum_files:
        return None, None  # pas de tum → skip

    out_dir = cfg_dir / "trajectoires"
    out_dir.mkdir(exist_ok=True)
    out_png = out_dir / f"{cfg_dir.name}.png"

    cmd = ["evo_traj", "tum"] + tum_files + ["--save_plot", str(out_png)]
    log(f"→ Lancement de: {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True, timeout=EVO_TIMEOUT)
        return True, f"Graphique écrit dans {out_png}"
    except subprocess.TimeoutExpired:
        return False, f"Timeout après {EVO_TIMEOUT}s"
    except subprocess.CalledProcessError as e:
        return False, f"evo_traj a échoué (rc={e.returncode})"

def main():
    parser = argparse.ArgumentParser(
        description="Génère un plot des trajectoires TUM pour chaque configuration."
    )
    parser.add_argument(
        "-d", "--dirs-file", required=True,
        help="Fichier .txt listant (une par ligne) les répertoires de config"
    )
    args = parser.parse_args()

    fd = Path(args.dirs_file)
    if not fd.is_file():
        print(f"❌ Fichier introuvable : {fd}", file=sys.stderr)
        sys.exit(1)

    succ, fail, skipped = [], [], []
    log("📖 Lecture du fichier de configurations")
    for line in fd.read_text().splitlines():
        cfg = Path(line.strip())
        log(f"▶ Traitement de la config : {cfg}")
        if not cfg.is_dir():
            fail.append((str(cfg), "Répertoire introuvable"))
            continue
        ok, msg = plot_config_trajectories(cfg)
        if ok is True:
            succ.append((str(cfg), msg))
        elif ok is False:
            fail.append((str(cfg), msg))
        else:
            skipped.append(str(cfg))

    # --- Bilan ---
    print("\n=== Graphiques générés ===")
    if succ:
        for path, info in succ:
            print(f"✔ {path}: {info}")
    else:
        print("Aucun graphique généré.")

    if skipped:
        print("\n=== Configurations ignorées (sans .tum) ===")
        for path in skipped:
            print(f"— {path}")

    print("\n=== Erreurs rencontrées ===")
    if fail:
        for path, err in fail:
            print(f"✖ {path}: {err}")
    else:
        print("Aucune erreur.")

if __name__ == "__main__":
    main()
