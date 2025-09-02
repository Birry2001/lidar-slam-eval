#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import subprocess
import time
from pathlib import Path
import sys

# délai max (en secondes) pour evo_traj
EVO_TIMEOUT = 120

def log(msg: str):
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | {msg}", flush=True)

def extract_for_value(val_dir: Path) -> (bool, str):
    name = val_dir.name
    bag_folder = val_dir / "ros_bag" / "all_bag"
    evo_dir    = val_dir / "evo"
    evo_dir.mkdir(parents=True, exist_ok=True)
    tmp_tum    = bag_folder / "slam_odom.tum"
    final_tum  = evo_dir / f"{name}.tum"

    log(f"🔍 Début extraction pour « {name} »")
    if not bag_folder.is_dir():
        return False, f"Pas de dossier bag: {bag_folder}"
    db3 = next(bag_folder.glob("*.db3"), None)
    if not db3:
        return False, f"Pas de .db3 dans {bag_folder}"
    log(f"→ Trouvé .db3 : {db3.name}")

    # suppression ancien temporaire
    if tmp_tum.exists():
        tmp_tum.unlink()
        log(f"→ Supprimé ancien temporaire {tmp_tum}")

    # commande complète sous bash pour sourcer l'env et lancer evo_traj
    bash_cmd = (
        f"source ~/test_ws/install/setup.bash && "
        f"evo_traj bag2 {bag_folder} /slam_odom --save_as_tum"
    )
    log(f"→ Exécution: {bash_cmd}")
    try:
        proc = subprocess.run(
            bash_cmd,
            shell=True,
            cwd=bag_folder,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            executable="/bin/bash",
            timeout=EVO_TIMEOUT,
            text=True
        )
    except subprocess.TimeoutExpired:
        return False, f"Timeout après {EVO_TIMEOUT}s"
    if proc.returncode != 0:
        err = proc.stderr.strip().splitlines()[-1] if proc.stderr else f"rc={proc.returncode}"
        log(f"❌ evo_traj a échoué ({err!r})")
        return False, f"evo_traj a échoué : {err!r}"
    log("✅ evo_traj terminé")

    # déplacer et renommer
    if not tmp_tum.exists():
        return False, "Aucun .tum généré"
    try:
        tmp_tum.rename(final_tum)
        log(f"🚚 Fichier déplacé vers {final_tum}")
    except Exception as e:
        return False, f"Erreur déplacement: {e}"
    return True, f"Trajectoire extraite: {final_tum}"

def main():
    p = argparse.ArgumentParser(
        description="Extrait TUM depuis ROS2-bags et range dans evo/<valeur>.tum"
    )
    p.add_argument("-d", "--dirs-file", required=True,
                   help="Fichier .txt listant les dossiers de config (un par ligne)")
    args = p.parse_args()

    df = Path(args.dirs_file)
    if not df.is_file():
        print(f"❌ Fichier introuvable : {df}", file=sys.stderr)
        sys.exit(1)

    succ, fail = [], []
    log("📖 Lecture du fichier de configurations")
    for line in df.read_text().splitlines():
        cfg_root = Path(line.strip())
        log(f"▶ Traitement de la config : {cfg_root}")
        if not cfg_root.is_dir():
            fail.append((str(cfg_root), "Dossier introuvable"))
            continue
        for val_dir in sorted(cfg_root.iterdir()):
            if not val_dir.is_dir(): continue
            ok, msg = extract_for_value(val_dir)
            (succ if ok else fail).append((str(val_dir), msg))

    # bilan
    print("\n=== Trajectoires extraites ===")
    if succ:
        for _, m in succ: print("✔", m)
    else:
        print("Aucune trajectoire extraite.")

    print("\n=== Erreurs rencontrées ===")
    if fail:
        for pth, err in fail: print("✖", pth, ":", err)
    else:
        print("Aucune erreur.")

if __name__ == "__main__":
    main()
