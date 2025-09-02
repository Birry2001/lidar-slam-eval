#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
from pathlib import Path
import shutil

def collect_csvs(config_dir: Path) -> bool:
    """
    Dans config_dir, cherche tous les CSV sous **/fichiers_csv/*.csv,
    crée config_dir/metriques_confidence et copie chacun d'eux dedans.
    """
    # 1) Trouver les CSV
    csv_files = list(config_dir.glob("**/fichiers_csv/*.csv"))
    if not csv_files:
        print(f"❌ Aucun fichier .csv trouvé dans {config_dir}/fichiers_csv")
        return False

    # 2) Préparer le dossier de destination
    target = config_dir / "metriques_confidence"
    target.mkdir(exist_ok=True)

    # 3) Copier chaque CSV
    for csv in csv_files:
        dest = target / csv.name
        shutil.copy2(csv, dest)
        print(f"  • Copié : {csv} → {dest}")

    return True

def main():
    parser = argparse.ArgumentParser(
        description="Copie tous les CSV de confiance depuis les sous-dossiers fichiers_csv/<valeur> vers metriques_confidence/"
    )
    parser.add_argument(
        "-d", "--dirs-file",
        required=True,
        help="Chemin vers le fichier texte listant (une ligne = un dossier de config)"
    )
    args = parser.parse_args()

    dirs_file = Path(args.dirs_file)
    if not dirs_file.is_file():
        print(f"❌ Fichier introuvable : {dirs_file}", file=sys.stderr)
        sys.exit(1)

    # Pour chaque chemin de config dans le *.txt
    for line in dirs_file.read_text().splitlines():
        cfg = Path(line.strip())
        if not cfg.is_dir():
            print(f"❌ Dossier introuvable : {cfg}")
            continue

        print(f"\n=== Traitement de {cfg} ===")
        success = collect_csvs(cfg)
        if success:
            print("✔️ Tous les CSV ont été copiés dans metriques_confidence.")
        else:
            print("⚠️ Aucun CSV copié.")

if __name__ == "__main__":
    main()
