#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
collect_summaries.py

Pour chaque répertoire de configuration listé dans un fichier dirs.txt,
ce script :
 1. Vérifie que le sous-dossier 'metriques_confidence' existe et contient au moins un .csv.
 2. Crée (ou nettoie) le sous-dossier 'summary'.
 3. Lance summarize_by_metric.py en prenant en entrée metriques_confidence
    et en sortie summary.
En cas d'erreur (dossier manquant ou vide, échec de summarize_by_metric), le script
interrompt immédiatement et affiche un message d'erreur.
Usage :
    ./collect_summaries.py -d dirs.txt
"""

import argparse
import subprocess
import sys
from pathlib import Path

def fatal(msg: str):
    print(f"❌ ERREUR: {msg}", file=sys.stderr)
    sys.exit(1)

def info(msg: str):
    print(f"[INFO] {msg}")

def main():
    parser = argparse.ArgumentParser(
        description="Collecte et génère les résumés 'summary' à partir des métriques de confiance."
    )
    parser.add_argument(
        "-d", "--dirs-file", required=True,
        help="Fichier listant (une par ligne) les chemins vers les dossiers de configuration."
    )
    args = parser.parse_args()

    dirs_file = Path(args.dirs_file)
    if not dirs_file.is_file():
        fatal(f"Fichier non trouvé : {dirs_file}")

    # Lecture des chemins de config
    config_paths = [Path(l.strip()) for l in dirs_file.read_text().splitlines() if l.strip()]
    if not config_paths:
        fatal(f"Aucun chemin valide dans {dirs_file}")

    for cfg in config_paths:
        info(f"→ Traitement de la configuration : {cfg}")
        if not cfg.is_dir():
            fatal(f"Répertoire introuvable : {cfg}")

        met_conf = cfg / "metriques_confidence"
        if not met_conf.is_dir():
            fatal(f"Le dossier 'metriques_confidence' est absent pour {cfg}")
        csv_files = list(met_conf.glob("*.csv"))
        if not csv_files:
            fatal(f"Aucun fichier CSV trouvé dans {met_conf}")

        summary_dir = cfg / "summary"
        # (re)création du dossier summary
        if summary_dir.exists():
            info(f"• Nettoyage de {summary_dir}")
            for f in summary_dir.iterdir():
                f.unlink()
        else:
            info(f"• Création de {summary_dir}")
            summary_dir.mkdir(parents=True)

        # Appel à summarize_by_metric.py
        cmd = [
            str(Path(__file__).parent / "summarize_by_metric.py"),
            str(met_conf),
            str(summary_dir)
        ]
        info(f"▶ Lancement : {' '.join(cmd)}")
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            fatal(f"summarize_by_metric.py a échoué (rc={e.returncode}) pour {cfg}")

        info(f"✔ Résumé généré dans {summary_dir}\n")

    info("Tous les résumés ont été générés avec succès.")

if __name__ == "__main__":
    main()
