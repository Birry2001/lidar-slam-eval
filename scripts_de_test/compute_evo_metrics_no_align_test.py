#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compute EVO metrics (APE & RPE) pour chaque dossier listé dans -d.
"""
import argparse, subprocess, glob, os, re, pandas as pd

STAT_RE = re.compile(r"^\s*(?P<label>rmse|mean|median|std|min|max)\s*[:=]?\s*(?P<value>[0-9.+\-eE]+)", re.IGNORECASE)
KEY_MAP = {'rmse':'RMSE','mean':'mean','median':'median','std':'std','min':'min','max':'max'}

def run_and_parse(cmd):
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    stats={}
    for ln in res.stdout.splitlines():
        m = STAT_RE.match(ln)
        if m: stats[KEY_MAP[m.group('label').lower()]] = float(m.group('value'))
    return stats

def process_dir(d):
    evo_dirs = [p for p in glob.glob(f"{d}/**/evo", recursive=True) if os.path.isdir(p)]
    records=[]
    for evo in evo_dirs:
        cfg=os.path.basename(os.path.dirname(evo))
        tum=glob.glob(f"{evo}/*.tum")[0]
        # APE
        ape=run_and_parse(["evo_ape","tum","/…/par_defaut.tum", tum, "--save_plot",f"{evo}/{cfg}_ape.png"])
        # RPE
        rpe=run_and_parse(["evo_rpe","tum","/…/par_defaut.tum", tum,"--delta","0.5","--delta_unit","m","--save_plot",f"{evo}/{cfg}_rpe.png"])
        rec={"config":cfg}
        rec.update({f"APE_{k}":v for k,v in ape.items()})
        rec.update({f"RPE_{k}":v for k,v in rpe.items()})
        records.append(rec)
    if records:
        pd.DataFrame(records).to_excel(f"{d}/metriques_evo/metrics_local.xlsx", index=False)
    return records

def main():
    p=argparse.ArgumentParser()
    p.add_argument("-d","--dirs-file",required=True, help="fichier listant dirs")
    args=p.parse_args()
    with open(args.dirs_file) as f:
        ds=[l.strip() for l in f if l.strip()]
    allr=[]
    for d in ds:
        print("===",d)
        allr+=process_dir(d)
    if allr:
        df=pd.DataFrame(allr)
        df.to_csv("all_metrics.csv",index=False)
        df.to_excel("all_metrics.xlsx",index=False)
        print("✅ EVO metrics générés.")

if __name__=="__main__":
    main()






#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse, os, glob, re, subprocess, pandas as pd

# … (STAT_RE, KEY_MAP identiques)

STAT_RE = re.compile(r"^\s*(?P<label>rmse|mean|median|std|min|max)\s*[:=]?\s*(?P<value>[0-9.+\-eE]+)", re.IGNORECASE)
KEY_MAP = {'rmse':'RMSE','mean':'mean','median':'median','std':'std','min':'min','max':'max'}

def run_and_parse(cmd, name, typ):
    # idem
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    stats={}
    for ln in res.stdout.splitlines():
        m = STAT_RE.match(ln)
        if m: stats[KEY_MAP[m.group('label').lower()]] = float(m.group('value'))
    return stats

def process_param_dir(param_dir):
    # idem
    evo_dirs = [p for p in glob.glob(f"{d}/**/evo", recursive=True) if os.path.isdir(p)]
    records=[]
    for evo in evo_dirs:
        cfg=os.path.basename(os.path.dirname(evo))
        tum=glob.glob(f"{evo}/*.tum")[0]
        # APE
        ape=run_and_parse(["evo_ape","tum","/…/par_defaut.tum", tum, "--save_plot",f"{evo}/{cfg}_ape.png"])
        # RPE
        rpe=run_and_parse(["evo_rpe","tum","/…/par_defaut.tum", tum,"--delta","0.5","--delta_unit","m","--save_plot",f"{evo}/{cfg}_rpe.png"])
        rec={"config":cfg}
        rec.update({f"APE_{k}":v for k,v in ape.items()})
        rec.update({f"RPE_{k}":v for k,v in rpe.items()})
        records.append(rec)
    if records:
        pd.DataFrame(records).to_excel(f"{d}/metriques_evo/metrics_local.xlsx", index=False)
    return records

def main():
    p = argparse.ArgumentParser()
    p.add_argument('-d','--dirs-file', required=True, help="Fichier listant les répertoires à traiter")
    args = p.parse_args()

    if not os.path.isfile(args.dirs_file):
        print(f"❌ Pas de {args.dirs_file}")
        return

    with open(args.dirs_file) as f:
        dirs = [l.strip() for l in f if l.strip()]

    all_records=[]
    for d in dirs:
        print(f"=== Traitement de {d} ===")
        recs=process_param_dir(d)
        all_records+=recs

    if all_records:
        df=pd.DataFrame(all_records)
        df.to_csv('all_metrics.csv',index=False)
        df.to_excel('all_metrics.xlsx',index=False)
        print("✅ all_metrics.* générés")

if __name__=='__main__':
    main()






















#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compute_evo_metrics_no_align.py

Pour chaque dossier passé en argument (ou listé dans dirs.txt), recherche récursivement tous les sous-dossiers "evo"
contenant un .tum, exécute evo_ape et evo_rpe avec --save_plot,
parse les logs pour extraire rmse, mean, median, std, min, max,
génère :
  - un dossier metriques_evo/ dans chaque param_dir
    - un dossier par configuration (<config>/)
      - <config>_ape_plot.png, <config>_rpe_plot.png
    - metrics_local.xlsx (une ligne par config) dans metriques_evo
  - un fichier global all_metrics.csv et all_metrics.xlsx à la racine
Ignore les erreurs et continue.
"""

import argparse
import os
import glob
import re
import subprocess
import pandas as pd

# === CONFIGURATION ===
REF_TUM = "/home/nochi/test_ws/Evaluation_SLAM_KITWARE/Outdoor/Par_defaut/evo/par_defaut.tum"
DELTA = "0.5"
DELTA_UNIT = "m"
DIRS_FILE_DEFAULT = "dirs_1.txt"

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


def run_and_parse(cmd: list[str], name: str, typ: str) -> dict[str, float]:
    """Lance la commande cmd et parse sa sortie pour extraire les statistiques."""
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"⚠️ evo_{typ} a échoué pour {name}")
        output = e.stdout or ""
    else:
        output = res.stdout

    stats: dict[str, float] = {}
    for line in output.splitlines():
        m = STAT_RE.match(line)
        if m:
            label = m.group('label').lower()
            stats[KEY_MAP[label]] = float(m.group('value'))
    return stats


def process_param_dir(param_dir: str) -> list[dict]:
    """Traite un dossier de paramètre : recherche les evo/, exécute APE/RPE, retourne la liste des records."""
    evo_dirs = [
        p for p in glob.glob(os.path.join(param_dir, "**", "evo"), recursive=True)
        if os.path.isdir(p)
    ]
    records: list[dict] = []
    met_dir = os.path.join(param_dir, "metriques_evo")
    os.makedirs(met_dir, exist_ok=True)

    for evo in sorted(evo_dirs):
        cfg = os.path.basename(os.path.dirname(evo))
        tum_files = glob.glob(os.path.join(evo, "*.tum"))
        if not tum_files:
            print(f"⚠️ Pas de .tum dans {evo}, on skip.")
            continue
        tum = tum_files[0]

        # APE
        ape_plot = os.path.join(evo, f"{cfg}_ape.png")
        ape_stats = run_and_parse(
            ["evo_ape", "tum", REF_TUM, tum, "--save_plot", ape_plot],
            cfg, "ape"
        )

        # RPE
        rpe_plot = os.path.join(evo, f"{cfg}_rpe.png")
        rpe_stats = run_and_parse(
            ["evo_rpe", "tum", REF_TUM, tum,
             "--delta", DELTA, "--delta_unit", DELTA_UNIT,
             "--save_plot", rpe_plot],
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
        description="Compute EVO metrics across multiple evo/ subfolders."
    )
    parser.add_argument(
        "-d", "--dirs-file",
        default=DIRS_FILE_DEFAULT,
        help="Chemin vers le fichier listant les dossiers à traiter (un par ligne)."
    )
    args = parser.parse_args()

    if not os.path.isfile(args.dirs_file):
        print(f"❌ Fichier introuvable : {args.dirs_file}")
        return

    with open(args.dirs_file) as f:
        dirs = [l.strip() for l in f if l.strip()]

    all_records: list[dict] = []
    for d in dirs:
        print(f"\n=== Traitement de {d} ===")
        recs = process_param_dir(d)
        all_records.extend(recs)

    if all_records:
        df = pd.DataFrame(all_records)
        df.to_csv("all_metrics.csv", index=False)
        df.to_excel("all_metrics.xlsx", index=False)
        print("✅ all_metrics.csv et all_metrics.xlsx générés.")


if __name__ == "__main__":
    main()

