#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import argparse
import pathlib
import sys


def summarize_per_metric(input_dir: pathlib.Path, metrics: list):
    """
    Parcourt input_dir pour CSV nommés <metric>_<config>.csv,
    calcule min, max et mean pour chaque et retourne dict metric->DataFrame.
    """
    recs = {m: [] for m in metrics}

    for csv_path in sorted(input_dir.glob("*.csv")):
        name = csv_path.stem
        if "_" not in name:
            continue
        metric, config = name.rsplit("_", 1)
        if metric not in metrics:
            continue

        df = pd.read_csv(csv_path)
        if metric not in df.columns:
            continue

        col = df[metric]
        recs[metric].append({
            "config":         config,
            f"{metric}_min":  col.min(),
            f"{metric}_max":  col.max(),
            f"{metric}_mean": col.mean(),
        })

    dfs = {}
    for metric, rows in recs.items():
        if rows:
            dfs[metric] = (
                pd.DataFrame(rows)
                  .set_index("config")
                  .sort_index()
            )
    return dfs


def main():
    parser = argparse.ArgumentParser(
        description="Génère un Excel avec une feuille par métrique (min/max/mean par config)."
    )
    parser.add_argument("input_dir",  type=pathlib.Path,
                        help="Répertoire des CSV nommés <metric>_<config>.csv")
    parser.add_argument("output_path", type=pathlib.Path,
                        help="Fichier .xlsx de sortie ou dossier pour summary_by_metric.xlsx")
    args = parser.parse_args()

    input_dir = args.input_dir
    if not input_dir.is_dir():
        print(f"❌ Répertoire introuvable : {input_dir}", file=sys.stderr)
        sys.exit(1)

    op = args.output_path
    if op.exists() and op.is_dir():
        out_file = op / "summary_by_metric.xlsx"
    else:
        out_file = op.with_suffix(".xlsx")
        out_file.parent.mkdir(parents=True, exist_ok=True)

    metrics = ["computation_time", "overlap", "nb_matches", "std_position_error"]
    dfs = summarize_per_metric(input_dir, metrics)

    if not dfs:
        print("❌ Aucune métrique valide trouvée.", file=sys.stderr)
        sys.exit(1)

    with pd.ExcelWriter(out_file, engine="xlsxwriter") as writer:
        for metric, dfm in dfs.items():
            sheet = metric[:31]
            dfm.to_excel(writer, sheet_name=sheet)
            wks = writer.sheets[sheet]

            # Colonne "config"
            max_idx = max((len(idx) for idx in dfm.index), default=0)
            wks.set_column(0, 0, max(10, max_idx + 2))

            # Colonnes de stats (+10 de marge)
            for col_idx, col in enumerate(dfm.columns, start=1):
                # formater chaque valeur pour mesurer sa largeur
                lengths = dfm[col].apply(
                    lambda x: len(f"{x:.6f}") if isinstance(x, (int, float)) else len(str(x))
                )
                max_val_len = lengths.max() if not lengths.empty else 0
                header_len = len(col)
                width = max(header_len, max_val_len) + 10
                wks.set_column(col_idx, col_idx, width)

    print(f"✅ Résultats sauvegardés dans '{out_file}'")


if __name__ == "__main__":
    main()
