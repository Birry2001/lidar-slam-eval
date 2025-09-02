#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------
# run_plots_confiance.sh
#
# Usage:
#   ./run_plots_confiance.sh <dirs-file>
#
# Pour chaque répertoire listé dans <dirs-file>, génère
# dans <dir>/graphe_metriques_confiance des graphes pour
# chaque métrique (computation_time, nb_matches, overlap,
# std_position_error) en utilisant plot_configs.py.
# ------------------------------------------------------------

if [ "$#" -ne 1 ]; then
  echo "❌ Usage: $0 <dirs-file>" >&2
  exit 1
fi

DIRS_FILE="$1"
PLOT_SCRIPT="plot_configs.py"
METRICS=(computation_time nb_matches overlap std_position_error)

if [ ! -f "$DIRS_FILE" ]; then
  echo "❌ Fichier introuvable : $DIRS_FILE" >&2
  exit 1
fi

echo "📖 Lecture du fichier de configurations : $DIRS_FILE"
echo

while IFS= read -r cfg_dir || [ -n "$cfg_dir" ]; do
  # ignorer lignes vides ou commentaires
  [[ -z "${cfg_dir// }" ]] && continue
  [[ "${cfg_dir:0:1}" == "#" ]] && continue

  echo "============================================================"
  echo "📁 Traitement de la configuration : $cfg_dir"

  if [ ! -d "$cfg_dir" ]; then
    echo "✖ Répertoire introuvable, skip." >&2
    echo
    continue
  fi

  OUT_DIR="$cfg_dir/graphe_metriques_confiance"
  echo "📂 Création / vérification du dossier de sortie : $OUT_DIR"
  mkdir -p "$OUT_DIR"

  for m in "${METRICS[@]}"; do
    echo
    echo "  🔍 Métrique : $m"
    CSV_ARGS=()

    # collecte des CSV et extraction du label (parent parent du fichier)
    for f in "$cfg_dir"/*/fichiers_csv/*"${m}"*.csv; do
      [ -f "$f" ] || continue
      # dirname "$f"          => .../10/fichiers_csv
      # dirname "$(dirname $f)" => .../10
      value_label="$(basename "$(dirname "$(dirname "$f")")")"
      echo "    ▶ Ajout de la série :"
      echo "       - fichier : $f"
      echo "       - label   : $value_label"
      CSV_ARGS+=( --csv "$f" "$value_label" )
    done

    if [ "${#CSV_ARGS[@]}" -lt 4 ]; then
      echo "    ❌ $m : moins de 2 séries, skip."
      continue
    fi

    OUT_PNG="$OUT_DIR/comparaison_${m}.png"
    echo "    ▶ Génération du graphe : $OUT_PNG"
    if python3 "$PLOT_SCRIPT" "${CSV_ARGS[@]}" --output "$OUT_PNG"; then
      echo "    ✅ Sauvegardé : $OUT_PNG"
    else
      echo "    ✖ Erreur lors de la génération du graphe pour $m" >&2
    fi
  done

  echo
done < "$DIRS_FILE"

echo "============================================================"
echo "✅ Tous les graphes traités."
