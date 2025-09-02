#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_to_csv.py

1) Extraction d'un bag ROS2 en CSV pour le topic /slam_confidence :
   ros2 run odom_csv_extractor extract_to_csv \
       --bag /chemin/vers/bagdir --topic /slam_confidence \
       [--outdir /chemin/vers/outdir]

2) Batch via un fichier dirs.txt :
   ros2 run odom_csv_extractor extract_to_csv \
       -d /chemin/vers/dirs.txt
   Où chaque ligne de dirs.txt est un dossier de paramètres,
   et pour chaque config à l'intérieur on extrait :
     ros_bag/all_bag → fichiers_csv/<metric>_<config>.csv
"""

import os
import csv
import sys
import argparse
from pathlib import Path

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message

# Messages ROS utilisés
from nav_msgs.msg import Odometry
from lidar_slam.msg import Confidence

# Configuration des topics & champs à extraire
TOPICS_CFG = {
    '/slam_confidence': {
        'msg_type': Confidence,
        'fields': {
            'overlap':            ('overlap',),
            'nb_matches':         ('nb_matches',),
            'std_position_error': ('std_position_error',),
            'computation_time':   ('computation_time',),
        }
    }
}


def get_attr_by_path(msg, path_tuple):
    """Descend récursivement dans l'objet msg selon le tuple d'attributs."""
    val = msg
    for attr in path_tuple:
        val = getattr(val, attr)
    return val


def extract_to_csv(bag_path: str, topic_name: str, output_dir: str):
    """
    Extrait tous les messages du bag ROS2 pour topic_name
    et génère un CSV par champ configuré, nommé <champ>_<config>.csv.
    """
    cfg = TOPICS_CFG.get(topic_name)
    if cfg is None:
        raise ValueError(f"Pas de configuration pour le topic {topic_name!r}")

    msg_type = cfg['msg_type']
    fields   = cfg['fields']

    # Prépare le reader ROS2
    storage_opts = StorageOptions(uri=bag_path, storage_id='sqlite3')
    conv_opts    = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)
    reader.set_filter(StorageFilter(topics=[topic_name]))

    # Crée le répertoire de sortie
    os.makedirs(output_dir, exist_ok=True)
    # Nom de la config = nom du dossier parent de output_dir
    cfg_name = Path(output_dir).parent.name

    # Préparation des CSV
    files, writers = {}, {}
    for key in fields:
        csv_path = os.path.join(output_dir, f"{key}_{cfg_name}.csv")
        f = open(csv_path, 'w', newline='')
        w = csv.writer(f)
        w.writerow(['timestamp', key])
        files[key]   = f
        writers[key] = w

    # Lecture & écriture
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, msg_type)
        ts  = msg.header.stamp
        timestamp = ts.sec + ts.nanosec * 1e-9
        for key, path_tuple in fields.items():
            try:
                value = get_attr_by_path(msg, path_tuple)
            except AttributeError:
                value = ''
            writers[key].writerow([timestamp, value])

    # Fermeture
    for f in files.values():
        f.close()


def main():
    parser = argparse.ArgumentParser(
        description="Extracteur CSV pour odom_csv_extractor"
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        '-d', '--dirs-file',
        help="Fichier listant les dossiers de paramètres à traiter")
    group.add_argument(
        '--bag',
        help="Chemin vers le dossier du bag ROS2 (sqlite3)")
    parser.add_argument(
        '--topic',
        default='/slam_confidence',
        choices=list(TOPICS_CFG.keys()),
        help="Topic à extraire en CSV (défaut: /slam_confidence)")
    parser.add_argument(
        '-o', '--outdir',
        help="Répertoire de sortie pour les CSV (mode unique --bag)")
    args = parser.parse_args()

    if args.dirs_file:
        # Mode batch : on traite tous les dossiers listés
        if not os.path.isfile(args.dirs_file):
            print(f"❌ Fichier introuvable : {args.dirs_file}", file=sys.stderr)
            sys.exit(1)
        with open(args.dirs_file) as f:
            base_dirs = [line.strip() for line in f if line.strip()]

        for base in base_dirs:
            print(f"\n=== Traitement de {base} ===")
            base_path = Path(base)
            if not base_path.is_dir():
                print(f"⚠️  {base} n'est pas un dossier, skip.")
                continue

            # Chaque sous-dossier est une config
            for cfg_dir in sorted(base_path.iterdir()):
                if not cfg_dir.is_dir():
                    continue
                bag_dir = cfg_dir / 'ros_bag' / 'all_bag'
                if not bag_dir.is_dir():
                    print(f"⚠️  Pas de bag dans {bag_dir}, skip.")
                    continue

                out_csv = cfg_dir / 'fichiers_csv'
                print(f"▶ Extraction pour la config '{cfg_dir.name}'")
                try:
                    extract_to_csv(str(bag_dir), args.topic, str(out_csv))
                except Exception as e:
                    print(f"❌ Échec pour {cfg_dir.name} : {e}", file=sys.stderr)

    else:
        # Mode unique : extraction sur un seul bag
        outdir = args.outdir or 'csv_output'
        os.makedirs(outdir, exist_ok=True)
        extract_to_csv(args.bag, args.topic, outdir)
        print(f"✅ CSV générés dans {outdir}")


if __name__ == '__main__':
    main()
