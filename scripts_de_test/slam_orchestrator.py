#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam_orchestrator.py - Orchestrateur SLAM KITWARE (sans manipulation de ROS_DOMAIN_ID)

Changements par rapport aux versions précédentes :
- Suppression totale de toute logique liée au ROS_DOMAIN_ID (pas de calcul, pas d’export).
- Pas de forcing de RMW_IMPLEMENTATION : on hérite 100% de l'environnement de l'utilisateur.
- Les commandes ros2/launch/param/daemon utilisent l'env courant (os.environ) tel quel.
- Le reste du pipeline est inchangé : build, override YAML, record bag, lancement SLAM,
  inspection des services/params en --no-daemon, lecture des params, arrêt propre, rapports.
"""

import json
import os
import re
import signal
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import typer
from jinja2 import Environment, FileSystemLoader, select_autoescape
import yaml
from subprocess import CalledProcessError

app = typer.Typer(add_completion=False, no_args_is_help=True)

# -------------------------------------------------------------------------
# Constantes
# -------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).parent.resolve()
BASE_DIR   = Path.home() / "test_ws" / "Evaluation_SLAM_KITWARE"
WS_DIR     = Path.home() / "test_ws"
SRC_DIR    = WS_DIR / "src"

# YAML actifs (utilisés par les launch files)
OUTDOOR_YAML = Path.home() / "test_ws/src/ros2_wrapping/lidar_slam/params/slam_config_outdoor.yaml"
INDOOR_YAML  = Path.home() / "test_ws/src/ros2_wrapping/lidar_slam/params/slam_config_indoor.yaml"

# YAML de référence (défauts fournis)
ORIG_OUT_YAML = Path.home() / "Téléchargements" / "slam_config_outdoor.yaml"
ORIG_IN_YAML  = Path.home() / "Téléchargements" / "slam_config_indoor.yaml"

LAUNCHES = {
    "hesai":    "slam_hesai.launch.py",
    "livox":    "slam_livox.launch.py",
    "ouster":   "slam_ouster.launch.py",
    "velodyne": "slam_velodyne.launch.py",
}

POST_SCRIPTS = [
    ("collect_confidence_metrics.py",    "-d {index}"),
    ("collect_summaries.py",             "-d {index}"),
    ("extract_trajectories.py",          "-d {index}"),
    ("extract_trajectory_plots.py",      "-d {index}"),
    ("summarize_by_metric.py",           "{exp_dir} {summary_output}"),
    ("compute_evo_metrics_no_align.py",  "-d {index} -r {ref}"),
    ("run_plots_confiance.sh",           "{exp}.txt"),
]

ENV = Environment(loader=FileSystemLoader(str(SCRIPT_DIR)),
                  autoescape=select_autoescape(['html']))

# Délais/temps
RECORDER_FLUSH_TIMEOUT_S = 90
LAUNCH_STOP_TIMEOUT_S    = 30
SPAWN_GRACE_S            = 3

# Patience (par défaut) — surchargeable par options CLI
WAIT_SERVICE_TIMEOUT_S   = 30.0
WAIT_PUB_TIMEOUT_S       = 15.0
PARAM_CHECK_MAX_TRIES    = 40
PARAM_CHECK_SLEEP_S      = 0.5

# -------------------------------------------------------------------------
# Utilitaires shell
# -------------------------------------------------------------------------
def run_cmd(cmd: str, dry_run: bool, **kwargs) -> int:
    typer.echo(f"   ▶ RUN: {cmd}")
    if dry_run:
        return 0
    res = subprocess.run(cmd, shell=True, executable="/bin/bash", **kwargs)
    return res.returncode

def validate_path(path: Path) -> bool:
    if not path.exists():
        typer.secho(f"❌ Chemin introuvable: {path}", fg=typer.colors.RED)
        return False
    return True

def clean_for_json(data: Any) -> Any:
    if isinstance(data, dict):
        return {k: clean_for_json(v) for k, v in data.items()}
    if isinstance(data, list):
        return [clean_for_json(v) for v in data]
    if data is None or isinstance(data, (str, int, float, bool)):
        return data
    return str(data)

def load_checkpoint(path: Path) -> Dict[str, Any]:
    if path.exists():
        try:
            return json.loads(path.read_text())
        except json.JSONDecodeError:
            return {}
    return {}

def save_checkpoint(path: Path, data: Dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2))

# -------------------------------------------------------------------------
# Accès/écriture paramètres YAML
# -------------------------------------------------------------------------
def _resolve_root_key(d: Dict[str, Any], k: str) -> str:
    if k in d:
        return k
    if k.startswith('/') and k[1:] in d:
        return k[1:]
    if ('/' + k) in d:
        return '/' + k
    return k

def get_in(d: Dict[str, Any], path: str) -> Optional[Any]:
    parts = path.split('.')
    sub: Any = d
    for i, k in enumerate(parts):
        if not isinstance(sub, dict):
            return None
        if i == 0:
            k = _resolve_root_key(sub, k)
        if k not in sub:
            return None
        sub = sub[k]
    return sub

def set_in(d: Dict[str, Any], path: str, value: Any) -> None:
    parts = path.split('.')
    sub = d
    for i, k in enumerate(parts[:-1]):
        if i == 0:
            k = _resolve_root_key(sub, k)
        sub = sub.setdefault(k, {})
    sub[parts[-1]] = value

def yaml_to_param_name(pth: str) -> str:
    for prefix in ("/lidar_slam.ros__parameters.", "lidar_slam.ros__parameters."):
        if pth.startswith(prefix):
            return pth[len(prefix):]
    return pth

# -------------------------------------------------------------------------
# Gestion process (arrêt propre)
# -------------------------------------------------------------------------
def graceful_stop(p: Optional[subprocess.Popen],
                  name: str,
                  first_sig=signal.SIGINT,
                  t1: int = RECORDER_FLUSH_TIMEOUT_S,
                  t2: int = 15,
                  t3: int = 5) -> None:
    if p is None or p.poll() is not None:
        return
    try:
        pgid = os.getpgid(p.pid)
    except ProcessLookupError:
        return
    try:
        os.killpg(pgid, first_sig)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=t1); return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=t2); return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        return
    p.wait(timeout=t3)

# -------------------------------------------------------------------------
# Dossiers & YAML override
# -------------------------------------------------------------------------
def make_dirs(base: Path, exp: str,
              tests_data: Dict[str, Dict[str, Any]]) -> Path:
    exp_dir = base / exp
    exp_dir.mkdir(parents=True, exist_ok=True)
    for t, td in tests_data.items():
        for sub in ("graphe_metriques_confiance", "metriques_evo", "summary",
                    "trajectoire", "metriques_confidence"):
            (exp_dir / t / sub).mkdir(parents=True, exist_ok=True)
        for c in td["configs"]:
            for sub in ("evo", "fichiers_csv", "maps", "ros_bag"):
                (exp_dir / t / c / sub).mkdir(parents=True, exist_ok=True)
    return exp_dir

def override_yaml_values(orig_yaml: Path, yaml_path: Path,
                         values: Dict[str, Any]) -> None:
    orig = yaml.safe_load(orig_yaml.read_text())
    data = yaml.safe_load(orig_yaml.read_text())
    for path, val in values.items():
        dv = get_in(orig, path)
        if isinstance(dv, bool):
            if isinstance(val, bool):
                nv = val
            elif isinstance(val, int):
                nv = bool(val)
            else:
                raise ValueError(f"Attendu bool/int pour {path}, reçu {type(val).__name__}")
        elif isinstance(dv, int) and not isinstance(dv, bool):
            nv = int(val)
        elif isinstance(dv, float):
            nv = float(val)
        else:
            nv = val
        set_in(data, path, nv)
    yaml_path.write_text(yaml.safe_dump(data))

# -------------------------------------------------------------------------
# Lecture & Validation du plan YAML
# -------------------------------------------------------------------------
def _read_yaml(path: Path) -> Any:
    if path.suffix.lower() not in (".yaml", ".yml"):
        raise RuntimeError("Le fichier doit être au format .yaml/.yml")
    return yaml.safe_load(path.read_text(encoding="utf-8"))

def _compose_from_style_A(doc: Dict[str, Any]) -> Tuple[Dict[str, Any], Dict[str, Dict[str, Any]]]:
    orch = {
        "experiment": doc["experiment"],
        "lidar": doc["lidar"],
        "mode": doc.get("mode", "outdoor"),
        "bag_path": doc.get("bag") or doc.get("bag_path"),
        "ref_tum": doc["ref_tum"],
    }
    tests_data: Dict[str, Dict[str, Any]] = {}
    tests_map = doc["tests"]
    for test_name, test_obj in tests_map.items():
        cfg_map = test_obj["configs"]
        cfgs = list(cfg_map.keys())
        params = sorted({p for v in cfg_map.values() for p in v.keys()})
        vals = {cfg: dict(cfg_map[cfg]) for cfg in cfgs}
        tests_data[test_name] = {"params": params, "configs": cfgs, "vals": vals}
    return orch, tests_data

def _compose_from_style_B(doc: Dict[str, Any]) -> Tuple[Dict[str, Any], Dict[str, Dict[str, Any]]]:
    orch = {
        "experiment": doc["orchestrator"]["experiment"],
        "lidar": doc["orchestrator"]["lidar"],
        "mode": doc["orchestrator"].get("mode", "outdoor"),
        "bag_path": doc["orchestrator"].get("bag") or doc["orchestrator"]["bag_path"],
        "ref_tum": doc["orchestrator"]["ref_tum"],
    }
    tests_data: Dict[str, Dict[str, Any]] = {}
    tests_list = doc["tests"]
    for t in tests_list:
        name = t["name"]
        params = list(t.get("params", []))
        cfgs = [c["name"] for c in t["configs"]]
        vals = {c["name"]: dict(c.get("values", {})) for c in t["configs"]}
        if not params:
            params = sorted({p for v in vals.values() for p in v.keys()})
        tests_data[name] = {"params": params, "configs": cfgs, "vals": vals}
    return orch, tests_data

def load_plan_yaml(plan_path: Path) -> Tuple[Dict[str, Any], Dict[str, Dict[str, Any]]]:
    doc = _read_yaml(plan_path)
    if not isinstance(doc, dict):
        raise RuntimeError("Plan YAML invalide: la racine doit être un mapping.")
    if "experiment" in doc and "tests" in doc:
        return _compose_from_style_A(doc)
    if "orchestrator" in doc and "tests" in doc:
        return _compose_from_style_B(doc)
    raise RuntimeError("Plan YAML invalide: utilisez le style 'compact' (experiment+tests) ou 'orchestrator+tests'.")

# ---------- Validation ----------
def _paramname_ok(name: str) -> bool:
    return name.startswith("/lidar_slam.ros__parameters.") or name.startswith("lidar_slam.ros__parameters.")

def _configname_ok(name: str) -> bool:
    return bool(re.fullmatch(r"[a-z0-9][a-z0-9\.\-]*", name))

def _has_default_config(cfgs: List[str]) -> bool:
    return any(c.startswith("par-defaut") for c in cfgs)

def _coerce_type_like(default_val: Any, v: Any) -> bool:
    if isinstance(default_val, bool):
        return isinstance(v, (bool, int))
    if isinstance(default_val, int) and not isinstance(default_val, bool):
        return isinstance(v, int)
    if isinstance(default_val, float):
        return isinstance(v, (int, float))
    return True

def validate_plan(orch: Dict[str, Any],
                  tests_data: Dict[str, Dict[str, Any]],
                  mode_yaml_ref: Path) -> List[str]:
    errors: List[str] = []

    for req in ("experiment", "lidar", "mode", "bag_path", "ref_tum"):
        if req not in orch or orch[req] in (None, "", []):
            errors.append(f"[orchestrator] Champ requis manquant: {req}")

    if orch.get("lidar") not in LAUNCHES:
        errors.append(f"[orchestrator] Lidar inconnu: {orch.get('lidar')} (attendu: {', '.join(LAUNCHES.keys())})")

    if orch.get("mode") not in ("indoor", "outdoor"):
        errors.append("[orchestrator] Mode doit être 'indoor' ou 'outdoor'.")

    bag_path = Path(orch.get("bag_path", ""))
    if not bag_path.exists():
        errors.append(f"[orchestrator] bag_path introuvable: {bag_path}")

    ref_tum = Path(orch.get("ref_tum", ""))
    if not ref_tum.exists():
        errors.append(f"[orchestrator] ref_tum introuvable: {ref_tum}")

    if not tests_data:
        errors.append("[tests] Aucun test défini.")
        return errors

    yaml_ref_path = mode_yaml_ref
    yaml_ref = None
    if yaml_ref_path.exists():
        try:
            yaml_ref = yaml.safe_load(yaml_ref_path.read_text())
        except Exception as e:
            errors.append(f"[validation] Impossible de charger YAML de référence '{yaml_ref_path}': {e}")

    for tname, td in tests_data.items():
        cfgs = td.get("configs", [])
        vals = td.get("vals", {})
        params = td.get("params", [])

        if not _has_default_config(cfgs):
            errors.append(f"[{tname}] Aucune config par défaut ('par-defaut-...')")

        for c in cfgs:
            if not _configname_ok(c):
                errors.append(f"[{tname}] Nom de config invalide '{c}'")

        for p in params:
            if not _paramname_ok(p):
                errors.append(f"[{tname}] Nom de paramètre non conforme: {p}")

        for c in cfgs:
            if c not in vals:
                errors.append(f"[{tname}] Manque le bloc 'values' pour la config '{c}'")
                continue
            for pth, v in vals[c].items():
                if not _paramname_ok(pth):
                    errors.append(f"[{tname}/{c}] Paramètre non conforme: {pth}")
                    continue
                if yaml_ref is not None:
                    dv = get_in(yaml_ref, pth)
                    if dv is None:
                        errors.append(f"[{tname}/{c}] Paramètre inconnu dans YAML de référence: {pth}")
                    else:
                        if not _coerce_type_like(dv, v):
                            errors.append(f"[{tname}/{c}] Type incompatible pour {pth} "
                                          f"(défaut={type(dv).__name__}, valeur={type(v).__name__})")
        if params:
            for c in cfgs:
                extra = sorted(set(vals.get(c, {}).keys()) - set(params))
                if extra:
                    errors.append(f"[{tname}/{c}] Valeurs définies pour des paramètres non listés dans 'params': {extra}")
    return errors

# -------------------------------------------------------------------------
# Exécution d'une config
# -------------------------------------------------------------------------
def process_config(
    exp_dir: Path,
    test: str,
    cfg: str,
    values: Dict[str, Any],
    bag_path: Path,
    dry_run: bool,
    report: Dict[str, Any],
    launch_file: str,
    yaml_path: Path,
    orig_yaml: Path,
    *,
    wait_pub: bool,
    param_max_tries: int,
    param_sleep_s: float
):
    key = f"{test}/{cfg}"
    report[key] = {"steps": {}, "status": "ok", "artefacts": {}}

    # On hérite 100% de l'environnement courant (pas de ROS_DOMAIN_ID/RMW forcés)
    run_env = os.environ.copy()

    def record_step(name: str):
        def deco(fn):
            def wrap(*a, **k):
                t0 = time.time()
                typer.secho(f"🛠 {name} ({key})", fg=typer.colors.GREEN)
                try:
                    res = fn(*a, **k)
                    report[key]["steps"][name] = time.time() - t0
                    return res
                except Exception as e:
                    report[key]["status"] = f"error:{e}"
                    report[key]["steps"][name] = time.time() - t0
                    raise
            return wrap
        return deco

    def _restore_yaml():
        try:
            shutil.copy2(orig_yaml, yaml_path)
            typer.echo(f"▶ YAML restored from {orig_yaml}")
        except Exception as e:
            typer.secho(f"⚠️  Restauration YAML a échoué: {e}", fg=typer.colors.YELLOW)

    @record_step("override_yaml")
    def step_override_yaml():
        override_yaml_values(orig_yaml, yaml_path, values)

    @record_step("record_bag")
    def step_record_bag():
        bag_dir = exp_dir / test / cfg / "ros_bag" / "all_bag"
        bag_dir.parent.mkdir(parents=True, exist_ok=True)
        cmd = (
            f'bash -lc "cd ~/test_ws && '
            f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
            f'exec ros2 bag record -o {bag_dir} --storage sqlite3 '
            f'/slam_odom /slam_confidence"'
        )
        typer.echo(f"▶ {cmd}")
        if dry_run:
            return
        p = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                             start_new_session=True, env=run_env)
        report[key]["artefacts"]["bag_proc"] = p
        time.sleep(SPAWN_GRACE_S)

    # --- Pré-nettoyage : tue anciens processus + reset daemon ROS 2 ---
    def preflight_clean():
        cmds = [
            "pkill -f rviz2 || true",
            "pkill -f 'ros2 launch lidar_slam' || true",
            "pkill -f static_transform_publisher || true",
            "ros2 daemon stop || true",
            "ros2 daemon start",
        ]
        for c in cmds:
            subprocess.run(f'bash -lc "{c}"', shell=True, executable="/bin/bash", env=run_env)
        time.sleep(0.5)

    @record_step("launch_slam")
    def step_launch_slam():
        preflight_clean()
        outdoor_arg = 'true' if yaml_path == OUTDOOR_YAML else 'false'
        cmd = (
            f'bash -lc "cd ~/test_ws && '
            f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
            f'exec ros2 launch lidar_slam {launch_file} outdoor:={outdoor_arg}"'
        )
        typer.echo(f"▶ {cmd}")
        if dry_run:
            return
        p = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                             start_new_session=True, env=run_env)
        report[key]["artefacts"]["slam_proc"] = p
        time.sleep(SPAWN_GRACE_S)

    @record_step("check_params_runtime")
    def step_check_params():
        if dry_run:
            report[key]["artefacts"]["effective_params"] = {}
            return

        # marge courte pour découverte
        time.sleep(2.0)

        # 1) Trouver le nœud via la liste des services (sans daemon)
        lidar_node_name: Optional[str] = None
        want_suffix = "/get_parameters"
        name_hints  = ("lidar_slam", "lidar_slam_node")  # sous-chaînes acceptées

        t0 = time.time()
        while time.time() - t0 < WAIT_SERVICE_TIMEOUT_S and not lidar_node_name:
            proc = subprocess.run(
                'bash -lc "ros2 --no-daemon service list"',
                shell=True, executable="/bin/bash",
                capture_output=True, text=True, env=run_env
            )
            services = [s.strip() for s in proc.stdout.splitlines() if s.strip()]
            for srv in services:
                if not srv.endswith(want_suffix):
                    continue
                node = srv[: -len(want_suffix)]
                if any(h in node for h in name_hints):
                    lidar_node_name = node
                    break
            if not lidar_node_name:
                time.sleep(0.5)

        if not lidar_node_name:
            nodes_dbg = subprocess.run(
                'bash -lc "ros2 --no-daemon node list || true"',
                shell=True, executable="/bin/bash",
                capture_output=True, text=True, env=run_env
            ).stdout.strip() or "<none>"
            svcs_dbg  = subprocess.run(
                'bash -lc "ros2 --no-daemon service list || true"',
                shell=True, executable="/bin/bash",
                capture_output=True, text=True, env=run_env
            ).stdout.strip() or "<none>"

            typer.secho("⚠️  Impossible d’identifier le nœud SLAM via /get_parameters.", fg=typer.colors.YELLOW)
            typer.echo(f"  - nodes: {nodes_dbg}")
            typer.echo(f"  - services:\n{svcs_dbg}")
            report[key]["artefacts"]["effective_params"] = {
                yaml_to_param_name(p): "<node not found>" for p in values.keys()
            }
            report[key]["artefacts"]["services_snapshot"] = svcs_dbg
            return

        typer.echo(f"▶ Nœud SLAM retenu: {lidar_node_name}")

        # 2) Optionnel : attendre un publisher /slam_odom (sans daemon)
        if wait_pub:
            t0 = time.time()
            while time.time() - t0 < WAIT_PUB_TIMEOUT_S:
                proc = subprocess.run(
                    'bash -lc "ros2 --no-daemon topic info /slam_odom -v"',
                    shell=True, executable="/bin/bash",
                    capture_output=True, text=True, env=run_env
                )
                if "Publisher count:" in proc.stdout:
                    try:
                        line = next(l for l in proc.stdout.splitlines() if "Publisher count:" in l)
                        if int(line.split(":")[1].strip()) >= 1:
                            break
                    except Exception:
                        pass
                time.sleep(0.5)

        # 3) Lecture des paramètres (sans daemon) avec backoff
        effective: Dict[str, str] = {}
        tries = max(1, int(param_max_tries))
        base_sleep = max(0.05, float(param_sleep_s))

        for pth in values.keys():
            pn = yaml_to_param_name(pth)
            out_text = ""
            rc = 1
            sleep_s = base_sleep
            cmd = (
                f'bash -lc "cd ~/test_ws && '
                f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
                f'ros2 --no-daemon param get {lidar_node_name} {pn}"'
            )
            for _ in range(tries):
                proc = subprocess.run(cmd, shell=True, executable="/bin/bash",
                                      capture_output=True, text=True, env=run_env)
                rc = proc.returncode
                if rc == 0 and proc.stdout.strip():
                    out_text = proc.stdout.strip()
                    break
                time.sleep(sleep_s)
                sleep_s = min(sleep_s * 1.25, 2.0)
            if rc != 0:
                out_text = f"<get failed rc={rc}>"
            elif not out_text:
                out_text = "<empty param value>"
            effective[pn] = out_text

        report[key]["artefacts"]["effective_params"] = effective

    @record_step("play_lidar")
    def step_play_lidar():
        cmd = (
            f'bash -lc "cd ~/test_ws && '
            f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
            f'exec ros2 bag play {bag_path}"'
        )
        if dry_run:
            return 0
        res = subprocess.run(cmd, shell=True, executable="/bin/bash", env=run_env)
        return res.returncode

    @record_step("stop_restore")
    def step_stop_restore():
        bag_p  = report[key]["artefacts"].get("bag_proc")
        slam_p = report[key]["artefacts"].get("slam_proc")
        graceful_stop(bag_p,  "ros2 bag record",
                      first_sig=signal.SIGINT, t1=RECORDER_FLUSH_TIMEOUT_S)
        graceful_stop(slam_p, "ros2 launch",
                      first_sig=signal.SIGINT, t1=LAUNCH_STOP_TIMEOUT_S)
        run_cmd("pkill -f rviz2", dry_run)
        run_cmd('bash -lc "ros2 daemon stop || true"', dry_run)
        run_cmd('bash -lc "ros2 daemon start"', dry_run)
        _restore_yaml()

    # Pipeline (restaure YAML même si une étape plante)
    try:
        step_override_yaml()
        step_record_bag()
        step_launch_slam()
        step_check_params()
        step_play_lidar()
    finally:
        step_stop_restore()

# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------
@app.command()
def main(
    plan: Optional[Path] = typer.Option(None, "--plan", "-p", help="Chemin du plan (.yaml/.yml)"),
    dry_run: bool = typer.Option(False, "--dry-run", help="N'exécute pas les commandes ROS."),
    skip_build: bool = typer.Option(False, "--skip-build", help="Ne pas lancer colcon build."),
    skip_post: bool = typer.Option(False, "--skip-post", help="Ne pas lancer les scripts de post-traitement."),
    only_tests: Optional[str] = typer.Option(None, "--only-tests", help="Liste CSV de tests à exécuter (noms)."),
    resume: bool = typer.Option(True, "--resume/--no-resume", help="Respecter le checkpoint et ignorer les configs déjà 'done'."),
    # Patience/réglages
    wait_pub: bool = typer.Option(False, "--wait-pub", help="Attendre un publisher sur /slam_odom avant la lecture des paramètres."),
    param_max_tries: int = typer.Option(PARAM_CHECK_MAX_TRIES, "--param-max-tries", help="Nb max d'essais pour ros2 param get."),
    param_sleep_s: float = typer.Option(PARAM_CHECK_SLEEP_S, "--param-sleep", help="Sleep initial entre essais (backoff inclus)."),
):
    typer.secho("🛠️  Orchestrateur SLAM KITWARE", fg=typer.colors.BLUE)

    # 1) Plan YAML
    plan_path = plan or Path(typer.prompt("Chemin du plan (.yaml/.yml)"))
    if not validate_path(plan_path):
        raise typer.Exit(1)

    # 2) Chargement + normalisation
    try:
        orch, tests_data = load_plan_yaml(plan_path)
    except Exception as e:
        typer.secho(f"❌ Erreur de lecture du plan: {e}", fg=typer.colors.RED)
        raise typer.Exit(1)

    # 3) Validation (structure + types vs YAML de référence ROS 2)
    mode = orch.get("mode", "outdoor")
    yaml_ref = ORIG_IN_YAML if mode == "indoor" else ORIG_OUT_YAML
    if not yaml_ref.exists():
        yaml_ref = INDOOR_YAML if mode == "indoor" else OUTDOOR_YAML

    errors = validate_plan(orch, tests_data, yaml_ref)
    if errors:
        typer.secho("\n❌ Le plan est invalide. Détails :", fg=typer.colors.RED)
        for err in errors:
            typer.echo(f"  - {err}")
        typer.secho("\nAbandon (corrigez le plan YAML puis relancez).", fg=typer.colors.RED)
        raise typer.Exit(1)

    # 4) Métadonnées validées
    exp        = orch["experiment"]
    lidar      = orch["lidar"]
    launch_file= LAUNCHES[lidar]
    mode       = orch["mode"]
    bag_path   = Path(orch["bag_path"])
    ref_tum    = Path(orch["ref_tum"])
    yaml_path  = INDOOR_YAML if mode == "indoor" else OUTDOOR_YAML
    orig_yaml  = ORIG_IN_YAML if mode == "indoor" else ORIG_OUT_YAML

    typer.secho("✅ Plan validé, lancement de la campagne…", fg=typer.colors.GREEN)

    # 5) Dossiers & index
    exp_dir = make_dirs(BASE_DIR, exp, tests_data)
    index = WS_DIR / f"{exp}.txt"
    test_names = list(tests_data.keys())
    if only_tests:
        keep = {t.strip() for t in only_tests.split(",") if t.strip()}
        test_names = [t for t in test_names if t in keep]
    index.write_text("\n".join(str(exp_dir / t) for t in test_names))

    # JSON par config
    for t in test_names:
        td = tests_data[t]
        for c in td["configs"]:
            d = exp_dir / t / c
            d.mkdir(parents=True, exist_ok=True)
            (d / f"{c}.json").write_text(json.dumps(td["vals"][c], indent=2))

    # 6) Rapport & checkpoint
    report: Dict[str, Any] = {}
    checkpoint = load_checkpoint(exp_dir / "checkpoints.json")

    # 7) Build unique — symlink-install (inclut odom_csv_extractor si présent)
    if not skip_build:
        odom_pkg_dir = Path.home() / "test_ws" / "odom_csv_extractor"
        if odom_pkg_dir.exists() and (odom_pkg_dir / "package.xml").exists():
            build_cmd = (
                'bash -lc "cd ~/test_ws && '
                'if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi; '
                f'colcon build --symlink-install --base-paths src/ros2_wrapping {odom_pkg_dir}"'
            )
        else:
            build_cmd = (
                'bash -lc "cd ~/test_ws && '
                'if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi; '
                'colcon build --symlink-install --base-paths src/ros2_wrapping && '
                'colcon build --symlink-install --packages-select odom_csv_extractor || true"'
            )

        typer.echo(f"▶ Build once (with symlinks): {build_cmd}")
        if not dry_run:
            rc = run_cmd(build_cmd, dry_run=False)
            if rc != 0:
                typer.secho(f"❌ Échec build colcon (code {rc})", fg=typer.colors.RED)
                raise typer.Exit(1)

        # Vérif exécutable odom_csv_extractor
        if not dry_run:
            check_cmd = (
                'bash -lc "cd ~/test_ws && '
                'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
                'ros2 run odom_csv_extractor extract_to_csv -h >/dev/null 2>&1"'
            )
            rc = run_cmd(check_cmd, dry_run=False)
            if rc != 0:
                typer.secho(
                    "❌ 'odom_csv_extractor' introuvable après build. Vérifie ~/test_ws/odom_csv_extractor.",
                    fg=typer.colors.RED
                )
                raise typer.Exit(1)

    # 9) Boucle d’exécution
    for t in test_names:
        td = tests_data[t]
        for c in td["configs"]:
            key = f"{t}/{c}"
            if resume and checkpoint.get(key) == "done":
                typer.secho(f"↩️ Ignored: {key}", fg=typer.colors.CYAN)
                continue
            values = td["vals"][c]
            try:
                process_config(exp_dir, t, c, values, bag_path, dry_run,
                               report, launch_file, yaml_path, orig_yaml,
                               wait_pub=wait_pub,
                               param_max_tries=param_max_tries,
                               param_sleep_s=param_sleep_s)
                checkpoint[key] = "done"
                save_checkpoint(exp_dir / "checkpoints.json", checkpoint)
            except Exception as e:
                typer.secho(f"❌ Erreur pendant {key}: {e}", fg=typer.colors.RED)

    # 10) Extraction CSV (odom_csv_extractor)
    run_cmd(
        f'bash -lc "cd ~/test_ws && '
        f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
        f'ros2 run odom_csv_extractor extract_to_csv -d {index}"',
        dry_run
    )

    # 11) Post-traitements
    if not skip_post:
        summary_dir = exp_dir / "summary"
        summary_dir.mkdir(exist_ok=True)
        summary_json = summary_dir / f"summary_{exp}.json"

        for script, arg in POST_SCRIPTS:
            path = SCRIPT_DIR / script
            cmd_args = arg.format(exp_dir=exp_dir,
                                  summary_output=summary_json,
                                  exp=exp,
                                  index=index,
                                  ref=Path(orch["ref_tum"]))
            if script.endswith('.sh'):
                cmd = f"bash {path} {cmd_args}"
            else:
                cmd = f"chmod +x {path} && python3 {path} {cmd_args}"
            typer.echo(f"▶ {cmd}")
            if not dry_run:
                try:
                    subprocess.run(cmd, shell=True, executable="/bin/bash", check=True)
                except CalledProcessError as e:
                    typer.secho(f"❌ Erreur {script} code {e.returncode}", fg=typer.colors.RED)

    # 12) Déplacement index
    dirs = exp_dir / "dirs"
    dirs.mkdir(exist_ok=True)
    try:
        shutil.move(str(index), str(dirs / index.name))
    except Exception:
        pass

    # 13) Rapports
    out_json = exp_dir / "report.json"
    out_html = exp_dir / "report.html"
    out_json.write_text(json.dumps(clean_for_json(report), indent=2))

    tpl = ENV.from_string(
        """<!doctype html><html><body>
        <h1>Rapport {{exp}}</h1><p>{{now}}</p>
        <table border="1" cellpadding="6">
          <tr><th>Config</th><th>Status</th><th>Durées (s)</th></tr>
          {% for k,v in report.items() %}
            <tr>
              <td>{{k}}</td>
              <td>{{v.status}}</td>
              <td>
                {% for s,dt in v.steps.items() %}
                  <div>{{ s }}: {{ "%.2f"|format(dt) }}</div>
                {% endfor %}
              </td>
            </tr>
          {% endfor %}
        </table>
        </body></html>"""
    )
    out_html.write_text(tpl.render(exp=exp,
                                   report=report,
                                   now=time.strftime("%Y-%m-%d %H:%M:%S")))

    typer.secho("✅ Terminé", fg=typer.colors.GREEN)
    typer.echo(f"JSON: {out_json}")
    typer.echo(f"HTML: {out_html}")

# Entrée du programme
if __name__ == "__main__":
    app()
