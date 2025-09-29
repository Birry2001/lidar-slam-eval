#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import re
import shlex
import signal
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import typer
from jinja2 import Environment, FileSystemLoader, select_autoescape
import yaml
from subprocess import CalledProcessError

app = typer.Typer(add_completion=False, no_args_is_help=True)

# -------------------------------------------------------------------------
# Constantes (valeurs de repli - tout est surchargé plus bas par CLI/env)
# -------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).parent.resolve()
DEFAULT_WS_DIR   = Path.home() / "test_ws"
DEFAULT_BASE_DIR = DEFAULT_WS_DIR / "Evaluation_SLAM_KITWARE"
DEFAULT_SRC_DIR  = DEFAULT_WS_DIR / "src"

# YAML actifs (valeurs de repli)
DEFAULT_OUTDOOR_YAML = DEFAULT_WS_DIR / "src/ros2_wrapping/lidar_slam/params/slam_config_outdoor.yaml"
DEFAULT_INDOOR_YAML  = DEFAULT_WS_DIR / "src/ros2_wrapping/lidar_slam/params/slam_config_indoor.yaml"

# Lidar -> launch par défaut (surchargé par plan)
DEFAULT_LAUNCHES = {
    "hesai":    "slam_hesai.launch.py",
    "livox":    "slam_livox.launch.py",
    "ouster":   "slam_ouster.launch.py",
    "velodyne": "slam_velodyne.launch.py",
}

# Post scripts (nom, args-template)
POST_SCRIPTS = [
    ("collect_confidence_metrics.py",    "-d {index}"),
    ("collect_summaries.py",             "-d {index}"),
    ("extract_trajectories.py",          "-d {index}"),
    ("extract_trajectory_plots.py",      "-d {index}"),
    ("summarize_by_metric.py",           "{exp_dir} {summary_output}"),
    ("compute_evo_metrics_auto_align.py","-d {index} -r {ref}"),
    ("run_plots_confiance.sh",           "{exp}.txt"),
]

ENV = Environment(loader=FileSystemLoader(str(SCRIPT_DIR)),
                  autoescape=select_autoescape(['html']))

# Délais/temps
RECORDER_FLUSH_TIMEOUT_S = 290
LAUNCH_STOP_TIMEOUT_S    = 30
SPAWN_GRACE_S            = 3

# Patience (par défaut) — surchargeable par options CLI
WAIT_SERVICE_TIMEOUT_S   = 0
WAIT_PUB_TIMEOUT_S       = 15.0
PARAM_CHECK_MAX_TRIES    = 1
PARAM_CHECK_SLEEP_S      = 1.0

# -------------------------------------------------------------------------
# Helpers portabilité / chemins / env
# -------------------------------------------------------------------------
def detect_downloads_dir() -> Path:
    xdg = os.environ.get("XDG_DOWNLOAD_DIR")
    if xdg:
        p = Path(xdg).expanduser()
        if p.exists():
            return p
    for name in ("Downloads", "Téléchargements"):
        p = Path.home() / name
        if p.exists():
            return p
    return Path.home()

def env_path(var: str, default: Path) -> Path:
    v = os.environ.get(var)
    return Path(v).expanduser().resolve() if v else default.resolve()

def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)

def read_text_safe(p: Path, encoding="utf-8") -> str:
    return p.read_text(encoding=encoding)

def run(cmd: str, *, capture: bool=False, env: Optional[Dict[str,str]]=None, check: bool=False) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, shell=True, executable="/bin/bash",
                          capture_output=capture, text=True, env=env, check=check)

def run_capture_text(cmd: str, env=None) -> Tuple[int,str,str]:
    proc = run(cmd, capture=True, env=env)
    return proc.returncode, proc.stdout or "", proc.stderr or ""

def quote(p: Path) -> str:
    return shlex.quote(str(p))

def get_versions(env: Optional[Dict[str,str]]) -> Dict[str,str]:
    meta = {
        "python_version": sys.version.replace("\n"," "),
        "script_path": str(SCRIPT_DIR),
    }
    rc,o,_ = run_capture_text("ros2 --version || true", env)
    if o.strip():
        meta["ros2_version"] = o.strip()
    rc,o,_ = run_capture_text("colcon --version || true", env)
    if o.strip():
        meta["colcon_version"] = o.strip()
    return meta

def find_package_dir(ws_dir: Path, pkg_name: str) -> Optional[Path]:
    search_root = ws_dir
    if not search_root.exists():
        return None
    for pkg_xml in search_root.rglob("package.xml"):
        try:
            txt = pkg_xml.read_text(encoding="utf-8", errors="ignore")
            if re.search(rf"<\s*name\s*>\s*{re.escape(pkg_name)}\s*<\s*/\s*name\s*>", txt):
                return pkg_xml.parent
        except Exception:
            pass
    return None

# -------------------------------------------------------------------------
# Utilitaires shell génériques
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
        for sub in ("graphe_metriques_confiance", "metriques_evo", "summary", "metriques_confidence", "logs"):
            (exp_dir / t / sub).mkdir(parents=True, exist_ok=True)
        for c in td["configs"]:
            for sub in ("evo", "fichiers_csv", "maps", "ros_bag", "logs"):
                (exp_dir / t / c / sub).mkdir(parents=True, exist_ok=True)
    return exp_dir

def override_yaml_values(orig_yaml: Path, yaml_path: Path,
                         values: Dict[str, Any]) -> None:
    orig = yaml.safe_load(read_text_safe(orig_yaml))
    data = yaml.safe_load(read_text_safe(orig_yaml))
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
# Lecture & Validation du plan YAML (+ extensions)
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
        "record_topics": doc.get("record_topics"),
        "play_args": doc.get("play_args"),
        "launch_file": doc.get("launch_file"),
        "launches": doc.get("launches", {}),
        "node_name": doc.get("node_name"),  # NOUVEAU
    }
    tests_data: Dict[str, Dict[str, Any]] = {}
    tests_map = doc["tests"]
    for test_name, test_obj in tests_map.items():
        cfg_map = test_obj["configs"]
        cfgs = list(cfg_map.keys())
        params = sorted({p for v in cfg_map.values() for p in v.keys() if isinstance(v, dict)})
        vals = {cfg: dict(cfg_map[cfg]) for cfg in cfgs}
        tags = test_obj.get("tags", []) or []
        test_play_args = test_obj.get("play_args")
        play_args_per_cfg: Dict[str, Optional[str]] = {}
        for cfg in cfgs:
            pav = None
            if isinstance(cfg_map[cfg], dict) and "__play_args__" in cfg_map[cfg]:
                pav = cfg_map[cfg].pop("__play_args__")
            play_args_per_cfg[cfg] = pav
        tests_data[test_name] = {
            "params": params, "configs": cfgs, "vals": vals,
            "tags": tags, "test_play_args": test_play_args,
            "play_args_per_cfg": play_args_per_cfg
        }
    return orch, tests_data

def _compose_from_style_B(doc: Dict[str, Any]) -> Tuple[Dict[str, Any], Dict[str, Dict[str, Any]]]:
    orch_doc = doc["orchestrator"]
    orch = {
        "experiment": orch_doc["experiment"],
        "lidar": orch_doc["lidar"],
        "mode": orch_doc.get("mode", "outdoor"),
        "bag_path": orch_doc.get("bag") or orch_doc.get("bag_path"),
        "ref_tum": orch_doc["ref_tum"],
        "record_topics": orch_doc.get("record_topics"),
        "play_args": orch_doc.get("play_args"),
        "launch_file": orch_doc.get("launch_file"),
        "launches": orch_doc.get("launches", {}),
        "node_name": orch_doc.get("node_name"),  # NOUVEAU
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
        tags = t.get("tags", []) or []
        test_play_args = t.get("play_args")
        play_args_per_cfg: Dict[str, Optional[str]] = {}
        for c in t["configs"]:
            play_args_per_cfg[c["name"]] = c.get("play_args")
        tests_data[name] = {"params": params, "configs": cfgs, "vals": vals,
                            "tags": tags, "test_play_args": test_play_args,
                            "play_args_per_cfg": play_args_per_cfg}
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
                  mode_yaml_ref: Path,
                  record_topics: List[str]) -> List[str]:
    errors: List[str] = []

    for req in ("experiment", "lidar", "mode", "bag_path", "ref_tum"):
        if req not in orch or orch[req] in (None, "", []):
            errors.append(f"[orchestrator] Champ requis manquant: {req}")

    if not orch.get("launch_file"):
        launches_map = orch.get("launches", {})
        if orch.get("lidar") not in (set(DEFAULT_LAUNCHES.keys()) | set(launches_map.keys())):
            exp = ", ".join(sorted(set(DEFAULT_LAUNCHES.keys()) | set(launches_map.keys())))
            errors.append(f"[orchestrator] Lidar inconnu: {orch.get('lidar')} (attendu: {exp})")

    if orch.get("mode") not in ("indoor", "outdoor"):
        errors.append("[orchestrator] Mode doit être 'indoor' ou 'outdoor'.")

    bag_path = Path(orch.get("bag_path", ""))
    if not bag_path.exists():
        errors.append(f"[orchestrator] bag_path introuvable: {bag_path}")
    else:
        if bag_path.is_file() and bag_path.suffix.lower() not in (".db3", ".mcap"):
            errors.append(f"[orchestrator] bag_path fichier doit être .db3 ou .mcap: {bag_path}")

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
            yaml_ref = yaml.safe_load(read_text_safe(yaml_ref_path))
        except Exception as e:
            errors.append(f"[validation] Impossible de charger YAML de référence '{yaml_ref_path}': {e}")

    if not record_topics:
        errors.append("[orchestrator] Aucun topic à enregistrer (record_topics vide).")

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
                    if pth == "__play_args__":
                        continue
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
                extra = sorted(set(vals.get(c, {}).keys()) - set(params) - {"__play_args__"})
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
    launch_cmd: str,
    yaml_path: Path,
    orig_yaml: Path,
    *,
    wait_pub: bool,
    param_max_tries: int,
    param_sleep_s: float,
    ws_dir: Path,
    record_topics: List[str],
    play_args: str,
    node_name: str,                 # NOUVEAU
    record_storage: str = "sqlite3",
):
    key = f"{test}/{cfg}"
    report[key] = {
        "steps": {}, "status": "ok", "artefacts": {}, "commands": {},
        "record_topics": record_topics, "play_args": play_args, "node_name": node_name
    }

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
        topics_str = " ".join(shlex.quote(t) for t in record_topics)
        cmd = (
            f'bash -lc "cd {quote(ws_dir)} && '
            f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
            f'exec ros2 bag record -o {quote(bag_dir)} --storage {shlex.quote(record_storage)} {topics_str}"'
        )
        report[key]["commands"]["record"] = cmd
        typer.echo(f"▶ {cmd}")
        if dry_run:
            return
        p = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                             start_new_session=True, env=run_env,
                             stdout=open(exp_dir / test / cfg / "logs" / "record.log", "w"),
                             stderr=subprocess.STDOUT)
        report[key]["artefacts"]["bag_proc"] = p
        time.sleep(SPAWN_GRACE_S)

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
        report[key]["commands"]["launch"] = launch_cmd
        typer.echo(f"▶ {launch_cmd}")
        if dry_run:
            return
        p = subprocess.Popen(launch_cmd, shell=True, executable="/bin/bash",
                             start_new_session=True, env=run_env,
                             stdout=open(exp_dir / test / cfg / "logs" / "launch.log", "w"),
                             stderr=subprocess.STDOUT)
        report[key]["artefacts"]["slam_proc"] = p
        time.sleep(SPAWN_GRACE_S)

    @record_step("check_params_runtime")
    def step_check_params():
        """Interroge directement le nœud donné (plus AUCUNE découverte)."""
        if dry_run:
            report[key]["artefacts"]["effective_params"] = {}
            return

        time.sleep(2.0)

        lidar_node_name = node_name  # utilisation directe du nom fourni
        report[key]["artefacts"]["lidar_node_name"] = lidar_node_name
        typer.echo(f"▶ Nœud SLAM: {lidar_node_name}")

        # (optionnel) attendre /slam_odom
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

        # Lecture des paramètres (sans daemon) avec backoff
        effective: Dict[str, str] = {}
        tries = max(1, int(param_max_tries))
        base_sleep = max(0.05, float(param_sleep_s))

        for pth in values.keys():
            pn = yaml_to_param_name(pth)
            out_text = ""
            rc = 1
            sleep_s = base_sleep
            cmd = (
                f'bash -lc "cd {quote(ws_dir)} && '
                f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
                f'ros2 --no-daemon param get {shlex.quote(lidar_node_name)} {shlex.quote(pn)}"'
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

    @record_step("snapshot_runtime")
    def step_snapshot():
        """Sauvegarde juste avant play: nodes, topics, param dump"""
        if dry_run:
            return
        logs_dir = exp_dir / test / cfg / "logs"
        ensure_dir(logs_dir)
        rc,o,e = run_capture_text('bash -lc "ros2 --no-daemon node list || true"')
        (logs_dir / "node_list.txt").write_text(o or e, encoding="utf-8")
        rc,o,e = run_capture_text('bash -lc "ros2 --no-daemon topic list || true"')
        (logs_dir / "topic_list.txt").write_text(o or e, encoding="utf-8")
        node = report[key]["artefacts"].get("lidar_node_name")
        if node:
            cmd = f'bash -lc "ros2 --no-daemon param dump {shlex.quote(node)} || true"'
            rc,o,e = run_capture_text(cmd)
            (logs_dir / "param_dump.yaml").write_text(o or e, encoding="utf-8")
        report[key]["artefacts"]["snapshots"] = {
            "node_list": str(logs_dir / "node_list.txt"),
            "topic_list": str(logs_dir / "topic_list.txt"),
            "param_dump": str(logs_dir / "param_dump.yaml") if node else None
        }

    @record_step("play_lidar")
    def step_play_lidar():
        args_str = play_args.strip()
        play_target = bag_path  # fichier .db3/.mcap ou dossier rosbag2
        cmd = (
            f'bash -lc "cd {quote(ws_dir)} && '
            f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
            f'exec ros2 bag play {args_str} {quote(play_target)}"'
        )
        report[key]["commands"]["play"] = cmd
        if dry_run:
            return 0
        with open(exp_dir / test / cfg / "logs" / "play.log", "w") as f:
            res = subprocess.run(cmd, shell=True, executable="/bin/bash", env=run_env,
                                 stdout=f, stderr=subprocess.STDOUT)
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
        step_snapshot()
        step_play_lidar()
    finally:
        step_stop_restore()

# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------
@app.command()
def main(
    # --- Portabilité chemins / env ---
    ws: Optional[Path] = typer.Option(None, "--ws", help="Workspace ROS 2 (défaut: ~/test_ws ou $SLAM_WS)"),
    base_dir: Optional[Path] = typer.Option(None, "--base-dir", help="Dossier racine des expériences"),
    active_yaml_indoor: Optional[Path] = typer.Option(None, "--active-yaml-indoor", help="YAML actif indoor"),
    active_yaml_outdoor: Optional[Path] = typer.Option(None, "--active-yaml-outdoor", help="YAML actif outdoor"),
    orig_yaml_indoor: Optional[Path] = typer.Option(None, "--orig-yaml-indoor", help="YAML d'origine indoor"),
    orig_yaml_outdoor: Optional[Path] = typer.Option(None, "--orig-yaml-outdoor", help="YAML d'origine outdoor"),
    # --- Build & packages ---
    install_mode: str = typer.Option("symlink", "--install-mode", help="Colcon install mode: symlink|merge"),
    odom_pkg_path: Optional[Path] = typer.Option(None, "--odom-pkg-path", help="Chemin package odom_csv_extractor (optionnel)"),
    # --- Plan / sélection ---
    plan: Optional[Path] = typer.Option(None, "--plan", "-p", help="Chemin du plan (.yaml/.yml)"),
    only_tests: Optional[str] = typer.Option(None, "--only-tests", help="Liste CSV de tests à exécuter (noms)."),
    tags: Optional[str] = typer.Option(None, "--tags", help="Filtrer les tests par tags (CSV, OR logique)"),
    resume: bool = typer.Option(True, "--resume/--no-resume", help="Respecter le checkpoint et ignorer les configs déjà 'done'."),
    # --- ROS & exécution ---
    dry_run: bool = typer.Option(False, "--dry-run", help="N'exécute pas les commandes ROS."),
    skip_build: bool = typer.Option(False, "--skip-build", help="Ne pas lancer colcon build."),
    skip_post: bool = typer.Option(False, "--skip-post", help="Ne pas lancer les scripts de post-traitement."),
    wait_pub: bool = typer.Option(False, "--wait-pub", help="Attendre un publisher sur /slam_odom avant la lecture des paramètres."),
    param_max_tries: int = typer.Option(PARAM_CHECK_MAX_TRIES, "--param-max-tries", help="Nb max d'essais pour ros2 param get."),
    param_sleep_s: float = typer.Option(PARAM_CHECK_SLEEP_S, "--param-sleep", help="Sleep initial entre essais (backoff inclus)."),
    # --- Topics & play ---
    record_topics_cli: Optional[str] = typer.Option(None, "--record-topics", help='Topics à enregistrer (ex: "/slam_odom /slam_confidence")'),
    record_storage: str = typer.Option("sqlite3", "--record-storage", help="Plugin rosbag2 pour l'enregistrement: sqlite3|mcap"),
    play_args_cli: Optional[str] = typer.Option(None, "--play-args", help='Arguments supplémentaires pour "ros2 bag play" (ex: "--clock --rate 1.0")'),
    # --- Node name (suppression découverte auto) ---
    node_name_opt: Optional[str] = typer.Option(None, "--node-name", help="Nom complet du nœud à interroger (ex: /lidar_slam)"),
):
    typer.secho("🛠️  Orchestrateur SLAM KITWARE (portable+rapports enrichis)", fg=typer.colors.BLUE)

    # ====== Résolution des chemins (CLI > env > défauts) ======
    WS_DIR = (ws or env_path("SLAM_WS", DEFAULT_WS_DIR)).resolve()
    SRC_DIR = (WS_DIR / "src").resolve()
    BASE_DIR = (base_dir or env_path("SLAM_BASE_DIR", DEFAULT_BASE_DIR)).resolve()

    # Downloads dynamiques (pour ORIG_x par défaut)
    downloads_dir = detect_downloads_dir()
    default_orig_out = downloads_dir / "slam_config_outdoor.yaml"
    default_orig_in  = downloads_dir / "slam_config_indoor.yaml"

    OUTDOOR_YAML = (active_yaml_outdoor or env_path("SLAM_ACTIVE_YAML_OUT", DEFAULT_OUTDOOR_YAML)).resolve()
    INDOOR_YAML  = (active_yaml_indoor  or env_path("SLAM_ACTIVE_YAML_IN",  DEFAULT_INDOOR_YAML)).resolve()
    ORIG_OUT_YAML = (orig_yaml_outdoor or env_path("SLAM_ORIG_YAML_OUT", default_orig_out)).resolve()
    ORIG_IN_YAML  = (orig_yaml_indoor  or env_path("SLAM_ORIG_YAML_IN",  default_orig_in)).resolve()

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

    # 3) Résolution des launches (custom/plan/défaut)
    launches_map = orch.get("launches", {}) or {}
    if orch.get("launch_file"):
        launch_file = orch["launch_file"]
    else:
        lidar = orch.get("lidar")
        launch_file = launches_map.get(lidar, DEFAULT_LAUNCHES.get(lidar))

    # 4) record_topics & play_args (priorité: CLI > plan orch > test > config)
    def parse_topics(s: Optional[str]) -> List[str]:
        if not s:
            return []
        return [t for t in s.strip().split() if t.strip()]

    record_topics = parse_topics(record_topics_cli) or parse_topics(orch.get("record_topics")) or ["/slam_odom", "/slam_confidence"]
    play_args_orch = (play_args_cli if play_args_cli is not None else (orch.get("play_args") or ""))

    # 5) Validation (types vs YAML de référence ROS 2)
    mode = orch.get("mode", "outdoor")
    yaml_ref = ORIG_IN_YAML if mode == "indoor" else ORIG_OUT_YAML
    if not yaml_ref.exists():
        yaml_ref = INDOOR_YAML if mode == "indoor" else OUTDOOR_YAML

    errors = validate_plan(orch, tests_data, yaml_ref, record_topics)
    if errors:
        typer.secho("\n❌ Le plan est invalide. Détails :", fg=typer.colors.RED)
        for err in errors:
            typer.echo(f"  - {err}")
        typer.secho("\nAbandon (corrigez le plan YAML puis relancez).", fg=typer.colors.RED)
        raise typer.Exit(1)

    # 6) Métadonnées validées
    exp        = orch["experiment"]
    lidar      = orch["lidar"]
    if not launch_file:
        typer.secho("❌ Aucun launch file résolu (ni custom, ni map, ni défaut).", fg=typer.colors.RED)
        raise typer.Exit(1)
    bag_path   = Path(orch["bag_path"]).resolve()
    ref_tum    = Path(orch["ref_tum"]).resolve()
    yaml_path  = INDOOR_YAML if mode == "indoor" else OUTDOOR_YAML
    orig_yaml  = ORIG_IN_YAML if mode == "indoor" else ORIG_OUT_YAML

    # 6bis) Nom du nœud effectif (CLI > YAML > défaut)
    node_name_eff = node_name_opt or orch.get("node_name") or "/lidar_slam"

    typer.secho("✅ Plan validé, lancement de la campagne…", fg=typer.colors.GREEN)

    # 7) Filtrage tests (only-tests + tags)
    test_names = list(tests_data.keys())
    if only_tests:
        keep = {t.strip() for t in only_tests.split(",") if t.strip()}
        test_names = [t for t in test_names if t in keep]
    if tags:
        want = {tag.strip() for tag in tags.split(",") if tag.strip()}
        def has_tag(tn: str) -> bool:
            tags_t = set(tests_data[tn].get("tags", []) or [])
            return bool(want & tags_t)
        test_names = [t for t in test_names if has_tag(t)]
    if not test_names:
        typer.secho("❌ Aucun test à exécuter après filtrage.", fg=typer.colors.RED)
        raise typer.Exit(1)

    # 8) Dossiers & index
    ensure_dir(BASE_DIR)
    exp_dir = make_dirs(BASE_DIR, exp, tests_data)
    index = WS_DIR / f"{exp}.txt"
    index.write_text("\n".join(str(exp_dir / t) for t in test_names))

    # JSON par config
    for t in test_names:
        td = tests_data[t]
        for c in td["configs"]:
            d = exp_dir / t / c
            ensure_dir(d)
            (d / f"{c}.json").write_text(json.dumps(td["vals"][c], indent=2))

    # 9) Rapport & checkpoint + méta versions
    meta = get_versions(os.environ.copy())
    meta["node_name_defaulted"] = node_name_opt is None and orch.get("node_name") is None
    report: Dict[str, Any] = {"meta": meta, "post_scripts": []}
    checkpoint = load_checkpoint(exp_dir / "checkpoints.json")

    # 10) Build unique — colcon (odom_csv_extractor optionnel)
    have_odom = False
    if not skip_build:
        odom_dir = odom_pkg_path.resolve() if odom_pkg_path else find_package_dir(WS_DIR, "odom_csv_extractor")
        base_paths: List[str] = []
        wrap_dir = SRC_DIR / "ros2_wrapping"
        if wrap_dir.exists():
            base_paths.append(quote(wrap_dir))
        if odom_dir and odom_dir.exists():
            have_odom = True
            base_paths.append(quote(odom_dir))
        colcon_flag = "--symlink-install" if install_mode == "symlink" else "--merge-install"
        if not base_paths:
            typer.secho("⚠️  Aucun chemin de build détecté (ni ros2_wrapping, ni odom_csv_extractor).", fg=typer.colors.YELLOW)
        build_cmd = (
            f'bash -lc "cd {quote(WS_DIR)} && '
            f'if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then source /opt/ros/$ROS_DISTRO/setup.bash; fi; '
            f'colcon build {colcon_flag} ' +
            (f'--base-paths {" ".join(base_paths)}' if base_paths else '') +
            '"'
        )
        typer.echo(f"▶ Build once: {build_cmd}")
        if not dry_run:
            rc = run_cmd(build_cmd, dry_run=False)
            if rc != 0:
                typer.secho(f"❌ Échec build colcon (code {rc})", fg=typer.colors.RED)
                raise typer.Exit(1)
        if have_odom and not dry_run:
            check_cmd = (
                f'bash -lc "cd {quote(WS_DIR)} && '
                f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
                f'ros2 run odom_csv_extractor extract_to_csv -h >/dev/null 2>&1"'
            )
            rc = run_cmd(check_cmd, dry_run=False)
            if rc != 0:
                typer.secho("❌ 'odom_csv_extractor' introuvable après build.", fg=typer.colors.RED)
                raise typer.Exit(1)

    # 11) Boucle d’exécution
    for t in test_names:
        td = tests_data[t]
        test_play_args = td.get("test_play_args") or ""
        for c in td["configs"]:
            key = f"{t}/{c}"
            if resume and checkpoint.get(key) == "done":
                typer.secho(f"↩️ Ignored: {key}", fg=typer.colors.CYAN)
                continue

            # play_args precedence: CLI > config > test > orch > ""
            cfg_play_args = (td.get("play_args_per_cfg") or {}).get(c) or ""
            play_args_effective = (play_args_cli if play_args_cli is not None else
                                   (cfg_play_args or test_play_args or play_args_orch or ""))

            # Commande de launch (arg outdoor basé sur le mode)
            outdoor_arg = 'true' if mode == "outdoor" else 'false'
            launch_cmd = (
                f'bash -lc "cd {quote(WS_DIR)} && '
                f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
                f'exec ros2 launch lidar_slam {shlex.quote(launch_file)} outdoor:={outdoor_arg}"'
            )

            values = td["vals"][c]
            try:
                process_config(exp_dir, t, c, values, bag_path, dry_run,
                               report, launch_cmd, yaml_path, orig_yaml,
                               wait_pub=wait_pub,
                               param_max_tries=param_max_tries,
                               param_sleep_s=param_sleep_s,
                               ws_dir=WS_DIR,
                               record_topics=record_topics,
                               play_args=play_args_effective,
                               node_name=node_name_eff,
                               record_storage=record_storage)
                checkpoint[key] = "done"
                save_checkpoint(exp_dir / "checkpoints.json", checkpoint)
            except Exception as e:
                typer.secho(f"❌ Erreur pendant {key}: {e}", fg=typer.colors.RED)

    # 12) Extraction CSV (odom_csv_extractor) — optionnel
    if have_odom:
        run_cmd(
            f'bash -lc "cd {quote(WS_DIR)} && '
            f'if [ -f install/setup.bash ]; then source install/setup.bash; fi; '
            f'ros2 run odom_csv_extractor extract_to_csv -d {quote(index)}"',
            dry_run
        )
    else:
        typer.secho("ℹ️  odom_csv_extractor absent — étape CSV ignorée.", fg=typer.colors.YELLOW)

    # 13) Post-traitements (avec statut visible dans report)
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
                                  ref=ref_tum)
            if script.endswith('.sh'):
                cmd = f"bash {quote(path)} {cmd_args}"
            else:
                cmd = f"chmod +x {quote(path)} && python3 {quote(path)} {cmd_args}"
            typer.echo(f"▶ {cmd}")
            ps_entry = {"script": script, "cmd": cmd, "status": "ok", "returncode": 0, "duration_s": None}
            t0 = time.time()
            if not dry_run:
                try:
                    subprocess.run(cmd, shell=True, executable="/bin/bash", check=True)
                except CalledProcessError as e:
                    ps_entry["status"] = "error"
                    ps_entry["returncode"] = int(e.returncode)
                    typer.secho(f"❌ Erreur {script} code {e.returncode}", fg=typer.colors.RED)
            ps_entry["duration_s"] = round(time.time() - t0, 3)
            report["post_scripts"].append(ps_entry)

    # 14) Déplacement index
    dirs = exp_dir / "dirs"
    dirs.mkdir(exist_ok=True)
    try:
        shutil.move(str(index), str(dirs / index.name))
    except Exception:
        pass

    # 15) Rapports enrichis
    out_json = exp_dir / "report.json"
    out_html = exp_dir / "report.html"
    out_json.write_text(json.dumps(clean_for_json(report), indent=2))

    tpl = ENV.from_string(
        """<!doctype html><html><body>
        <h1>Rapport {{exp}}</h1><p>{{now}}</p>

        <h2>Méta</h2>
        <ul>
          {% for k,v in report.meta.items() %}
            <li><b>{{k}}</b>: {{v}}</li>
          {% endfor %}
        </ul>

        <h2>Configs</h2>
        <table border="1" cellpadding="6">
          <tr><th>Config</th><th>Status</th><th>Durées (s)</th><th>Record topics</th><th>Play args</th><th>Node</th></tr>
          {% for k,v in report.items() if k not in ('meta','post_scripts') %}
            <tr>
              <td>{{k}}</td>
              <td>{{v.status}}</td>
              <td>
                {% for s,dt in v.steps.items() %}
                  <div>{{ s }}: {{ "%.2f"|format(dt) }}</div>
                {% endfor %}
              </td>
              <td>{{ " ".join(v.record_topics) if v.record_topics else "" }}</td>
              <td>{{ v.play_args }}</td>
              <td>{{ v.node_name }}</td>
            </tr>
          {% endfor %}
        </table>

        <h2>Post-scripts</h2>
        <table border="1" cellpadding="6">
          <tr><th>Script</th><th>Status</th><th>RC</th><th>Durée (s)</th><th>Commande</th></tr>
          {% for ps in report.post_scripts %}
            <tr>
              <td>{{ ps.script }}</td>
              <td>{{ ps.status }}</td>
              <td>{{ ps.returncode }}</td>
              <td>{{ "%.2f"|format(ps.duration_s or 0) }}</td>
              <td><code>{{ ps.cmd }}</code></td>
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

