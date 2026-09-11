"""Numerical and MuJoCo utilities, local to this course repository."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
import mujoco

def dense_mass(model, data):
    """MuJoCo 3.13 signature. Call mj_forward before requesting derived quantities."""
    result = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, data, result)
    return result

def tangent_error(model, reference_q, q, reference_v, v):
    dq = np.empty(model.nv)
    mujoco.mj_differentiatePos(model, dq, 1.0, reference_q, q)
    return np.concatenate((dq, v - reference_v))

def quat_mul(a, b):
    w, v = a[0], np.asarray(a[1:])
    s, r = b[0], np.asarray(b[1:])
    return np.r_[w*s-v@r, w*r+s*v+np.cross(v,r)]

def quat_exp(phi):
    phi = np.asarray(phi, dtype=float)
    theta = np.linalg.norm(phi)
    # np.sinc(t) = sin(pi*t)/(pi*t)
    return np.r_[np.cos(theta/2), 0.5*np.sinc(theta/(2*np.pi))*phi]

def quat_matrix(q):
    out = np.empty(9)
    mujoco.mju_quat2Mat(out, np.asarray(q, dtype=float))
    return out.reshape(3, 3)

def rk4(f, x, dt):
    k1=f(x); k2=f(x+dt*k1/2); k3=f(x+dt*k2/2); k4=f(x+dt*k3)
    return x+dt*(k1+2*k2+2*k3+k4)/6

def run_cli(experiment, practice_dir):
    parser=argparse.ArgumentParser(description=experiment.__doc__)
    parser.add_argument("--config", type=Path, default=practice_dir/"config.yml")
    parser.add_argument("--output", type=Path)
    args=parser.parse_args()
    with args.config.open() as stream:
        config=yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Configuration must be a YAML mapping.")
    dt=config.get("dt", 0.01)
    if not isinstance(dt, (int,float)) or not np.isfinite(dt) or dt<=0:
        raise ValueError("dt must be a finite positive number.")
    if not isinstance(config.get("seed", 0), int):
        raise ValueError("seed must be an integer.")
    output=args.output or practice_dir.parents[1]/"outputs"/practice_dir.name
    output.mkdir(parents=True, exist_ok=True)
    metrics, trace=experiment(config)
    result={"experiment":practice_dir.name, "mujoco_version":mujoco.__version__,
            "config":config, "metrics":metrics}
    encoded=json.dumps(result, indent=2, allow_nan=False)
    (output/"metrics.json").write_text(encoded+"\n")
    if trace is not None:
        columns, values=trace
        np.savetxt(output/"trajectory.csv", values, delimiter=",",
                   header=",".join(columns), comments="")
    print(encoded)
