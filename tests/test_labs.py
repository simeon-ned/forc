from pathlib import Path
import numpy as np
import yaml
from forc import labs

ROOT=Path(__file__).resolve().parents[1]
def config(name):
    return yaml.safe_load((ROOT/"practices"/name/"config.yml").read_text())

def test_lqr_scalar_solution():
    K,P=labs.dlqr(np.ones((1,1)),np.ones((1,1)),np.ones((1,1)),np.ones((1,1)))
    assert np.allclose(P,(1+np.sqrt(5))/2)
    assert np.allclose(K,0.6180339887498949)

def test_robot_lqr_stabilizes():
    result,_=labs.lqr(config("p03-lqr"))
    for r in result.values():
        assert r["equilibrium_acceleration"]<1e-10
        assert r["closed_loop_spectral_radius"]<1
        assert r["final_error"]<r["initial_error"]*0.01

def test_manipulator_controller_and_ik():
    result,_=labs.manipulator(config("p02-manipulator"))
    assert result["computed_torque"]["final_joint_error"]<1e-5
    assert result["bias_pd"]["final_joint_error"]<1e-4
    assert result["differential_ik"]["final_task_error"]<1e-5

def test_predictive_constraints_and_reproducibility():
    cfg=config("p04-predictive-control"); cfg.update(steps=12,horizon=10,samples=32)
    first,_=labs.predictive(cfg); second,_=labs.predictive(cfg)
    assert first["mpc"]["height_violations"]==0
    assert first["mpc"]["solver_failures"]==0
    assert first["weighted_shooting"]["rms_height_error"]==second["weighted_shooting"]["rms_height_error"]

def test_dagger_aggregates_expert_labels():
    cfg=config("p05-imitation"); cfg.update(episodes=3,steps=30,rounds=2)
    result,_=labs.imitation(cfg)
    assert [r["expert_labels"] for r in result["iterations"]]==[90,180,270]
    assert all(np.isfinite(r["rollout_cost"]) for r in result["iterations"])

def test_q_learning_goal_regulation():
    result,_=labs.reinforcement(config("p06-reinforcement-learning"))
    assert result["learned"]["success_rate"]>=0.95
    assert result["learned"]["mean_cost"]<result["random"]["mean_cost"]
    assert result["baseline"]["success_rate"]==1.0
