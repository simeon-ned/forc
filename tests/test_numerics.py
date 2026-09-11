import numpy as np
import mujoco
from forc.models import quadrotor, arm
from forc.numerics import quat_exp, quat_mul, quat_matrix, tangent_error, dense_mass, rk4


def test_scalar_first_quarter_turn():
    q=quat_exp(np.array([0,0,np.pi/2]))
    assert np.allclose(q,[np.sqrt(0.5),0,0,np.sqrt(0.5)])
    assert np.allclose(quat_matrix(q)@np.array([1,0,0]),[0,1,0],atol=1e-14)

def test_quaternion_product_and_local_velocity():
    q=quat_exp(np.array([0.3,-0.2,0.1])); w=np.array([0.2,0.5,-0.7])
    expected=quat_mul(q,quat_exp(0.01*w))
    actual=q.copy(); mujoco.mju_quatIntegrate(actual,w,0.01)
    assert np.allclose(actual,expected,atol=1e-14)

def test_tangent_dimensions_and_roundtrip():
    m=quadrotor(); d=mujoco.MjData(m); reference=d.qpos.copy()
    delta=np.array([0.1,0.2,-0.1,0.02,-0.03,0.04])
    mujoco.mj_integratePos(m,d.qpos,delta,1.)
    e=tangent_error(m,reference,d.qpos,np.zeros(m.nv),d.qvel)
    assert (m.nq,m.nv,e.shape)==(7,6,(12,))
    assert np.allclose(e[:6],delta,atol=1e-12)
    d.qpos[3:7]*=-1
    assert np.allclose(tangent_error(m,reference,d.qpos,np.zeros(6),d.qvel),e,atol=1e-12)

def test_mass_and_force_contract():
    m=arm(); d=mujoco.MjData(m)
    d.qpos[:]=[0.2,-0.3]; d.qvel[:]=[0.4,-0.1]; d.ctrl[:]=[1.,-1.]
    mujoco.mj_forward(m,d); M=dense_mass(m,d)
    assert np.allclose(M,M.T)
    assert np.linalg.eigvalsh(M).min()>0
    applied=d.qfrc_actuator+d.qfrc_passive+d.qfrc_applied+d.qfrc_constraint
    assert np.linalg.norm(M@d.qacc+d.qfrc_bias-applied)<1e-10

def test_rk4_scalar_exponential():
    x=np.array([1.])
    for _ in range(100): x=rk4(lambda x:-x,x,0.01)
    assert abs(x[0]-np.exp(-1))<1e-9
