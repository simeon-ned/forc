"""Physics and control checks independent of the rendered teaching material."""
import numpy as np
import pytest
from scipy.integrate import solve_ivp
from scipy.signal import cont2discrete
from pathlib import Path
import runpy
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def cb():
    run_notebook = runpy.run_path(str(ROOT / "scripts/run_practices.py"))["run_notebook"]
    return SimpleNamespace(**run_notebook(ROOT / "practices/p02-control-basics/tutorial.ipynb"))


def test_spring_force_law_and_motor_equilibrium(cb):
    spring=cb.mass_spring(mass=2., damping=3., stiffness=7.)
    for q,v,u in [(0.3,-.4,1.7),(-.8,.2,-2.)]:
        np.testing.assert_allclose(cb.derivative(spring,[q,v],[u]), [v,(u-3*v-7*q)/2])
    motor=cb.rotor(inertia=2., damping=3.)
    np.testing.assert_allclose(cb.derivative(motor,[.8,-.4],[1.2]),[-.4,1.2])
    A,B=cb.linearize(motor)
    np.testing.assert_allclose(np.sort(np.linalg.eigvals(A)),[-1.5,0])


def test_flexible_coupling_and_poles(cb):
    m=cb.flexible_drive(); A,B=cb.linearize(m)
    expected=np.array([[0,0,1,0],[0,0,0,1],[-200,200,-2,0],[200,-200,0,-2.]])
    np.testing.assert_allclose(A,expected,atol=1e-9)
    np.testing.assert_allclose(B,[[0],[0],[0],[1]])
    # Nonzero spring twist generates equal and opposite restoring torques.
    np.testing.assert_allclose(cb.derivative(m,[.1,0,0,0],[0]),[0,0,-20,20])
    poles=[-1,-1.1,-1.5,-2];K=cb.place(A,B,poles)
    np.testing.assert_allclose(np.sort(np.linalg.eigvals(A-B@K)),np.sort(poles),atol=1e-7)
    with pytest.raises(ValueError,match='controllability'):
        cb.place(np.diag([-1.,-2.]),np.array([[1.],[0.]]),[-3,-4])


def test_cart_nonlinear_equations_and_linearization(cb):
    m=cb.cart_pole();A,B=cb.linearize(m);Aa,Ba=cb.cart_matrices()
    np.testing.assert_allclose(A,Aa,atol=1e-8);np.testing.assert_allclose(B,Ba,atol=1e-8)
    for theta,omega,u in [(.2,-.4,1.7),(1.2,.7,-30.),(-.8,1.1,4.)]:
        M,mass,L,J,g=2.,.5,.3,.0001,9.81
        mass_matrix=np.array([[M+mass,-mass*L*np.cos(theta)],
                              [-mass*L*np.cos(theta),mass*L**2+J]])
        force=np.array([u-mass*L*np.sin(theta)*omega**2,mass*g*L*np.sin(theta)])
        accel=np.linalg.solve(mass_matrix,force)
        np.testing.assert_allclose(cb.derivative(m,[.3,theta,-.2,omega],[u]),np.r_[[-.2,omega],accel],atol=1e-10)
    K=cb.place(A,B,[-1,-2,-3,-4])
    Ad,Bd,_,_,_=cont2discrete((A,B,np.eye(4),np.zeros((4,1))),.002)
    assert max(abs(np.linalg.eigvals(Ad-Bd@K)))<1
    with pytest.raises(ValueError,match='equilibrium'):
        cb.linearize(m,[0,.1,0,0])


def test_vector_field_equilibria_and_sampled_realization_converge(cb):
    for q in cb.EQUILIBRIA:
        np.testing.assert_allclose(cb.field(q),0)
        eps=1e-6
        jac=np.column_stack([(cb.field(q+eps*e)-cb.field(q-eps*e))/(2*eps) for e in np.eye(2)])
        np.testing.assert_allclose(cb.field_jacobian(q),jac,atol=1e-8)
        eigenvalues=np.linalg.eigvals(jac)
        assert bool(np.all(eigenvalues.real<0)) == (q[0] in [-1,1])
    errors=[]
    for dt in [.002,.001]:
        q0=np.array([-2.,-1.]);x0=np.r_[q0,cb.field(q0)]
        trace=cb.rollout(cb.vector_field_model(dt=dt),x0,cb.field_controller,duration=2.)
        original=solve_ivp(lambda t,q:cb.field(q),[0,trace.time[-1]],q0,t_eval=trace.time,rtol=1e-11,atol=1e-12)
        errors.append(np.max(abs(trace.x[:,:2]-original.y.T)))
    assert errors[0]<.02
    assert errors[1]<.6*errors[0]


def test_published_experiments_and_second_order_response(cb):
    cases = {
        "spring-free": (cb.spring_free, "spring", None),
        "spring-forced": (cb.spring_forced, "spring", None),
        "motor-free": (cb.motor_free, "rotor", None),
        **{"motor-" + name: (trace, "rotor", None) for name, trace in cb.motor_runs.items()},
        "flexible-pd": (cb.flex_pd, "flexible", None),
        "flexible-poles": (cb.flex_poles, "flexible", cb.flex_linear),
        "nonlinear-field": (cb.field_run, "field", None),
        "cart-small": (cb.cart_small, "cart", cb.small_linear),
        "cart-large": (cb.cart_large, "cart", cb.large_linear),
    }
    assert len(cases)==11
    for trace,kind,reference in cases.values():
        assert np.isfinite(trace.x).all()
        assert trace.x.shape==(len(trace.time),trace.model.nq+trace.model.nv)
        assert len(trace.applied)==len(trace.time)
        assert trace.time[-1] == (1.5 if kind=='cart' and len(trace.time)<1000 else 10.)
    np.testing.assert_allclose(cases['spring-forced'][0].x[-1],[.4,0],atol=1e-4)
    np.testing.assert_allclose(cases['motor-free'][0].x[-1],[1.5,0],atol=1e-7)
    predicted=np.exp(-np.pi*.35/np.sqrt(1-.35**2))
    measured=cases['motor-underdamped'][0].x[:,0].max()-1
    assert abs(predicted-measured)<.003
    critical=cases['motor-critical'][0].x[:,0]
    assert np.min(np.diff(critical))>=-1e-10
    trace,_,linear=cases['flexible-poles']
    assert np.max(abs(trace.x-linear))<1e-5
    small=cases['cart-small'][0]
    assert np.max(abs(small.x[-1]))<1e-3
    large=cases['cart-large'][0]
    assert abs(large.x[-1,1])>1
    assert np.max(abs(large.applied))<=30
    assert np.any(abs(large.requested)>30)


def test_tutorial_is_self_contained_and_has_no_saved_results():
    import ast
    import json
    from pathlib import Path
    folder = Path(__file__).resolve().parents[1] / "practices/p02-control-basics"
    notebook = json.loads((folder / "tutorial.ipynb").read_text())
    code = "\n".join("".join(c["source"]) for c in notebook["cells"] if c["cell_type"] == "code")
    tree = ast.parse(code)
    imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    assert not any(name and name.startswith("forc") for name in imports)
    functions = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}
    assert {"mass_spring", "rotor", "flexible_drive", "cart_pole", "vector_field_model",
            "rollout", "linearize", "simulation_frames", "show_run"} <= functions
    assert all(not cell.get("outputs") for cell in notebook["cells"])
    attachments = {name for cell in notebook["cells"] for name in cell.get("attachments", {})}
    assert attachments == {"feedback.svg", "mass-spring.svg", "motor.svg", "flexible-drive.svg", "cart-pole.svg"}
    assert {f.name for f in folder.iterdir() if not f.name.startswith(".")} == {"README.md", "practice.yml", "tutorial.ipynb"}
    assert not (folder / "videos").exists()
