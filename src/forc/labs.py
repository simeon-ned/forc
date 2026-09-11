"""Headless reference experiments for FORC. Each function returns metrics and a trace."""
import time
import numpy as np
import mujoco
from scipy.linalg import solve_discrete_are
from scipy.optimize import minimize
from . import models
from .numerics import dense_mass, tangent_error

def dlqr(A, B, Q, R):
    P=solve_discrete_are(A,B,Q,R)
    return np.linalg.solve(R+B.T@P@B, B.T@P@A), P

def simulation(cfg):
    """Inspect model dimensions and the generalized force balance."""
    m=models.arm(cfg["dt"]); d=mujoco.MjData(m)
    d.qpos[:]=[0.4,-0.6]; d.qvel[:]=[0.2,-0.1]; d.ctrl[:]=[1,-1]
    mujoco.mj_forward(m,d)
    M=dense_mass(m,d)
    residual=M@d.qacc+d.qfrc_bias-d.qfrc_passive-d.qfrc_actuator-d.qfrc_applied-d.qfrc_constraint
    rows=[]
    for _ in range(cfg["steps"]):
        mujoco.mj_step(m,d); rows.append([d.time,*d.qpos,*d.qvel])
    free=models.quadrotor()
    return {"force_residual":float(np.linalg.norm(residual)),
            "minimum_mass_eigenvalue":float(np.linalg.eigvalsh(M).min()),
            "arm_nq":m.nq,"arm_nv":m.nv,"free_nq":free.nq,"free_nv":free.nv}, (["time","q1","q2","v1","v2"],rows)

def manipulator(cfg):
    """Compare joint controllers and demonstrate damped differential IK."""
    m=models.arm(cfg["dt"]); target=np.array([-0.3,0.6]); initial=np.array([0.6,-0.8])
    metrics={}; rows=[]
    for method in ("pd","bias_pd","computed_torque"):
        d=mujoco.MjData(m); d.qpos[:]=initial; errors=[]; clips=0
        for _ in range(cfg["steps"]):
            mujoco.mj_forward(m,d); e=d.qpos-target
            a=-cfg["kp"]*e-cfg["kd"]*d.qvel
            if method=="pd": u=a
            elif method=="bias_pd": u=a+d.qfrc_bias
            else: u=dense_mass(m,d)@a+d.qfrc_bias-d.qfrc_passive
            clipped=np.clip(u,-cfg["torque_limit"],cfg["torque_limit"])
            clips+=int(np.any(np.abs(u-clipped)>1e-10)); d.ctrl[:]=clipped
            mujoco.mj_step(m,d); errors.append(np.linalg.norm(d.qpos-target))
            if method=="computed_torque": rows.append([d.time,*d.qpos,*d.qvel,*clipped])
        metrics[method]={"rms_joint_error":float(np.sqrt(np.mean(np.square(errors)))),
                         "final_joint_error":float(errors[-1]),"saturation_fraction":clips/cfg["steps"]}
    ref=mujoco.MjData(m); ref.qpos[:]=target; mujoco.mj_forward(m,ref)
    tip=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,"tip"); goal=ref.site_xpos[tip].copy()
    d=mujoco.MjData(m); d.qpos[:]=[0.4,0.5]
    max_speed=0.
    for _ in range(cfg["steps"]):
        mujoco.mj_forward(m,d); J=np.zeros((3,m.nv)); Jr=np.zeros_like(J)
        mujoco.mj_jacSite(m,d,J,Jr,tip)
        b=3*(goal-d.site_xpos[tip])
        vel=J.T@np.linalg.solve(J@J.T+cfg["damping"]**2*np.eye(3),b)
        vel=np.clip(vel,-2,2); max_speed=max(max_speed,float(np.linalg.norm(vel)))
        mujoco.mj_integratePos(m,d.qpos,vel,cfg["dt"])
    mujoco.mj_forward(m,d)
    metrics["differential_ik"]={"final_task_error":float(np.linalg.norm(goal-d.site_xpos[tip])),"max_joint_speed":max_speed}
    return metrics,(["time","q1","q2","v1","v2","u1","u2"],rows)

def linearize_equilibrium(m, d):
    n=2*m.nv+m.na
    A=np.zeros((n,n)); B=np.zeros((n,m.nu))
    mujoco.mj_forward(m,d)
    mujoco.mjd_transitionFD(m,d,1e-6,1,A,B,None,None)
    return A,B

def lqr(cfg):
    """Stabilize cart-pole and full free-joint quadrotor near their equilibria."""
    metrics={}; trace=[]
    for name in ("cartpole","quadrotor"):
        m=getattr(models,name)(cfg["dt"]); ref=mujoco.MjData(m)
        if name=="quadrotor": ref.ctrl[:]=9.81/4
        mujoco.mj_forward(m,ref)
        qref=ref.qpos.copy(); vref=ref.qvel.copy(); uref=ref.ctrl.copy()
        equilibrium_acc=float(np.linalg.norm(ref.qacc))
        A,B=linearize_equilibrium(m,ref)
        Q=np.diag([15,50,2,3] if name=="cartpole" else [20,20,40,10,10,5,2,2,4,1,1,1])
        K,P=dlqr(A,B,Q,np.eye(m.nu)*0.1)
        d=mujoco.MjData(m); d.qpos[:]=qref
        if name=="cartpole": d.qpos[:]=[0.15,0.12]
        else:
            perturb=np.array([0.12,-0.10,0.12,0.06,-0.04,0.05])
            mujoco.mj_integratePos(m,d.qpos,perturb,1.0)
        start=float(np.linalg.norm(tangent_error(m,qref,d.qpos,vref,d.qvel)))
        errors=[]; clips=0
        for _ in range(cfg["steps"]):
            e=tangent_error(m,qref,d.qpos,vref,d.qvel)
            u=uref-K@e
            limited=np.clip(u,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
            clips+=int(np.any(np.abs(limited-u)>1e-10))
            d.ctrl[:]=limited; mujoco.mj_step(m,d)
            en=float(np.linalg.norm(tangent_error(m,qref,d.qpos,vref,d.qvel))); errors.append(en)
            if name=="quadrotor": trace.append([d.time,*d.qpos,*d.qvel])
        metrics[name]={"nq":m.nq,"nv":m.nv,"equilibrium_acceleration":equilibrium_acc,
            "closed_loop_spectral_radius":float(np.abs(np.linalg.eigvals(A-B@K)).max()),
            "initial_error":start,"final_error":errors[-1],
            "rms_tangent_error":float(np.sqrt(np.mean(np.square(errors)))),"saturation_fraction":clips/cfg["steps"]}
    return metrics,(["time","x","y","z","qw","qx","qy","qz","vx","vy","vz","wx","wy","wz"],trace)

def predictive(cfg):
    """Compare MPC, clipped LQR, and MPPI-inspired weighted shooting in vertical flight."""
    dt=cfg["dt"]; A=np.array([[1,dt],[0,1.]])
    B=np.array([[dt*dt/2],[dt]])
    Q=np.diag([12.,2.]); R=np.array([[0.2]])
    K,P=dlqr(A,B,Q,R); N=cfg["horizon"]; lo,hi=cfg["acceleration_bounds"]
    target=cfg["target_height"]; metrics={}; trace=[]
    def rollout(x,U):
        states=[]
        for u in U:
            x=A@x+B[:,0]*u; states.append(x.copy())
        return np.array(states)
    def objective(U,x):
        X=rollout(x,U)
        return float(np.einsum("ni,ij,nj->",X,Q,X)+0.2*(U@U)+X[-1]@P@X[-1])
    for method in ("clipped_lqr","mpc","weighted_shooting"):
        rng=np.random.default_rng(cfg["seed"]); x=np.array([cfg["initial_height"]-target,0.])
        U=np.zeros(N); errors=[]; violations=0; failures=0; times=[]; effective=[]
        for k in range(cfg["steps"]):
            tic=time.perf_counter()
            if method=="clipped_lqr": u=float(np.clip((-K@x).item(),lo,hi))
            elif method=="mpc":
                def limits(U):
                    z=rollout(x,U)[:,0]+target
                    return np.r_[z-cfg["height_bounds"][0],cfg["height_bounds"][1]-z]
                opt=minimize(objective,U,args=(x,),method="SLSQP",bounds=[(lo,hi)]*N,
                    constraints={"type":"ineq","fun":limits},options={"maxiter":60,"ftol":1e-6})
                if opt.success and np.min(limits(opt.x))>=-1e-5:
                    U=opt.x; u=float(U[0])
                else:
                    failures+=1; u=float(np.clip((-K@x).item(),lo,hi))
            else:
                candidates=np.clip(U+rng.normal(0,cfg["noise"],(cfg["samples"],N)),lo,hi)
                X=np.repeat(x[None,:],cfg["samples"],axis=0); scores=np.zeros(cfg["samples"])
                for j in range(N):
                    X=X@A.T+candidates[:,j,None]*B[:,0]
                    scores+=np.einsum("ni,ij,nj->n",X,Q,X)+0.2*candidates[:,j]**2
                    violation=np.maximum(cfg["height_bounds"][0]-(X[:,0]+target),0)+np.maximum(X[:,0]+target-cfg["height_bounds"][1],0)
                    scores+=1e4*violation**2
                scores+=np.einsum("ni,ij,nj->n",X,P,X)
                weights=np.exp(-(scores-scores.min())/cfg["temperature"]); weights/=weights.sum()
                # Use evaluated clipped candidates, not unbounded noise.
                U=weights@candidates; u=float(U[0]); effective.append(float(1/(weights@weights)))
            times.append(time.perf_counter()-tic)
            x=A@x+B[:,0]*u; z=x[0]+target
            errors.append(float(x[0])); violations+=int(z<cfg["height_bounds"][0]-1e-6 or z>cfg["height_bounds"][1]+1e-6)
            if method=="mpc": trace.append([(k+1)*dt,z,x[1],u])
            U=np.r_[U[1:],0.]
        metrics[method]={"rms_height_error":float(np.sqrt(np.mean(np.square(errors)))),
            "final_height_error":float(abs(x[0])),"height_violations":violations,"solver_failures":failures,
            "max_compute_seconds":max(times),"mean_compute_seconds":float(np.mean(times)),
            "mean_effective_samples":float(np.mean(effective)) if effective else None}
    return metrics,(["time","height","vertical_velocity","acceleration_from_hover"],trace)

def imitation(cfg):
    """Compare BC and DAgger with a small restricted polynomial student."""
    rng=np.random.default_rng(cfg["seed"]); dt=cfg["dt"]
    def expert(x): return float(np.clip(-2*x[0]-1.5*x[1],-1.5,1.5))
    def features(x): return np.array([1,x[0],x[1],x[0]**3,x[1]**3])
    def policy(w,x): return float(np.clip(features(x)@w,-1.5,1.5))
    def dynamics(x,u): return np.array([x[0]+dt*x[1]+0.5*dt*dt*u,x[1]+dt*u])
    def episode(x,w=None):
        states=[]; labels=[]; cost=0.
        for _ in range(cfg["steps"]):
            states.append(x.copy()); labels.append(expert(x))
            u=expert(x) if w is None else policy(w,x)
            cost+=float(x@x+0.1*u*u); x=dynamics(x,u)
        return np.array(states),np.array(labels),cost/cfg["steps"]
    def fit(X,y):
        F=np.array([features(x) for x in X])
        return np.linalg.solve(F.T@F+cfg["ridge"]*np.eye(F.shape[1]),F.T@y)
    # Split episodes, never adjacent rows of the same episode.
    train=[episode(rng.uniform(-0.3,0.3,2)) for _ in range(cfg["episodes"])]
    X=np.concatenate([a[0] for a in train]); y=np.concatenate([a[1] for a in train])
    validation=[episode(rng.uniform(-0.3,0.3,2)) for _ in range(8)]
    heldX=np.concatenate([a[0] for a in validation]); heldY=np.concatenate([a[1] for a in validation])
    eval_rng=np.random.default_rng(cfg["seed"]+10000)
    starts=eval_rng.uniform(-1.5,1.5,(32,2))
    results=[]; labels=len(y)
    w=fit(X,y)
    for i in range(cfg["rounds"]+1):
        results.append({"round":i,"expert_labels":labels,
            "expert_validation_mse":float(np.mean([(policy(w,x)-a)**2 for x,a in zip(heldX,heldY)])),
            "rollout_cost":float(np.mean([episode(x.copy(),w)[2] for x in starts]))})
        if i==cfg["rounds"]: break
        new=[episode(rng.uniform(-1.5,1.5,2),w) for _ in range(cfg["episodes"])]
        X=np.r_[X,np.concatenate([a[0] for a in new])]; y=np.r_[y,np.concatenate([a[1] for a in new])]
        labels=len(y); w=fit(X,y)
    return {"expert_rollout_cost":float(np.mean([episode(x.copy())[2] for x in starts])),
            "iterations":results,"scope":"Restricted polynomial student, simulation expert, fixed evaluation episodes."},(
            ["round","expert_labels","expert_validation_mse","rollout_cost"],
            [[r[k] for k in ("round","expert_labels","expert_validation_mse","rollout_cost")] for r in results])

def reinforcement(cfg):
    """Tabular Q-learning on a finite-state position-regulation MDP."""
    rng=np.random.default_rng(cfg["seed"]); n=41; center=20
    actions=np.array([-1,0,1]); Q=np.zeros((n,3))
    def transition(s,a):
        ns=int(np.clip(s+actions[a],0,n-1))
        reward=-((ns-center)/20)**2-0.01*float(actions[a]**2)
        return ns,reward,ns==center
    history=[]
    for episode in range(cfg["episodes"]):
        s=int(rng.integers(n))
        if s==center: continue
        total=0.
        for step in range(cfg["steps"]):
            a=int(rng.integers(3)) if rng.random()<cfg["epsilon"] else int(np.argmax(Q[s]))
            ns,r,terminal=transition(s,a)
            target=r+(0 if terminal else cfg["gamma"]*Q[ns].max())
            Q[s,a]+=cfg["alpha"]*(target-Q[s,a])
            s=ns; total+=r
            if terminal: break
            # A training cutoff does not turn the underlying state into a terminal state.
        history.append([episode,total,step+1])
    def evaluate(kind):
        successes=0; costs=[]
        erng=np.random.default_rng(cfg["seed"]+999)
        for start in [s for s in range(n) if s!=center]:
            s=start; total=0.
            for _ in range(cfg["steps"]):
                if kind=="learned": a=int(np.argmax(Q[s]))
                elif kind=="baseline": a=0 if s>center else 2
                else: a=int(erng.integers(3))
                s,r,done=transition(s,a); total-=r
                if done: successes+=1; break
            costs.append(total)
        return {"success_rate":successes/(n-1),"mean_cost":float(np.mean(costs))}
    return {k:evaluate(k) for k in ("learned","baseline","random")},(["episode","return","steps"],history)
