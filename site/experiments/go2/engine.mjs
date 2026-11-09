// The physics/controller contract is shared by the browser worker and numerical tests.
export const DT = 0.002;
export const FRAME_STEPS = 10;
export const HISTORY_SECONDS = 15;
export const DEFAULTS = Object.freeze({ mode: 'passive', kp: 35, kd: 2, offset: 0, joint: 1, lift: 0.12 });
export const clamp = (x, low, high) => Math.max(low, Math.min(high, x));

export class Experiment {
  constructor(mj, model) {
    this.mj = mj;
    this.model = model;
    this.data = new mj.MjData(model);
    model.opt.timestep = DT;
    model.opt.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST.value;
    // Surface numerical failures instead of silently resetting to a plausible-looking pose.
    model.opt.disableflags |= mj.mjtDisableBit.mjDSBL_AUTORESET.value;
    this.base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, 'base');
    if (this.base < 0) throw new Error('The experiment needs a named floating base.');
    this.home = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY.value, 'home');
    if (this.home < 0) throw new Error('The experiment needs the home keyframe.');
    this.force = new mj.DoubleBuffer(6);
    this.joints = Array.from({ length: model.nu }, (_, i) => {
      const joint = model.actuator_trnid[2 * i];
      if (model.actuator_trntype[i] !== mj.mjtTrn.mjTRN_JOINT.value ||
          model.jnt_type[joint] !== mj.mjtJoint.mjJNT_HINGE.value ||
          model.actuator_dyntype[i] !== mj.mjtDyn.mjDYN_NONE.value ||
          model.actuator_gaintype[i] !== mj.mjtGain.mjGAIN_FIXED.value ||
          model.actuator_gainprm[10 * i] !== 1 ||
          model.actuator_biastype[i] !== mj.mjtBias.mjBIAS_NONE.value ||
          model.actuator_gear[6 * i] !== 1 || !model.actuator_ctrllimited[i]) {
        throw new Error('This lesson requires limited, unit-gear hinge torque motors.');
      }
      return {
        name: mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT.value, joint),
        q: model.jnt_qposadr[joint], v: model.jnt_dofadr[joint],
        low: model.actuator_ctrlrange[2 * i], high: model.actuator_ctrlrange[2 * i + 1],
        qlow: model.jnt_range[2 * joint], qhigh: model.jnt_range[2 * joint + 1],
      };
    });
    this.settings = { ...DEFAULTS };
    this.requested = new Float64Array(model.nu);
    this.targets = new Float64Array(model.nu);
    this.reset();
  }
  configure(settings) {
    const s = { ...this.settings, ...settings };
    if (!['passive', 'pd'].includes(s.mode)) throw new Error('Unknown controller');
    for (const [name, low, high] of [['kp', 0, 100], ['kd', 0, 8], ['offset', -0.3, 0.3], ['lift', 0, 0.5], ['joint', 0, this.model.nu - 1]]) {
      if (!Number.isFinite(s[name]) || s[name] < low || s[name] > high) throw new Error(`Invalid ${name}`);
    }
    if (!Number.isInteger(s.joint)) throw new Error('Joint index must be an integer');
    this.settings = s;
  }
  reset() {
    this.mj.mj_resetDataKeyframe(this.model, this.data, this.home);
    this.data.qpos[2] += this.settings.lift;
    this.data.ctrl.fill(0); // Menagerie keyframe ctrl contains pose values, not our torque command.
    this.nominal = this.joints.map(j => this.data.qpos[j.q]);
    this.steps = 0;
    this.pulseRemaining = 0;
    this.events = [];
    this.mj.mj_forward(this.model, this.data);
  }
  pulse() {
    // A reproducible 30 N world +y force at the base COM for 0.1 simulated seconds.
    if (this.pulseRemaining) return false;
    this.pulseRemaining = 50;
    this.events.push({ time: this.data.time, kind: 'force', body: 'base', frame: 'world', force: [0, 30, 0], duration: 0.1 });
    return true;
  }
  step() {
    const { data: d, model: m, mj, settings: s } = this;
    for (let i = 0; i < this.joints.length; i++) {
      const j = this.joints[i];
      this.targets[i] = clamp(this.nominal[i] + (i === s.joint ? s.offset : 0), j.qlow, j.qhigh);
      this.requested[i] = s.mode === 'pd' ? s.kp * (this.targets[i] - d.qpos[j.q]) - s.kd * d.qvel[j.v] : 0;
      d.ctrl[i] = clamp(this.requested[i], j.low, j.high);
    }
    d.xfrc_applied.fill(0);
    if (this.pulseRemaining > 0) d.xfrc_applied[6 * this.base + 1] = 30;
    // Sample state, input and contact from ONE consistent pre-integration instant.
    mj.mj_forward(m, d);
    let normalForce = 0;
    for (let i = 0; i < d.ncon; i++) {
      mj.mj_contactForce(m, d, i, this.force);
      normalForce += this.force.GetView()[0];
    }
    const row = [d.time, d.xpos[3 * this.base + 2], d.ncon, normalForce, d.xfrc_applied[6 * this.base + 1]];
    for (let i = 0; i < this.joints.length; i++) {
      const j = this.joints[i];
      row.push(d.qpos[j.q], d.qvel[j.v], this.requested[i], d.qfrc_actuator[j.v], s.mode === 'pd' ? this.targets[i] : NaN);
    }
    if (!row.slice(0, 4).every(Number.isFinite)) throw new Error('Non-finite dynamics: reset the experiment.');
    mj.mj_step(m, d);
    if (!Array.from(d.qpos).every(Number.isFinite) || !Array.from(d.qvel).every(v => Number.isFinite(v) && Math.abs(v) < 1e5)) {
      throw new Error('Simulation diverged. Reduce gains or reset the experiment.');
    }
    this.steps++;
    this.pulseRemaining = Math.max(0, this.pulseRemaining - 1);
    return row;
  }
  frame() {
    this.mj.mj_forward(this.model, this.data); // Refresh poses after the last integration.
    return { time: this.data.time, steps: this.steps, positions: this.data.geom_xpos.slice(), rotations: this.data.geom_xmat.slice(), ncon: this.data.ncon };
  }
  dispose() { this.force.delete(); this.data.delete(); this.model.delete(); }
}

export class History {
  constructor(capacity = Math.round(HISTORY_SECONDS / DT)) { this.capacity = capacity; this.clear(); }
  clear() { this.rows = new Array(this.capacity); this.head = 0; this.length = 0; }
  push(row) { this.rows[this.head] = row; this.head = (this.head + 1) % this.capacity; this.length = Math.min(this.length + 1, this.capacity); }
  values() { return Array.from({ length: this.length }, (_, i) => this.rows[(this.head - this.length + i + this.capacity) % this.capacity]); }
}
