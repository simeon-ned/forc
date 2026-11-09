import { test, before, after } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import { createHash } from 'node:crypto';
import loadMujoco from '@mujoco/mujoco';
import { Experiment, History, DT, DEFAULTS } from '../go2/engine.mjs';
const manifest = JSON.parse(await fs.readFile(new URL('../go2/model.json', import.meta.url)));
let mj, exp;
before(async () => {
  mj = await loadMujoco();
  mj.FS.mkdirTree('/go2/assets');
  const cache = path.join(os.tmpdir(), 'forc-go2-model');
  const queue = [...manifest.files];
  await Promise.all(Array.from({ length: 4 }, async () => {
    while (queue.length) {
      const item = queue.shift(), local = path.join(cache, item.path);
      let bytes;
      try { bytes = await fs.readFile(local); } catch { /* Download on first test run. */ }
      const valid = b => b && b.length === item.bytes && createHash('sha256').update(b).digest('hex') === item.sha256;
      if (!valid(bytes)) {
        const response = await fetch(manifest.baseUrl + item.path, { signal: AbortSignal.timeout(60000) });
        assert.equal(response.status, 200, item.path);
        bytes = Buffer.from(await response.arrayBuffer());
        assert.ok(valid(bytes), `Checksum ${item.path}`);
        await fs.mkdir(path.dirname(local), { recursive: true }); await fs.writeFile(local, bytes);
      }
      mj.FS.writeFile('/go2/' + item.path, bytes);
    }
  }));
  exp = new Experiment(mj, mj.MjModel.from_xml_path('/go2/scene.xml'));
});
after(() => exp?.dispose());
function reset(extra = {}) { exp.configure({ ...DEFAULTS, ...extra }); exp.reset(); }

test('real Go2 preserves floating coordinates and named motor addresses', () => {
  assert.deepEqual([exp.model.nq, exp.model.nv, exp.model.nu], [19, 18, 12]);
  assert.equal(exp.joints[0].q, 7); assert.equal(exp.joints[0].v, 6);
  assert.equal(exp.joints[1].name, 'FL_thigh_joint');
  assert.equal(mj.mj_versionString(), '4.0.0');
});
test('zero-input motion advances at exactly 2 ms and develops ground contacts', () => {
  reset(); let contact = false;
  for (let i = 0; i < 500; i++) { const row = exp.step(); assert.equal(row[7], 0); contact ||= row[2] > 0; }
  assert.ok(Math.abs(exp.data.time - 1) < 1e-10); assert.ok(contact);
  assert.ok(exp.data.qpos[2] < 0.27 + DEFAULTS.lift);
});
test('PD maps targets to joint coordinates and clips actual torque', () => {
  reset({ mode: 'pd', kp: 100, kd: 0, joint: 0, offset: 0.3 });
  const row = exp.step();
  assert.ok(Math.abs(row[7] - 30) < 1e-9); // Requested first-joint torque.
  assert.ok(Math.abs(row[8] - exp.joints[0].high) < 1e-9);
  assert.equal(row[5], 0); assert.equal(row[9], 0.3);
});
test('reset clears force history and reproduces a trajectory and force pulse', () => {
  const rollout = () => {
    reset({ mode: 'pd' }); exp.pulse();
    assert.equal(exp.pulse(), false);
    return Array.from({ length: 250 }, () => exp.step());
  };
  const a = rollout(), b = rollout(); assert.deepEqual(a, b);
  assert.equal(a.filter(row => row[4] === 30).length, 50);
  assert.equal(a[50][4], 0); assert.equal(exp.events.length, 1);
  reset(); assert.equal(exp.events.length, 0); assert.equal(exp.data.time, 0);
});
test('PD contact simulation stays finite through a 15-second recorded window', () => {
  reset({ mode: 'pd' });
  const begin = performance.now();
  for (let i = 0; i < 7500; i++) exp.step();
  assert.ok(Math.abs(exp.data.time - 15) < 1e-8);
  console.log(`15 simulated seconds: ${((performance.now() - begin) / 1000).toFixed(2)} s in this Node/WASM environment (not a browser benchmark).`);
});
test('history retains bounded chronological samples without aliasing the live state', () => {
  const h = new History(3); for (let i = 0; i < 5; i++) h.push([i]);
  assert.deepEqual(h.values(), [[2], [3], [4]]); h.clear(); assert.equal(h.length, 0);
  assert.equal(DT, 0.002);
});
test('invalid controller configuration is rejected', () => {
  assert.throws(() => exp.configure({ kp: NaN })); assert.throws(() => exp.configure({ joint: 0.5 }));
});
