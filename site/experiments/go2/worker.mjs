import loadMujoco from '@mujoco/mujoco';
import wasm from '@mujoco/mujoco/mujoco.wasm';
import { Experiment, DT, FRAME_STEPS } from './engine.mjs';
import { loadAssets, describeScene, manifest } from './assets.mjs';
let experiment, playing = false, timer, deadline = 0, run = 0, sequence = 0, initializing = false;
const send = (type, body = {}) => postMessage({ type, run, ...body });
function pause() { playing = false; clearTimeout(timer); }
function publish(rows = []) { send('frame', { ...experiment.frame(), rows, playing, sequence }); }
function tick() {
  if (!playing) return;
  try {
    const start = performance.now();
    const rows = Array.from({ length: FRAME_STEPS }, () => experiment.step());
    publish(rows);
    const duration = performance.now() - start;
    send('speed', { ratio: Math.min(1, FRAME_STEPS * DT * 1000 / Math.max(duration, 0.01)) });
    deadline += FRAME_STEPS * DT * 1000;
    // Bound catch-up: a slow or background tab never accumulates unbounded physics work.
    if (deadline < performance.now() - 40) deadline = performance.now();
    timer = setTimeout(tick, Math.max(0, deadline - performance.now()));
  } catch (error) { pause(); send('error', { message: String(error.message || error) }); }
}
onmessage = async ({ data: message }) => {
  try {
    if (message.type === 'init') {
      if (initializing || experiment) return;
      initializing = true;
      send('progress', { message: 'Starting MuJoCo 4…' });
      const mj = await loadMujoco({ locateFile: path => path.endsWith('.wasm') ? new URL(wasm, import.meta.url).href : path });
      await loadAssets(mj, (done, total) => send('progress', { message: `Loading Go2 assets: ${done}/${total}` }));
      send('progress', { message: 'Compiling the Go2 model…' });
      experiment = new Experiment(mj, mj.MjModel.from_xml_path('/go2/' + manifest.entry));
      experiment.configure(message.settings);
      experiment.reset();
      send('ready', { scene: describeScene(experiment.model), joints: experiment.joints,
        nq: experiment.model.nq, nv: experiment.model.nv, nu: experiment.model.nu,
        version: mj.mj_versionString(), revision: manifest.revision });
      publish();
      return;
    }
    if (!experiment) return;
    if (message.type === 'play') { if (!playing) { playing = true; deadline = performance.now(); tick(); } }
    if (message.type === 'pause') { pause(); publish(); }
    if (message.type === 'step') { pause(); publish([experiment.step()]); }
    if (message.type === 'reset') {
      pause(); run++; sequence = 0;
      experiment.configure(message.settings); experiment.reset();
      send('reset', { settings: experiment.settings }); publish();
    }
    if (message.type === 'configure') {
      experiment.configure(message.settings); sequence++;
      send('event', { event: { time: experiment.data.time, kind: 'settings', settings: experiment.settings, sequence } });
    }
    if (message.type === 'pulse' && experiment.pulse()) send('event', { event: experiment.events.at(-1) });
  } catch (error) { pause(); send('error', { message: String(error.message || error) }); }
};
