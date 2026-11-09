import './style.css';
import { RobotView } from './renderer.mjs';
import { Plots } from './plots.mjs';
import { History, DEFAULTS, DT, HISTORY_SECONDS } from './engine.mjs';
import { manifest } from './assets.mjs';

const $ = id => document.getElementById(id);
const controlLesson = new URLSearchParams(location.search).get('lesson') === 'control';
let worker, view, plots, joints = [], loaded = false, playing = false, currentRun = 0;
let history = new History(), previous = null, previousMeta = null, events = [], settingsAtReset;
let metadata, lastPlot = 0, selectedJoint = DEFAULTS.joint;
let observedStart = null;
let settings = { ...DEFAULTS, mode: controlLesson ? 'pd' : 'passive' };
$('controller-panel').hidden = !controlLesson;
if (!controlLesson) document.querySelectorAll('[data-control-exercise]').forEach(el => el.hidden = true);
$('lesson-description').textContent = controlLesson
  ? 'Tune joint PD gains and inspect tracking, torque limits, and contact. Joint regulation alone does not guarantee balance.'
  : 'Inspect a floating robot, its motion and contact with the ground. The motors are set to zero torque; passive joint damping remains active.';
$('lesson-title').textContent = controlLesson ? 'Go2 · joint feedback' : 'Go2 · models and motion';
$('lesson-link').href = controlLesson ? '?lesson=models' : '?lesson=control';
$('lesson-link').textContent = controlLesson ? 'Open the modeling experiment' : 'Open the feedback experiment';
$('model-source').href = `https://github.com/google-deepmind/mujoco_menagerie/tree/${manifest.revision}/unitree_go2`;
$('model-license').href = manifest.licenseUrl;
$('download-size').textContent = `${(manifest.files.reduce((sum, f) => sum + f.bytes, 0) / 1e6).toFixed(1)} MB of robot assets, downloaded once and cached when storage is available.`;
$('kp').value = settings.kp; $('kd').value = settings.kd; $('lift').value = settings.lift;
function post(type, extra = {}) { worker?.postMessage({ type, ...extra }); }
function status(text, error = false) { $('status').textContent = text; $('status').classList.toggle('error', error); }
function setPlaying(value) {
  playing = value; $('play').textContent = value ? 'Pause' : 'Play'; $('step').disabled = value;
  if (!value) { observedStart = null; }
}
function syncLabels() {
  for (const id of ['kp', 'kd', 'offset', 'lift']) $(id + '-value').textContent = Number($(id).value).toFixed(id === 'kp' ? 0 : 2);
}
syncLabels();
function refreshPlots(force = false) {
  if (!plots || !joints.length || (!force && performance.now() - lastPlot < 50)) return;
  plots.update(history.values(), previous, selectedJoint, joints[selectedJoint].name); lastPlot = performance.now();
}
function resetRun() {
  if (!loaded) return;
  settings.lift = Number($('lift').value);
  settingsAtReset = { ...settings };
  post('reset', { settings });
}
function recordEvent(event) { events.push(event); }
function enableControls(enabled) {
  document.querySelectorAll('[data-needs-model]').forEach(el => el.disabled = !enabled);
}
async function load() {
  $('load').disabled = true; settings.lift = Number($('lift').value); status('Preparing the experiment…');
  try {
    view?.dispose(); plots?.dispose(); worker?.terminate();
    view = new RobotView($('viewport'));
    plots = new Plots($('plots'));
    worker = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' });
    worker.onerror = event => fail(event.message || 'Could not start the simulation worker.');
    worker.onmessage = ({ data: m }) => {
      try {
      if (m.type === 'progress') status(m.message);
      if (m.type === 'error') fail(m.message);
      if (m.type === 'ready') {
        metadata = m; joints = m.joints; loaded = true; settingsAtReset = { ...settings };
        view.load(m.scene); enableControls(true);
        $('joint').replaceChildren(...joints.map((j, i) => new Option(j.name.replace('_joint', ''), i)));
        $('joint').value = selectedJoint;
        $('model-stats').textContent = `MuJoCo ${m.version} · nq ${m.nq} · nv ${m.nv} · nu ${m.nu} · floating base`;
        $('load').hidden = true; $('intro-load').hidden = true;
        view.collisionOnly($('collision').checked);
        status('Ready. Inspect the robot, then press Play.'); refreshPlots(true);
      }
      if (m.type === 'reset') {
        currentRun = m.run; history.clear(); events = [];
        $('save-run').disabled = true; $('export').disabled = true;
        $('speed').textContent = '500 Hz physics · 50 Hz state updates';
        settingsAtReset = { ...m.settings }; observedStart = null;
        status(previous ? 'Reset to the initial pose. Run A is retained for comparison.' : 'Reset to the initial pose. Press Play to start a new run.'); refreshPlots(true);
      }
      if (m.type === 'event' && m.run === currentRun) recordEvent(m.event);
      if (m.type === 'frame' && m.run === currentRun) {
        for (const row of m.rows) history.push(row);
        view.update(m); setPlaying(m.playing);
        $('time').textContent = `${m.time.toFixed(3)} s`;
        $('contacts').textContent = `${m.ncon} detected contacts`;
        if (m.playing) {
          const now = performance.now();
          if (!observedStart) observedStart = { wall: now, sim: m.time };
          if (now - observedStart.wall >= 500) {
            const observedRatio = (m.time - observedStart.sim) / ((now - observedStart.wall) / 1000);
            $('speed').textContent = `${observedRatio.toFixed(2)}× real time`; observedStart = { wall: now, sim: m.time };
          }
        }
        if (history.length) { $('save-run').disabled = false; $('export').disabled = false; }
        refreshPlots();
        if (!m.playing) refreshPlots(true);
      }
      } catch (error) { fail(error.message || String(error)); }
    };
    post('init', { settings });
  } catch (error) { fail(error.message); }
}
function fail(message) {
  status(message + ' You can reload the model to retry.', true);
  loaded = false; setPlaying(false); enableControls(false);
  for (const id of ['save-run', 'export', 'clear-run']) $(id).disabled = true;
  $('comparison').textContent = '';
  $('collision').checked = false;
  $('load').hidden = false; $('load').disabled = false; $('load').textContent = 'Reload model';
  worker?.terminate(); currentRun = 0; history.clear(); events = []; previous = null; previousMeta = null;
}
$('load').addEventListener('click', load);
$('play').addEventListener('click', () => { if (playing) post('pause'); else { status('Running. Time and plots use simulation seconds.'); post('play'); } });
$('step').addEventListener('click', () => post('step'));
$('reset').addEventListener('click', resetRun);
$('home-camera').addEventListener('click', () => view?.home());
$('collision').addEventListener('change', e => view?.collisionOnly(e.target.checked));
$('push').addEventListener('click', () => { post('pulse'); status('30 N push queued: +y at the base COM for the next 0.10 simulation seconds.'); });
$('joint').addEventListener('change', e => {
  selectedJoint = Number(e.target.value);
  // Selecting a plot does not move the target to another joint. Target has a separate control.
  refreshPlots(true); updateLimit();
});
function updateLimit() {
  const j = joints[selectedJoint];
  if (j) $('torque-limit').textContent = `${j.name.replace('_joint', '')}: motor torque range ${j.low.toFixed(2)} to ${j.high.toFixed(2)} N m.`;
}
$('target-joint').addEventListener('change', e => { settings.joint = Number(e.target.value); post('configure', { settings }); });
for (const id of ['kp', 'kd', 'offset']) $(id).addEventListener('input', () => {
  settings[id] = Number($(id).value); syncLabels(); if (loaded) post('configure', { settings });
});
$('lift').addEventListener('input', syncLabels);
$('mode').value = settings.mode;
$('mode').addEventListener('change', e => { settings.mode = e.target.value; if (loaded) post('configure', { settings }); });
$('save-run').addEventListener('click', () => {
  previous = history.values().map(row => [...row]);
  previousMeta = { settingsAtReset, events: structuredClone(events), interval: [previous[0][0], previous.at(-1)[0]] };
  $('comparison').textContent = `Run A saved: ${previous[0][0].toFixed(2)}–${previous.at(-1)[0].toFixed(2)} s. Reset, change one parameter, and run again.`;
  $('clear-run').disabled = false; refreshPlots(true);
});
$('clear-run').addEventListener('click', () => { previous = null; previousMeta = null; $('comparison').textContent = ''; $('clear-run').disabled = true; refreshPlots(true); });
function download(name, content, type) {
  const url = URL.createObjectURL(new Blob([content], { type }));
  const a = document.createElement('a'); a.href = url; a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
}
$('export').addEventListener('click', () => {
  const rows = history.values();
  if (!rows.length) return;
  const columns = ['time_s', 'base_origin_height_m', 'contact_count', 'contact_normal_magnitude_sum_N', 'base_push_world_y_N'];
  for (const j of joints) for (const suffix of ['position_rad', 'velocity_rad_s', 'requested_torque_Nm', 'applied_torque_Nm', 'reference_rad']) columns.push(j.name + '_' + suffix);
  download('go2-current.csv', [columns.join(','), ...rows.map(row => row.map(v => Number.isFinite(v) ? v.toPrecision(12) : '').join(','))].join('\n'), 'text/csv');
});
$('export-settings').addEventListener('click', () => download('go2-experiment.json', JSON.stringify({
  schema: 1, model: manifest.id, revision: manifest.revision, mujoco: metadata?.version, dt: DT,
  integrator: 'implicitfast', settingsAtReset, events, currentSettings: settings,
  recordedInterval: history.length ? [history.values()[0][0], history.values().at(-1)[0]] : null,
  historySeconds: HISTORY_SECONDS, samplePhase: 'before integration; after mj_forward', previousRun: previousMeta,
}, null, 2), 'application/json'));
// Initialization of named target controls uses the same model-derived mapping as the worker.
new MutationObserver(() => {
  $('target-joint').replaceChildren(...[...$('joint').options].map(o => new Option(o.text, o.value)));
  $('target-joint').value = settings.joint; updateLimit();
}).observe($('joint'), { childList: true });
document.addEventListener('visibilitychange', () => { if (document.hidden && loaded) { post('pause'); status('Paused while this page was hidden. Press Play to resume.'); } });
window.addEventListener('pagehide', () => { worker?.terminate(); view?.dispose(); plots?.dispose(); });
window.addEventListener('pageshow', e => { if (e.persisted) location.reload(); });
