# FORC site

Eleventy documentation site for **Fundamentals of Robot Control**.

```bash
npm ci
COURSE_PATH_PREFIX= npm run dev    # http://localhost:8080/
npm run build                      # → _site/ (prefix /forc)
```

Local lecture: http://localhost:8080/notes/01-modeling/

Local slides: http://localhost:8080/lectures/01-modeling/

Published URL: https://simeon-ned.github.io/forc/

- Title page: `content/index.njk` (`/`)
- Layout / CSS: `src/`
- Cite: `content/cite.md`

## Interactive Go2 experiment

The normal build and development commands also bundle the browser lab. Install
from the lockfile with `npm ci` (inside `site/`). No separate frontend dev server
or Python service is needed.

- Modeling: http://localhost:8080/experiments/go2/?lesson=models
- Joint feedback: http://localhost:8080/experiments/go2/?lesson=control
- Both are embedded in their corresponding lectures, with full-page links.

The browser runs the official `@mujoco/mujoco@4.0.0` single-threaded WASM engine
inside a dedicated Web Worker. This does not require cross-origin isolation.
Three.js renders the compiled mesh vertices and geom poses; uPlot displays
selected signals. The normal `/forc/` deployment prefix is supported. The Python
practices remain pinned to 3.13.0 pending a separate migration.

### Model assets

`experiments/go2/model.json` pins the Menagerie revision, complete file list,
byte sizes, and SHA-256 checksums. The 28.4 MB Go2 asset set is fetched from
`raw.githubusercontent.com` only after **Load Go2**. Files are cached in browser
CacheStorage when available, verified before compilation, and mounted in the
WASM filesystem. Neither meshes nor model XML are committed to the course.
First load requires network access to GitHub. Browser caching can be disabled or
evicted; it is an optimization, not a guarantee of offline availability.

To update the robot, review upstream XML/includes and all dependencies, update
all manifest checksums and sizes, and rerun both test suites. The Go2 license is
retained in `experiments/go2/NOTICE.txt`. The rendered page links to the pinned
source and license.

### Simulation and observation contract

- Fixed timestep: 0.002 s, implicitfast integrator, controller evaluated each step.
- 10 physics steps per worker publication (50 Hz at real time).
- Requested PD torques are clipped to the original motor limits. Gains apply to
  all twelve joints; the target offset affects only the selected control joint.
- Selecting a **plot** joint does not change the controller.
- Every CSV row is sampled after `mj_forward`, before integration, so states,
  applied torque and contact forces describe the same instant.
- Contact telemetry is the sum of contact-frame normal-force magnitudes, not a
  world-frame resultant or a per-foot force estimate.
- The 30 N disturbance acts in world +y at the base COM for exactly 50 steps.
- Reset restores the keyframe, applies the chosen base-height offset, clears
  keyframe control values and pending pushes, and clears the current recording.
  Run A remains available until explicitly cleared.
- Raw samples cover the most recent 15 simulation seconds. Charts update at most
  20 Hz and reduce display points while preserving local extrema. CSV retains
  every recorded sample; empty reference cells mean no reference in passive mode.
- Settings export includes version, model revision, reset configuration and
  timestamped settings/push events. Run A metadata is included when present;
  download its CSV before replacing the current run if its raw samples are needed.
- A slow browser reduces wall-clock playback speed; it does not increase the
  physics timestep. Hidden pages pause and need an explicit Play to resume.
- No arbitrary student code is evaluated. This first lab provides supplied
  controllers and parameter tuning. Joint PD is not a balance controller.

### Development and checks

```bash
npm run build
npm run check
npm run test:lab       # Real Go2 physics in the shipped WASM engine
npx playwright install chromium
npm run test:browser   # Starts the local site if it is not already running
```

To use an installed Chrome, set `CHROME_EXECUTABLE=/path/to/google-chrome` when
running `npm run test:browser`. The numerical suite downloads and checksum-checks
the pinned assets on first use and caches them under the system temporary
directory. The browser suite exercises real model loading, plots, controls,
exports, cache reuse, lecture embeds, and narrow viewports. It needs network
access for its first asset load. Neither test changes the Python environment.

Implementation boundaries:

- `experiments/go2/engine.mjs`: simulation, controller, sampling and reset contract.
- `experiments/go2/worker.mjs`: browser scheduling and message protocol.
- `experiments/go2/assets.mjs`: verified downloads and compiled mesh extraction.
- `experiments/go2/renderer.mjs`: mesh/primitive rendering and camera controls.
- `experiments/go2/plots.mjs`: chart data and display downsampling.
- `experiments/go2/main.mjs`: controls, bounded recording, comparison and exports.
- `content/go2-experiment.njk`: accessible page layout.
- `lib/experiments.cjs`: build hook; `.lab-build/` is generated and ignored.

The renderer intentionally uses simple materials, not MuJoCo's full Filament
appearance. Physics uses the original collision geometry. To add another robot,
review its asset manifest, supported geometry, joint transmissions, keyframe,
force limits, and observation mappings rather than assuming `qpos[:nu]` matches
actuated joints.
