const path = require('node:path');
const fs = require('node:fs/promises');
const esbuild = require('esbuild');
const root = path.resolve(__dirname, '..');
module.exports = function(config) {
  config.addWatchTarget('experiments/');
  config.watchIgnores.add('**/.lab-build/**');
  config.on('eleventy.before', async () => {
    await esbuild.build({
      absWorkingDir: root,
      entryPoints: { main: 'experiments/go2/main.mjs', worker: 'experiments/go2/worker.mjs' },
      outdir: '.lab-build', bundle: true, format: 'esm', splitting: true, platform: 'browser',
      external: ['module'], // Emscripten's Node-only branch; never executed by the browser.
      target: ['es2022'], loader: { '.wasm': 'file' }, assetNames: 'assets/[name]-[hash]',
      minify: true, sourcemap: true, logLevel: 'warning',
    });
    await fs.copyFile(path.join(root, 'experiments/go2/NOTICE.txt'), path.join(root, '.lab-build/NOTICE.txt'));
    await fs.cp(path.join(root, 'experiments/go2/licenses'), path.join(root, '.lab-build/licenses'), { recursive: true });
  });
  config.addPassthroughCopy({ '.lab-build': 'demos/lab' });
};
