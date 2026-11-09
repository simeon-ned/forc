import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';

export class Plots {
  constructor(host) {
    this.host = host;
    const definitions = [
      ['Joint position', 'Angle (rad)', ['Measured', 'Reference', 'Run A']],
      ['Joint velocity', 'Velocity (rad/s)', ['Measured', 'Run A']],
      ['Joint torque', 'Torque (N m)', ['Requested', 'Applied', 'Run A applied']],
      ['Base height', 'Height (m)', ['Base origin', 'Run A']],
      ['Contact normal forces', 'Sum of magnitudes (N)', ['All contacts', 'Run A']],
    ];
    this.plots = definitions.map(([title, unit, names]) => {
      const panel = document.createElement('section'); panel.className = 'plot-panel';
      const heading = document.createElement('h3'); heading.textContent = title;
      panel.append(heading); host.append(panel);
      const chart = new uPlot({ width: Math.max(240, panel.clientWidth), height: 205,
        cursor: { sync: { key: 'go2-time' } },
        scales: { x: { time: false } },
        series: [{ label: 'Time (s)' }, ...names.map((label, i) => ({ label, stroke: label.includes('Run A') ? '#687786' : ['#145c98', '#bf5b19'][i],
          width: 1.5, dash: label.includes('Run A') ? [5, 4] : undefined, spanGaps: false,
          value: (_, value) => value == null ? '—' : value.toFixed(3) }))],
        axes: [{ label: 'Simulation time (s)', size: 42, labelSize: 18 }, { label: unit, size: 55, labelSize: 18 }],
      }, [[], ...names.map(() => [])], panel);
      const observer = new ResizeObserver(() => chart.setSize({ width: Math.max(240, panel.clientWidth), height: 205 }));
      observer.observe(panel);
      return { chart, observer, heading, title };
    });
  }
  update(rows, previous, joint, name) {
    const start = 5 + 5 * joint;
    // Full 500 Hz samples remain available in the CSV.
    // Preserve local extrema as well as every 10th sample to avoid hiding contact/torque spikes.
    const current = new Map(rows.map(r => [Math.round(r[0] / 0.002), r]));
    const prior = new Map((previous || []).map(r => [Math.round(r[0] / 0.002), r]));
    const all = [...new Set([...current.keys(), ...prior.keys()])].sort((a, b) => a - b);
    const selected = new Set();
    const columns = [start, start + 1, start + 2, start + 3, start + 4, 1, 3];
    for (let i = 0; i < all.length; i += 10) {
      const bucket = all.slice(i, i + 10); selected.add(bucket[0]); selected.add(bucket.at(-1));
      for (const source of [current, prior]) for (const column of columns) {
        let lo = Infinity, hi = -Infinity, li, hiIndex;
        for (const k of bucket) { const v = source.get(k)?.[column]; if (v < lo) { lo = v; li = k; } if (v > hi) { hi = v; hiIndex = k; } }
        if (li !== undefined) selected.add(li); if (hiIndex !== undefined) selected.add(hiIndex);
      }
    }
    const keys = [...selected].sort((a, b) => a - b);
    const values = (map, col) => keys.map(k => { const v = map.get(k)?.[col]; return Number.isFinite(v) ? v : null; });
    const t = keys.map(k => k * 0.002);
    const data = [
      [t, values(current, start), values(current, start + 4), values(prior, start)],
      [t, values(current, start + 1), values(prior, start + 1)],
      [t, values(current, start + 2), values(current, start + 3), values(prior, start + 3)],
      [t, values(current, 1), values(prior, 1)],
      [t, values(current, 3), values(prior, 3)],
    ];
    this.plots.forEach((p, i) => {
      p.heading.textContent = i < 3 ? `${p.title} · ${name.replace('_joint', '')}` : p.title;
      p.chart.setData(data[i]);
    });
  }
  dispose() { for (const p of this.plots) { p.observer.disconnect(); p.chart.destroy(); } this.host.replaceChildren(); }
}
