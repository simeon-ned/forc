# Fundamentals of Robot Control (FORC)

Introductory course on classical, optimal, and learning-based control for robots, with MuJoCo practices.

**Site (docs):** build from [`site/`](site/) · **Slides:** [`lectures/`](lectures/) · **Labs:** [`practices/`](practices/) · **Python:** [`src/forc/`](src/forc/)

## Layout

```text
forc/
├── site/           # Eleventy course docs (MuJoCo-style nav)
├── lectures/       # Reveal.js decks (_template + per-week)
├── practices/      # Guided notebooks + assets
├── src/forc/       # Shared Python helpers (pip install -e .)
├── hw/             # Take-homes (th1_mpc, th2_learning) + legacy templates
├── images/         # Figures shared with the site
└── _legacy/        # Archived Jekyll + Quarto materials
```

## Quick start (students)

```bash
git clone https://github.com/simeon-ned/forc.git
cd forc
pip install -e ".[sim]"
```

Then open notebooks under `practices/` (see `practices/README.md` for Docker / devcontainer).

## Site (local)

From the repo root (after `npm run install:site` once):

```bash
npm run install:site
FORC_PATH_PREFIX= npm run dev
```

Or from `site/`:

```bash
cd site && npm install && FORC_PATH_PREFIX= npm run dev
```

Production builds use path prefix `/forc` for GitHub Project Pages (`https://simeon-ned.github.io/forc/`).

## Lectures

```bash
cp -r lectures/_template lectures/02-classical
cd lectures/01-intro && python3 -m http.server 8081
```

## Syllabus

**12 lectures** in **6 topics**, plus weekly practice. Assessment: practices 20%, TH1 25%, TH2 25%, course project (exam) 30%. Guest talks (Sber / Yandex) are separate from the topic list.

Canonical: [`site/content/syllabus.md`](site/content/syllabus.md) · PDF: [`site/public/syllabus.pdf`](site/public/syllabus.pdf) (legacy layout; web page is source of truth).

## License

See [LICENSE](LICENSE).
