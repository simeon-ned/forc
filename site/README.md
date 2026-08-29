# FORC site

Eleventy documentation site for **Fundamentals of Robot Control**.

From this directory, or from the repo root via `npm run install:site` / `npm run dev`:

```bash
npm install
FORC_PATH_PREFIX= npm run dev    # http://localhost:8080
npm run build                    # → _site/ (prefix /forc)
```

- Content: `content/`
- Layout / CSS: `src/`
- Static demos: `demos/`
- Nav data: `src/_data/nav.yaml`
