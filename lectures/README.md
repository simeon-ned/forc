# Lecture decks (Reveal.js)

## Template

Copy `_template/` to a new folder:

```bash
cp -r lectures/_template lectures/02-classical
```

Edit `index.html`, keep `css/` + `js/config.js`. Serve locally:

```bash
cd lectures/01-intro && python3 -m http.server 8081
```

Built site passthrough: `/lectures/<slug>/`.
