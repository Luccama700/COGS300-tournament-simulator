# Robot Configurator — Desktop App

A standalone Electron app for configuring your maze robot's chassis dimensions, drive system, and sensor layout. Your config persists locally between sessions.

## Quick start (run without building)

```bash
cd robot-configurator-app
npm install
npm start
```

This opens the app immediately — no build step needed.

## Build a standalone .app / .exe

### macOS
```bash
npm run build:mac
# Output: dist/Robot Configurator.dmg
```

### Windows
```bash
npm run build:win
# Output: dist/Robot Configurator.exe (portable)
```

### Linux
```bash
npm run build:linux
# Output: dist/Robot Configurator.AppImage
```

## No Electron? Just open the HTML

The app also works as a plain browser page — no Node.js required:

```bash
open src/index.html
```

Config will persist via localStorage instead of the filesystem.

## Exporting your config

Click **Export** in the app to get YAML or JSON. Use **Save to file...** (Electron only) to write directly to disk. The YAML output plugs straight into the simulation pipeline's `configs/default.yaml`.
