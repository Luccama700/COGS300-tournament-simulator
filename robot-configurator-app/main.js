const { app, BrowserWindow, ipcMain, dialog } = require("electron");
const path = require("path");
const fs = require("fs");

const CONFIG_PATH = path.join(app.getPath("userData"), "robot-config.json");

function createWindow() {
  const win = new BrowserWindow({
    width: 1060,
    height: 740,
    minWidth: 800,
    minHeight: 600,
    titleBarStyle: "hiddenInset",
    trafficLightPosition: { x: 16, y: 16 },
    backgroundColor: "#0d0d0d",
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  win.loadFile(path.join(__dirname, "src", "index.html"));
}

// Config persistence via IPC
ipcMain.handle("config:load", async () => {
  try {
    if (fs.existsSync(CONFIG_PATH)) {
      return JSON.parse(fs.readFileSync(CONFIG_PATH, "utf-8"));
    }
  } catch (e) {}
  return null;
});

ipcMain.handle("config:save", async (_event, config) => {
  try {
    fs.writeFileSync(CONFIG_PATH, JSON.stringify(config, null, 2));
    return true;
  } catch (e) {
    return false;
  }
});

ipcMain.handle("config:export", async (_event, content, defaultName) => {
  const { canceled, filePath } = await dialog.showSaveDialog({
    defaultPath: defaultName,
    filters: [
      { name: "YAML", extensions: ["yaml", "yml"] },
      { name: "JSON", extensions: ["json"] },
      { name: "All Files", extensions: ["*"] },
    ],
  });
  if (canceled || !filePath) return false;
  fs.writeFileSync(filePath, content);
  return filePath;
});

app.whenReady().then(createWindow);

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});

app.on("activate", () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
