const fs = require("fs");
const path = require("path");
const { pathToFileURL } = require("url");
const { spawn } = require("child_process");

function findBrowserExecutable() {
  const candidates = [
    "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe",
    "C:\\Program Files (x86)\\Google\\Chrome\\Application\\chrome.exe",
    "C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe",
    "C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe",
  ];
  return candidates.find((candidate) => fs.existsSync(candidate));
}

function render(root, htmlName, pngName) {
  const htmlPath = path.join(root, "docs", "assets", "quickstart", htmlName);
  const pngPath = path.join(root, "docs", "assets", "quickstart", pngName);
  const browserPath = findBrowserExecutable();

  if (!browserPath) {
    throw new Error("No Chrome or Edge executable found.");
  }

  return new Promise((resolve, reject) => {
    const child = spawn(browserPath, [
      "--headless=new",
      "--disable-gpu",
      "--hide-scrollbars",
      "--window-size=2400,1350",
      `--screenshot=${pngPath}`,
      pathToFileURL(htmlPath).href,
    ], { stdio: "inherit" });

    child.on("error", reject);
    child.on("exit", (code) => {
      if (code === 0) {
        console.log(pngPath);
        resolve();
      } else {
        reject(new Error(`Browser exited with code ${code}`));
      }
    });
  });
}

async function main() {
  const root = path.resolve(__dirname, "..");
  await render(root, "evo-control-system-main-flow.html", "evo-control-system-main-flow.png");
  await render(root, "evo-control-system-diagnostics.html", "evo-control-system-diagnostics.png");
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
