import { TieRobotFrontApp } from "./app/TieRobotFrontApp.js";
import { loadFrontendSharedState } from "./utils/storage.js";
import "./styles/app.css";

const appRoot = document.getElementById("app");

async function bootstrapTieRobotFrontApp() {
  await loadFrontendSharedState();
  const app = new TieRobotFrontApp(appRoot);
  window.tieRobotFrontApp = app;
  app.init();
}

bootstrapTieRobotFrontApp();
