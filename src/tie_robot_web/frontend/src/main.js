import { TieRobotFrontApp } from "./app/TieRobotFrontApp.js";
import "./styles/app.css";

const appRoot = document.getElementById("app");
const app = new TieRobotFrontApp(appRoot);
window.tieRobotFrontApp = app;
app.init();
