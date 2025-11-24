// Configuration du radar
let config = {
  maxDistance: 12,
  updateSpeed: 200,
  persistence: 500,
  pointSize: 2,
  minDist: 50,
  maxDist: 12000,
};

let radarData = null;
let lastDataTime = 0;
let updateInterval;

// Éléments du DOM
const canvas = document.getElementById("radar");
const ctx = canvas.getContext("2d");
const centerX = canvas.width / 2;
const centerY = canvas.height / 2;
const maxRadius = Math.min(centerX, centerY) - 40;

// Fonction pour mettre à jour les contrôles
function updateControls() {
  document.getElementById("maxDistValue").textContent =
    config.maxDistance + "m";
  document.getElementById("speedValue").textContent = config.updateSpeed + "ms";
  document.getElementById("persistValue").textContent =
    config.persistence + "ms";
  document.getElementById("sizeValue").textContent = config.pointSize + "px";
}

// Fonction pour dessiner les règles du radar
function drawRules() {
  ctx.strokeStyle = "#444";
  ctx.lineWidth = 1;
  ctx.fillStyle = "#666";
  ctx.font = "12px Arial";

  // Cercles de distance
  for (let i = 1; i <= 6; i++) {
    const radius = (maxRadius * i) / 6;
    const distance = (config.maxDistance * i) / 6;
    ctx.beginPath();
    ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText(distance.toFixed(1) + "m", centerX + radius - 20, centerY - 5);
  }

  // Lignes d'angle
  for (let i = 0; i < 8; i++) {
    const angle = (i * Math.PI) / 4;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(
      centerX + Math.cos(angle) * maxRadius,
      centerY + Math.sin(angle) * maxRadius
    );
    ctx.stroke();
  }

  // Axe Nord
  ctx.strokeStyle = "#00ffff";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(centerX, centerY);
  ctx.lineTo(centerX, centerY - maxRadius + 20);
  ctx.stroke();
  ctx.fillStyle = "#00ffff";
  ctx.font = "bold 16px Arial";
  ctx.fillText("N", centerX - 8, centerY - maxRadius - 10);
}

// Fonction principale de dessin du radar
function drawRadar() {
  const scale = maxRadius / (config.maxDistance * 1000);

  // Effacer le canvas
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Dessiner les règles
  drawRules();

  if (radarData && radarData.length === 360) {
    const now = Date.now();
    ctx.fillStyle = "#00ff00";
    let validPoints = 0;

    for (let i = 0; i < 360; i++) {
      if (radarData[i] > config.minDist && radarData[i] <= config.maxDist) {
        const angle = (i * Math.PI) / 180;
        const distance =
          (radarData[i] / (config.maxDistance * 1000)) * maxRadius;

        if (distance <= maxRadius) {
          const x = centerX + Math.cos(angle) * distance;
          const y = centerY + Math.sin(angle) * distance;

          ctx.beginPath();
          ctx.arc(x, y, config.pointSize, 0, 2 * Math.PI);
          ctx.fill();
          validPoints++;
        }
      }
    }

    document.getElementById("radarInfo").innerHTML =
      "Points: " +
      validPoints +
      " | Distance max: " +
      config.maxDistance +
      "m | FPS: " +
      Math.round(1000 / config.updateSpeed);
  } else {
    document.getElementById("radarInfo").innerHTML =
      "En attente des données LiDAR...";
  }
}

// Fonction de mise à jour du radar
function updateRadar() {
  fetch("/lidar360")
    .then((r) => {
      if (r.status === 503) {
        console.log("Serveur occupé, retry...");
        return;
      }
      return r.json();
    })
    .then((d) => {
      if (d && d.connected && d.data) {
        radarData = d.data;
        lastDataTime = Date.now();
        document.getElementById("rpmDisplay").innerHTML = "(" + d.rpm + " RPM)";
      } else if (d && d.error) {
        console.log("Erreur serveur:", d.error);
        return;
      } else {
        if (Date.now() - lastDataTime > config.persistence) {
          radarData = null;
          document.getElementById("rpmDisplay").innerHTML = "";
        }
      }
      drawRadar();
    })
    .catch((e) => {
      console.log("Erreur fetch:", e.message);
      drawRadar();
    });
}

// Fonction pour appliquer la configuration
function applyConfig() {
  config.maxDistance = parseFloat(document.getElementById("maxDistance").value);
  config.updateSpeed = parseInt(document.getElementById("updateSpeed").value);
  config.persistence = parseInt(document.getElementById("persistence").value);
  config.pointSize = parseInt(document.getElementById("pointSize").value);
  config.minDist = parseInt(document.getElementById("minDist").value);
  config.maxDist = parseInt(document.getElementById("maxDist").value);

  clearInterval(updateInterval);
  updateInterval = setInterval(updateRadar, config.updateSpeed);
  updateControls();
}

// Fonction pour mettre à jour le statut
function updateStatus() {
  fetch("/data")
    .then((r) => r.json())
    .then((data) => {
      document.getElementById("lidarStatus").textContent = data.connected
        ? "Connecté"
        : "Déconnecté";
      document.getElementById("distanceValue").textContent =
        data.distance + " mm";
      document.getElementById("pointsValue").textContent = data.points || "0";
      document.getElementById("maxDistanceValue").textContent =
        data.maxDistance + " mm";
    })
    .catch((e) => console.log("Erreur statut:", e));
}

// Event listeners
document.getElementById("maxDistance").addEventListener("input", applyConfig);
document.getElementById("updateSpeed").addEventListener("input", applyConfig);
document.getElementById("persistence").addEventListener("input", applyConfig);
document.getElementById("pointSize").addEventListener("input", applyConfig);
document.getElementById("minDist").addEventListener("input", applyConfig);
document.getElementById("maxDist").addEventListener("input", applyConfig);

// Gestion des commandes
let currentCommand = 0;
const commandNames = ["AVANCE", "RECUL", "GAUCHE", "DROITE"];

function sendCommand(cmd) {
  fetch(`/command?cmd=${cmd}`)
    .then((response) => response.json())
    .then((data) => {
      if (data.status === "ok") {
        currentCommand = data.commande;
        updateCommandDisplay();
        updateButtonStates();
        console.log("Commande envoyée:", commandNames[currentCommand]);
      } else {
        console.error("Erreur commande:", data.error);
      }
    })
    .catch((error) => {
      console.error("Erreur envoi commande:", error);
    });
}

function updateCommandDisplay() {
  const display = document.getElementById("current-command");
  if (display) {
    display.textContent = `${currentCommand} (${commandNames[currentCommand]})`;
  }
}

function updateButtonStates() {
  document.querySelectorAll(".cmd-btn").forEach((btn) => {
    btn.classList.remove("active");
    if (parseInt(btn.dataset.cmd) === currentCommand) {
      btn.classList.add("active");
    }
  });
}

// Initialisation
drawRadar();
updateControls();
updateInterval = setInterval(updateRadar, config.updateSpeed);
setTimeout(updateRadar, 500);

// Gestion des boutons de commande
document.querySelectorAll(".cmd-btn").forEach((btn) => {
  btn.addEventListener("click", function () {
    const cmd = parseInt(this.dataset.cmd);
    sendCommand(cmd);
  });
});

// Initialisation de l'affichage des commandes
updateCommandDisplay();
updateButtonStates();

// Mise à jour du statut toutes les 2 secondes
setInterval(updateStatus, 2000);
