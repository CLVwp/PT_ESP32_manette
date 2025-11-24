#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <esp_now.h>
#include <ArduinoJson.h>

// Configuration WiFi
const char* ssid = "ESP32_LiDAR";
const char* password = "12345678";
// ESP32 MAC : 8C:4F:00:28:17:20
// ESP01 MAC : 34:5F:45:5B:9B:2C

// Configuration ESP-NOW
uint8_t esp01Address[] = {0x34, 0x5F, 0x45, 0x5B, 0x9B, 0x2C}; // Adresse MAC de l'ESP-01

// Structure pour les messages ESP-NOW
typedef struct struct_msg{
  uint16_t packet_id;
  uint8_t text[32];
  uint32_t value;
} struct_msg;

struct_msg receivingData;
bool newDataReceived = false;

// Variables pour stocker les dernières données ESP-NOW
String lastMessage = "Aucune donnée reçue";
uint16_t lastPacketId = 0;
uint32_t lastValue = 0;
unsigned long lastUpdateTime = 0;


// Serveur web
WebServer server(80);

// LiDAR UART - UART1 fonctionne avec GPIO16/17
HardwareSerial lidarSerial(1);
#define LIDAR_RX 16  // UART1 RX
#define LIDAR_TX 17  // UART1 TX
#define LIDAR_CTL 25  // Pin de contrôle PWM pour la vitesse

// Variables LiDAR (partagées entre les cœurs)
float distance = 0.0;
bool lidarConnected = false;
float lidarData[360]; // Tableau pour stocker les 360 points
uint16_t currentAngle = 0;
uint16_t lidarRPM = 0; // Vitesse de rotation du LiDAR

// Variables pour construction image 360°
bool imageComplete = false;
uint32_t lastCompleteImage = 0;
uint16_t pointsInImage = 0;
String completeImageJson = "";

uint8_t commande = 0;

// Mutex pour la synchronisation entre cœurs
SemaphoreHandle_t lidarMutex;
SemaphoreHandle_t espnowMutex;

// Déclarations de fonctions
void readLiDARData();
void parseLD06Data(uint8_t* data);
void handleRoot();
void handleData();
void handleLidar360();
void handleCommand();
void handleESPNow();
void handleCSS();
void handleJS();
void handleNotFound();
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

// Tâches pour les deux cœurs selon spécifications techniques
void TaskCarControl(void *parameter);  // Cœur 0 : Contrôle voiture + LEDs
void TaskLiDAR(void *parameter);       // Cœur 1 : LiDAR
void TaskWebServer(void *parameter);   // Cœur 1 : GUI web + ESP-NOW

void setup() {
  Serial.begin(115200);
  Serial.println("=== Démarrage ESP32 LiDAR LD06 ===");
  
  // Initialisation de SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("ERREUR: Impossible de monter SPIFFS");
    return;
  }
  Serial.println("SPIFFS monté avec succès");
  
  // Vérification des fichiers SPIFFS (débogage)
  Serial.println("Fichiers SPIFFS disponibles:");
  File root = SPIFFS.open("/");
  if (!root) {
    Serial.println("ERREUR: Impossible d'ouvrir le répertoire racine");
  } else {
    File file = root.openNextFile();
    int fileCount = 0;
    while(file){
      Serial.println("- " + String(file.name()) + " (" + String(file.size()) + " bytes)");
      file = root.openNextFile();
      fileCount++;
    }
    Serial.println("Total: " + String(fileCount) + " fichiers trouvés");
    
    // Vérification spécifique d'index.html
    if (SPIFFS.exists("/index.html")) {
      Serial.println("✓ index.html trouvé");
    } else {
      Serial.println("✗ index.html NON TROUVÉ");
    }
  }
  
  // Initialisation des mutex AVANT de créer les tâches
  lidarMutex = xSemaphoreCreateMutex();
  if (lidarMutex == NULL) {
    Serial.println("ERREUR: Impossible de créer le mutex LiDAR");
    return;
  }
  Serial.println("✓ Mutex LiDAR créé avec succès");
  
  espnowMutex = xSemaphoreCreateMutex();
  if (espnowMutex == NULL) {
    Serial.println("ERREUR: Impossible de créer le mutex ESP-NOW");
    return;
  }
  Serial.println("✓ Mutex ESP-NOW créé avec succès");
  
  // Configuration UART pour LiDAR - UART1 fonctionne
  lidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  Serial.println("UART1 configuré - RX: " + String(LIDAR_RX) + ", TX: " + String(LIDAR_TX) + ", Baud: 230400");
  
  // Configuration PWM pour contrôle de vitesse
  ledcSetup(0, 1000, 8);
  ledcAttachPin(LIDAR_CTL, 0);
  ledcWrite(0, 128);
  
  // Configuration WiFi en mode hybride (AP + STA) pour ESP-NOW + serveur web
  WiFi.mode(WIFI_AP_STA);
  
  // Créer le point d'accès pour le serveur web
  Serial.println("Creation du point d'acces WiFi...");
  bool apStarted = WiFi.softAP(ssid, password);
  if (apStarted) {
    Serial.println("Point d'acces cree avec succes");
  } else {
    Serial.println("ERREUR: Impossible de creer le point d'acces");
  }
  
  // Configuration pour ESP-NOW (mode STA)
  WiFi.begin(); // Se connecte en mode STA (même sans réseau)
  delay(1000); // Délai de stabilisation
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Mot de passe: ");
  Serial.println(password);
  Serial.println("Adresse IP STA: " + WiFi.localIP().toString());
  Serial.println("Adresse MAC: " + WiFi.macAddress());
  
  // Initialisation ESP-NOW
  if(esp_now_init() != ESP_OK) {
    Serial.println("Erreur: Initialisation ESP-NOW failed");
  } else {
    Serial.println("ESP-NOW initialisé avec succès");
  }
  esp_now_register_recv_cb(OnDataRecv);
  
  // Configuration serveur web
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/lidar360", handleLidar360);
  server.on("/command", handleCommand);
  server.on("/espnow", handleESPNow);
  server.on("/style.css", handleCSS);
  server.on("/script.js", handleJS);
  server.onNotFound(handleNotFound);
  server.begin();
  
  Serial.println("LiDAR Hat OKdo - Serveur démarré");
  Serial.println("Connectez-vous au WiFi: " + String(ssid));
  Serial.println("Mot de passe: " + String(password));
  Serial.println("Ouvrez: http://" + IP.toString());
  
  // Vérification de l'alimentation
  Serial.println("=== Vérification système ===");
  Serial.println("Mémoire libre: " + String(ESP.getFreeHeap()) + " bytes");
  Serial.println("Fréquence CPU: " + String(ESP.getCpuFreqMHz()) + " MHz");
  
  // Délai pour s'assurer que tout est initialisé
  delay(1000);
  Serial.println("Initialisation terminée, création des tâches...");
  
  // Création des tâches selon les spécifications techniques imposées
  Serial.println("=== Configuration multi-threading selon spécifications ===");
  
  // CŒUR 0 : Contrôle de la voiture (LEDs contrôlées via WiFi)
  xTaskCreatePinnedToCore(
    TaskCarControl, // Fonction de la tâche
    "TaskCarControl", // Nom de la tâche
    8192,           // Taille de la pile
    NULL,           // Paramètres
    2,              // Priorité haute (contrôle voiture)
    NULL,           // Handle de la tâche
    0               // Cœur 0 : Contrôle voiture + LEDs
  );
  
  // CŒUR 1 : LiDAR + Serveur web (GUI)
  xTaskCreatePinnedToCore(
    TaskLiDAR,      // Fonction de la tâche
    "TaskLiDAR",    // Nom de la tâche
    8192,           // Taille de la pile
    NULL,           // Paramètres
    3,              // Priorité MAXIMALE (LiDAR critique)
    NULL,           // Handle de la tâche
    1               // Cœur 1 : LiDAR + GUI web
  );
  
  xTaskCreatePinnedToCore(
    TaskWebServer,  // Fonction de la tâche
    "TaskWebServer", // Nom de la tâche
    8192,           // Taille de la pile
    NULL,           // Paramètres
    1,              // Priorité normale (GUI web)
    NULL,           // Handle de la tâche
    1               // Cœur 1 : WiFi + Serveur web
  );
  
  Serial.println("✅ Tâches créées selon spécifications techniques :");
  Serial.println("   Cœur 0: Contrôle voiture + LEDs (priorité 2)");
  Serial.println("   Cœur 1: LiDAR (priorité 3) + GUI web (priorité 1)");
}

void loop() {
  // La boucle principale est maintenant vide
  // Les tâches s'exécutent sur les cœurs dédiés
}

// Structure pour parser les données LD06
struct LD06Data {
  uint8_t header[2];
  uint8_t ver_len;
  uint8_t speed[2];
  uint8_t start_angle[2];
  uint8_t data[360];
  uint8_t end_angle[2];
  uint8_t timestamp[2];
  uint8_t crc[2];
};

void readLiDARData() {
  // Code optimisé pour le LD06 - lecture continue et mise à jour immédiate
  static uint8_t buffer[47]; // Taille du paquet LD06
  static uint8_t bufferIndex = 0;
  static unsigned long lastPacketTime = 0;
  static unsigned long lastUpdateTime = 0;
  
  // Lecture continue des données UART
  while (lidarSerial.available()) {
    uint8_t byte = lidarSerial.read();
    
    if (bufferIndex == 0 && byte == 0x54) {
      // Début de paquet trouvé
      buffer[0] = byte;
      bufferIndex = 1;
      lastPacketTime = millis();
    } else if (bufferIndex > 0) {
      buffer[bufferIndex] = byte;
      bufferIndex++;
      
      if (bufferIndex >= 47) {
        // Paquet complet reçu - traitement immédiat
        parseLD06Data(buffer);
        bufferIndex = 0;
        lastPacketTime = millis();
      }
    }
  }
  
  // Mise à jour forcée si pas de données depuis 100ms
  if (millis() - lastPacketTime > 100 && millis() - lastUpdateTime > 50) {
    if (xSemaphoreTake(lidarMutex, 10)) {
      // Marquer comme déconnecté si pas de données récentes
      if (millis() - lastPacketTime > 500) {
        if (lidarConnected) {
          Serial.println("⚠️ LiDAR déconnecté - Vérifiez les connexions");
        }
        lidarConnected = false;
        lidarRPM = 0;
      }
      xSemaphoreGive(lidarMutex);
      lastUpdateTime = millis();
    }
  }
}

void parseLD06Data(uint8_t* data) {
  // Vérification du header (0x54 0x2C)
  if (data[0] == 0x54 && data[1] == 0x2C) {
    // Extraction de la vitesse (little-endian)
    uint16_t speed = (data[3] << 8) | data[2];
    
    // Extraction de l'angle de début (little-endian, en centièmes de degré)
    uint16_t startAngleRaw = (data[5] << 8) | data[4];
    float startAngle = startAngleRaw / 100.0;
    
    // Extraction de l'angle de fin (little-endian, en centièmes de degré)
    uint16_t endAngleRaw = (data[43] << 8) | data[42];
    float endAngle = endAngleRaw / 100.0;
    
    // Correction pour les angles qui dépassent 360°
    if (endAngle < startAngle) {
      endAngle += 360.0;
    }
    
    // Vérification de la validité des angles
    if (startAngle >= 0 && startAngle < 360 && endAngle > startAngle && endAngle <= startAngle + 30) {
      // Calcul de la distance moyenne et remplissage du tableau
      uint16_t totalDistance = 0;
      uint8_t validPoints = 0;
      
      // Mise à jour sécurisée des données LiDAR
      if (xSemaphoreTake(lidarMutex, portMAX_DELAY)) {
        for (int i = 0; i < 12; i++) { // 12 points de données par paquet
          uint16_t dist = (data[6 + i*3 + 1] << 8) | data[6 + i*3];
          if (dist > 0 && dist < 12000) { // Filtrage des valeurs valides
            // Calcul de l'angle pour ce point
            float angleStep = (endAngle - startAngle) / 12.0;
            float pointAngle = startAngle + (i * angleStep);
            
            // Normalisation de l'angle
            while (pointAngle >= 360.0) pointAngle -= 360.0;
            while (pointAngle < 0) pointAngle += 360.0;
            
            // Stockage dans le tableau 360°
            int angleIndex = (int)round(pointAngle);
            if (angleIndex >= 0 && angleIndex < 360) {
              lidarData[angleIndex] = dist;
            }
            
            totalDistance += dist;
            validPoints++;
          }
        }
        
        if (validPoints > 0) {
          distance = totalDistance / validPoints;
          lidarRPM = speed;
          if (!lidarConnected) {
            Serial.println("✅ LiDAR reconnecté - " + String(validPoints) + " points reçus");
          }
          lidarConnected = true;
          pointsInImage += validPoints;
          
          // Vérifier si l'image 360° est complète
          if (pointsInImage >= 300) { // Seuil pour image complète
            imageComplete = true;
            lastCompleteImage = millis();
            pointsInImage = 0;
            
            // Construire le JSON de l'image complète
            completeImageJson = "{\"data\":[";
            for (int i = 0; i < 360; i++) {
              if (i > 0) completeImageJson += ",";
              completeImageJson += lidarData[i];
            }
            completeImageJson += "],\"connected\":";
            completeImageJson += lidarConnected ? "true" : "false";
            completeImageJson += ",\"rpm\":";
            completeImageJson += lidarRPM;
            completeImageJson += "}";
          }
        }
        
        xSemaphoreGive(lidarMutex);
      }
      
      if (validPoints > 0) {
        //Serial.println("LD06 - Vitesse: " + String(speed) + " RPM, Angle: " + String(startAngle, 1) + "-" + String(endAngle, 1) + "°, Distance moyenne: " + String(distance) + " mm, Points: " + String(validPoints));
      }
    } else {
      //Serial.println("LD06 - Angles invalides: " + String(startAngle, 1) + "-" + String(endAngle, 1) + "°");
    }
  }
}


void handleRoot() {
  Serial.println("=== Demande de page principale ===");
  
  // Vérifier si SPIFFS est monté
  if (!SPIFFS.begin(false)) {
    Serial.println("ERREUR: SPIFFS non monté");
    server.send(500, "text/plain", "SPIFFS non disponible");
    return;
  }
  
  // Servir le fichier HTML depuis SPIFFS
  if (SPIFFS.exists("/index.html")) {
    Serial.println("Fichier index.html trouvé, envoi...");
    File file = SPIFFS.open("/index.html", "r");
    if (file) {
      server.streamFile(file, "text/html");
      file.close();
      Serial.println("Fichier envoyé avec succès");
    } else {
      Serial.println("ERREUR: Impossible d'ouvrir le fichier");
      server.send(500, "text/plain", "Erreur lecture fichier");
    }
  } else {
    Serial.println("ERREUR: Fichier index.html non trouvé");
    server.send(404, "text/html", "<h1>Fichier non trouvé</h1><p>Les fichiers SPIFFS ne sont pas uploadés.</p><p>Exécutez: pio run --target uploadfs</p>");
  }
}

void handleData() {
  Serial.println("=== API /data appelee ===");
  
  // Accès sécurisé aux données LiDAR
  if (xSemaphoreTake(lidarMutex, 1000)) { // Timeout de 1 seconde
    String json = "{";
    json += "\"distance\":" + String(distance) + ",";
    json += "\"connected\":" + String(lidarConnected ? "true" : "false") + ",";
    json += "\"timestamp\":" + String(millis());
    json += "}";
    
    Serial.println("Envoi JSON: " + json);
    Serial.println("Distance: " + String(distance) + ", Connected: " + String(lidarConnected));
    server.send(200, "application/json", json);
    xSemaphoreGive(lidarMutex);
  } else {
    Serial.println("ERREUR: Impossible de prendre le mutex");
    server.send(500, "text/plain", "Erreur mutex");
  }
}

void handleLidar360() {
  // API optimisée - Envoi de l'image 360° complète
  if (xSemaphoreTake(lidarMutex, 50)) {
    // Vérification de la mémoire libre
    if (ESP.getFreeHeap() < 15000) {
      server.send(503, "application/json", "{\"error\":\"low_memory\"}");
      xSemaphoreGive(lidarMutex);
      return;
    }
    
    // Vérifier si l'image est complète et récente
    if (imageComplete && (millis() - lastCompleteImage < 1000)) {
      // Envoyer l'image complète pré-construite
      server.send(200, "application/json", completeImageJson);
      imageComplete = false; // Marquer comme envoyée
    } else {
      // Fallback : image partielle si pas d'image complète
      String json = "{\"data\":[";
      for (int i = 0; i < 360; i++) {
        if (i > 0) json += ",";
        json += lidarData[i];
      }
      json += "],\"connected\":";
      json += lidarConnected ? "true" : "false";
      json += ",\"rpm\":";
      json += lidarRPM;
      json += "}";
      server.send(200, "application/json", json);
    }
    
    xSemaphoreGive(lidarMutex);
  } else {
    server.send(503, "application/json", "{\"error\":\"busy\"}");
  }
}

void handleCSS() {
  if (SPIFFS.exists("/style.css")) {
    File file = SPIFFS.open("/style.css", "r");
    if (file) {
      server.streamFile(file, "text/css");
      file.close();
    } else {
      server.send(500, "text/plain", "Erreur lecture CSS");
    }
  } else {
    server.send(404, "text/plain", "CSS non trouvé");
  }
}

void handleJS() {
  if (SPIFFS.exists("/script.js")) {
    File file = SPIFFS.open("/script.js", "r");
    if (file) {
      server.streamFile(file, "application/javascript");
      file.close();
    } else {
      server.send(500, "text/plain", "Erreur lecture JS");
    }
  } else {
    server.send(404, "text/plain", "JS non trouvé");
  }
}

void handleCommand() {
  if (server.hasArg("cmd")) {
    int newCommande = server.arg("cmd").toInt();
    if (newCommande >= 0 && newCommande <= 3) {
      commande = newCommande;
      Serial.println("Commande reçue: " + String(commande));
      server.send(200, "application/json", "{\"status\":\"ok\",\"commande\":" + String(commande) + "}");
    } else {
      server.send(400, "application/json", "{\"error\":\"Commande invalide\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"Paramètre cmd manquant\"}");
  }
}

void handleESPNow() {
  Serial.println("=== API /espnow appelée ===");
  
  // Accès sécurisé aux données ESP-NOW
  if (xSemaphoreTake(espnowMutex, 1000)) { // Timeout de 1 seconde
    DynamicJsonDocument doc(1024);
    
    doc["status"] = "success";
    doc["packet_id"] = lastPacketId;
    doc["message"] = lastMessage;
    doc["value"] = lastValue;
    doc["timestamp"] = lastUpdateTime;
    doc["uptime"] = millis();
    doc["esp01_mac"] = "34:5F:45:5B:9B:2C";
    
    String response;
    serializeJson(doc, response);
    
    Serial.println("Envoi JSON ESP-NOW: " + response);
    server.send(200, "application/json", response);
    xSemaphoreGive(espnowMutex);
  } else {
    Serial.println("ERREUR: Impossible de prendre le mutex ESP-NOW");
    server.send(500, "text/plain", "Erreur mutex ESP-NOW");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Page non trouvée");
}

// ========== FONCTION ESP-NOW ==========

/**
 * Callback de réception des données ESP-NOW
 */
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("=== PACKET ESP-NOW RECEIVED ===");
  Serial.print("From MAC: ");
  for(int i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);
    if(i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.print("Data length: ");
  Serial.print(len);
  Serial.print(" bytes (attendu: ");
  Serial.print(sizeof(struct_msg));
  Serial.println(" bytes)");
  
  // Vérifier si le paquet vient de notre ESP-01
  bool isFromESP01 = true;
  for(int i = 0; i < 6; i++) {
    if(mac[i] != esp01Address[i]) {
      isFromESP01 = false;
      break;
    }
  }
  
  if(!isFromESP01) {
    Serial.println("❌ Paquet ignoré (MAC incorrecte)");
    Serial.print("Attendu: ");
    for(int i = 0; i < 6; i++) {
      Serial.print(esp01Address[i], HEX);
      if(i < 5) Serial.print(":");
    }
    Serial.println();
    Serial.println("=== END ESP-NOW PACKET ===");
    return;
  }
  
  if(len != sizeof(struct_msg)) {
    Serial.print("❌ Paquet ignoré (taille incorrecte: ");
    Serial.print(len);
    Serial.print(" vs ");
    Serial.print(sizeof(struct_msg));
    Serial.println(" attendu)");
    Serial.println("=== END ESP-NOW PACKET ===");
    return;
  }
  
  Serial.println("✅ Paquet valide de l'ESP-01");
  
  // Accès sécurisé aux données ESP-NOW
  if (xSemaphoreTake(espnowMutex, 10)) {
    // Copie directe des données reçues
    memcpy(&receivingData, incomingData, sizeof(receivingData));
    
    // Affichage immédiat des données reçues pour débogage
    Serial.print("Packet ID: ");
    Serial.println(receivingData.packet_id);
    Serial.print("Text: ");
    Serial.println((char*)receivingData.text);
    Serial.print("Value: ");
    Serial.println(receivingData.value);
    
    // Marquer qu'on a de nouvelles données
    newDataReceived = true;
    
    xSemaphoreGive(espnowMutex);
  } else {
    Serial.println("⚠️ Impossible de prendre le mutex ESP-NOW");
  }
  
  Serial.println("=== END ESP-NOW PACKET ===");
}

// Tâche LiDAR sur le cœur 0
void TaskLiDAR(void *parameter) {
  Serial.println("Tâche LiDAR démarrée sur le cœur " + String(xPortGetCoreID()));
  
  // Initialisation du tableau avec des valeurs nulles
  for (int i = 0; i < 360; i++) {
    lidarData[i] = 0; // Pas de données par défaut
  }
  
  for (;;) {
    // Vérification de sécurité
    if (lidarMutex == NULL) {
      Serial.println("ERREUR: Mutex non initialisé dans TaskLiDAR");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    
    // Lecture des données LiDAR
    readLiDARData();
    
    // Aucune simulation - utilisation des vraies données LiDAR uniquement
    // Les données sont mises à jour uniquement par readLiDARData()
    
    // Petite pause pour éviter de surcharger le cœur
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms
  }
}

// Tâche CarControl sur le cœur 0 (selon spécifications)
void TaskCarControl(void *parameter) {
  Serial.println("Tâche CarControl démarrée sur le cœur " + String(xPortGetCoreID()));
  
  // Configuration des LEDs de test pour commandes
  pinMode(LED_BUILTIN, OUTPUT);  // Statut système

  // LEDs pour servos (4 LEDs avec résistances 120Ω obligatoires)
  // Utilisation de D5, D18, D19, D21 (vérifier conflits SPI/I2C)
   pinMode(5, OUTPUT);  // LED Servo avant gauche (verte) - D5
   pinMode(18, OUTPUT); // LED Servo avant droite (verte) - D18
   pinMode(19, OUTPUT); // LED Servo arrière gauche (rouge) - D19
   pinMode(21, OUTPUT); // LED Servo arrière droite (rouge) - D21
  
  for (;;) {
    // Contrôle des LEDs de la voiture
    // (contrôlées via WiFi selon spécifications)
    
    // LED de statut (clignotement)
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Logique de commandes de direction reçues via ESP-NOW
    // 0=Avancer, 1=Reculer, 2=Gauche, 3=Droite, 4/5=Idle
    // LEDs: D5=Avant Gauche, D18=Avant Droite, D19=Arrière Gauche, D21=Arrière Droite
    
    if (commande == 0) {
      // AVANCER - LEDs vertes devant allumées
      digitalWrite(5, HIGH);  // Avant gauche (vert)
      digitalWrite(18, HIGH); // Avant droite (vert)
      digitalWrite(19, LOW);  // Arrière gauche
      digitalWrite(21, LOW);  // Arrière droite
    }
    else if (commande == 1) {
      // RECULER - LEDs rouges arrière allumées
      digitalWrite(5, LOW);   // Avant gauche
      digitalWrite(18, LOW);  // Avant droite
      digitalWrite(19, HIGH); // Arrière gauche (rouge)
      digitalWrite(21, HIGH); // Arrière droite (rouge)
    }
    else if (commande == 2) {
      // GAUCHE - LEDs côté gauche allumées
      digitalWrite(5, HIGH);  // Avant gauche (vert)
      digitalWrite(18, LOW);  // Avant droite
      digitalWrite(19, HIGH); // Arrière gauche (rouge)
      digitalWrite(21, LOW);  // Arrière droite
    }
    else if (commande == 3) {
      // DROITE - LEDs côté droit allumées
      digitalWrite(5, LOW);   // Avant gauche
      digitalWrite(18, HIGH); // Avant droite (vert)
      digitalWrite(19, LOW);  // Arrière gauche
      digitalWrite(21, HIGH); // Arrière droite (rouge)
    }
    else {
      // IDLE (commande 4, 5 ou autre) - Toutes les LEDs éteintes
      digitalWrite(5, LOW);   // Avant gauche
      digitalWrite(18, LOW);  // Avant droite
      digitalWrite(19, LOW);  // Arrière gauche
      digitalWrite(21, LOW);  // Arrière droite
    }
  }
}

// Tâche WebServer + ESP-NOW sur le cœur 1
void TaskWebServer(void *parameter) {
  Serial.println("Tâche WebServer + ESP-NOW démarrée sur le cœur " + String(xPortGetCoreID()));
  
  for (;;) {
    // Vérification de sécurité
    if (lidarMutex == NULL || espnowMutex == NULL) {
      Serial.println("ERREUR: Mutex non initialisé dans TaskWebServer");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    
    // Gestion des données ESP-NOW
    if(newDataReceived) {
      // Accès sécurisé aux données ESP-NOW
      if (xSemaphoreTake(espnowMutex, 10)) {
        // Mise à jour des variables globales
        lastPacketId = receivingData.packet_id;
        lastMessage = String((char*)receivingData.text);
        lastValue = receivingData.value;
        lastUpdateTime = millis();
        
        // Traitement des commandes de direction
        // Si le message contient "COMMAND", utiliser la valeur comme commande
        if (lastMessage == "COMMAND") {
          commande = lastValue; // Utiliser la valeur comme commande (0-4)
          Serial.println("🎮 Commande reçue: " + String(commande));
        }
        // Sinon, essayer de parser le texte comme commande
        else if (lastMessage == "IDLE") {
          commande = 0;
          Serial.println("🎮 Commande: IDLE");
        }
        else if (lastMessage == "AVANCER") {
          commande = 1;
          Serial.println("🎮 Commande: AVANCER");
        }
        else if (lastMessage == "RECULER") {
          commande = 2;
          Serial.println("🎮 Commande: RECULER");
        }
        else if (lastMessage == "GAUCHE") {
          commande = 3;
          Serial.println("🎮 Commande: GAUCHE");
        }
        else if (lastMessage == "DROITE") {
          commande = 4;
          Serial.println("🎮 Commande: DROITE");
        }
        
        // Affichage sur le Serial Monitor
        Serial.println("---New ESP-NOW data received---");
        Serial.println("Packet ID: " + String(receivingData.packet_id));
        Serial.println("Text: " + String((char*)receivingData.text));
        Serial.println("Value: " + String(receivingData.value));
        Serial.println("Commande active: " + String(commande));
        Serial.println("Timestamp: " + String(lastUpdateTime));
        Serial.println("----------------------");
        
        newDataReceived = false;
        xSemaphoreGive(espnowMutex);
      }
    }
    
    // Gestion du serveur web
    server.handleClient();
    
    // Petite pause pour éviter de surcharger le cœur
    vTaskDelay(1 / portTICK_PERIOD_MS); // 1ms
  }
}

