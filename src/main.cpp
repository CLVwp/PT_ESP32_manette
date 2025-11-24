#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>

// Configuration UART pour communication avec Arduino Nano
#define UART_RX_PIN 16  // GPIO 16 = RX de l'ESP32 (reçoit du TX de l'Arduino)
#define UART_TX_PIN 17  // GPIO 17 = TX de l'ESP32 (envoie au RX de l'Arduino)
#define UART_BAUD 9600  // Vitesse de communication

// Configuration WiFi
const char* ssid = "RoArm";
const char* password = "12345678";

// Configuration du rover
const char* roverIP = "192.168.4.1"; // Adresse IP du rover

// Instance UART (UART2 sur ESP32)
HardwareSerial SerialArduino(2);

// Buffer pour recevoir les données
String receivedData = "";
String lastCommand = "Aucune";
String lastDirection = "---";
int commandValue = -1;
const int BUFFER_SIZE = 256;
char buffer[BUFFER_SIZE];
unsigned long lastUpdateTime = 0;
int commandCount = 0;

// Déclaration des fonctions
void processCommand(String command);
void displayStatus();
String sendHTTPCommandToRover(const char* jsonCommand);
String urlEncode(String str);
bool checkEmergency(); // Fonction pour vérifier l'arrêt d'urgence

void setup() {
  // Initialisation du Serial Monitor pour le debug
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 UART Receiver + WiFi ===");
  Serial.println("");
  
  // Initialisation de l'UART pour communiquer avec l'Arduino Nano
  SerialArduino.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART initialisé sur GPIO " + String(UART_RX_PIN) + " (RX) et GPIO " + String(UART_TX_PIN) + " (TX)");
  Serial.println("");

  // Connexion au WiFi
  Serial.print("Connexion au WiFi: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Attendre la connexion WiFi
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println("");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ WiFi connecté !");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("   MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("   Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("❌ Échec de connexion WiFi");
    Serial.println("   Vérifiez les credentials et la portée du signal");
  }
  
  Serial.println("");
  Serial.println("=== Système prêt ===");
  Serial.println("En attente de commandes UART...");
  Serial.println("");
}

void loop() {
  // --- LOGIQUE DE SURVEILLANCE ET RETOUR VERS ARDUINO ---
  static unsigned long lastCheckTime = 0;
  // Vérification toutes les 500ms (ajustable)
  if (millis() - lastCheckTime >= 500) {
    lastCheckTime = millis();

    // 1. Vérifier l'arrêt d'urgence via une requête au rover
    bool isEmergency = checkEmergency();
    
    // 2. Récupérer la force du signal (0 si non connecté)
    int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
    
    // 3. Construire le packet JSON : {"T":150} si urgence, sinon {"T":1} (exemple pour "Normal") + Signal
    // Format : {"T":<type>, "S":<rssi>}
    int type = isEmergency ? 150 : 1; 
    String jsonPacket = "{\"T\":" + String(type) + ",\"S\":" + String(rssi) + "}";

    // 4. Printf dans le Serial Monitor (DEBUG)
    Serial.printf("Packet pour Arduino: %s\n", jsonPacket.c_str());

    // 5. Envoyer à l'Arduino via SerialArduino
    SerialArduino.println(jsonPacket);
  }
  // --- FIN NOUVELLE LOGIQUE ---

  // Vérifier si des données sont disponibles sur l'UART
  if (SerialArduino.available() > 0) {
    // Lire les données disponibles
    int bytesRead = SerialArduino.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
    
    if (bytesRead > 0) {
      buffer[bytesRead] = '\0';  // Terminer la chaîne
      receivedData = String(buffer);
      receivedData.trim();  // Supprimer les espaces et retours à la ligne
      
      // Afficher les données reçues
      Serial.print("📥 Données reçues: ");
      Serial.println(receivedData);
      
      // Traiter la commande reçue
      processCommand(receivedData);
      
      // Mettre à jour les variables pour l'affichage continu
      lastCommand = receivedData;
      lastUpdateTime = millis();
      commandCount++;
    }
  }
  
  // Afficher le statut en continu toutes les 5 secondes
  static unsigned long lastDisplayTime = 0;
  if (millis() - lastDisplayTime >= 5000) {
    displayStatus();
    lastDisplayTime = millis();
  }
  
  // Vérifier la connexion WiFi périodiquement
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck >= 10000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ WiFi déconnecté, tentative de reconnexion...");
      WiFi.reconnect();
    }
    lastWiFiCheck = millis();
  }
  
  // Petite pause pour éviter de surcharger le CPU
  delay(10);
}

// --- FONCTIONS ---

// Fonction pour vérifier l'état d'urgence
// Envoie une requête HTTP {"T":150} au rover et vérifie si la réponse contient "true"
bool checkEmergency() {
  if (WiFi.status() != WL_CONNECTED) {
    return false; // Pas de WiFi, on suppose pas d'urgence du rover (ou alors urgence communication ?)
  }

  // Envoi de la requête au rover
  String response = sendHTTPCommandToRover("{\"T\":150}");
  
  // Analyse de la réponse
  // Si la réponse contient "true", on considère qu'il y a une urgence
  if (response.indexOf("true") != -1) {
    return true;
  }
  
  return false; 
}

void processCommand(String command) {
  // Convertir la chaîne en entier
  commandValue = command.toInt();
  
  // Afficher la commande reçue
  Serial.print("🔧 Commande reçue: ");
  Serial.print(commandValue);
  Serial.print(" -> ");
  
  // Traiter les commandes selon leur valeur et envoyer via HTTP
  String jsonCommand = "";
  
  switch(commandValue) {
    case 0: // IDLE/STOP
      Serial.println("IDLE");
      lastDirection = "IDLE";
      jsonCommand = "{\"T\":1,\"L\":0,\"R\":0}";
      break;
      
    case 1: // HAUT/AVANCER
      Serial.println("HAUT");
      lastDirection = "HAUT";
      jsonCommand = "{\"T\":1,\"L\":0.5,\"R\":0.5}";
      break;
      
    case 2: // BAS/RECULER
      Serial.println("BAS");
      lastDirection = "BAS";
      jsonCommand = "{\"T\":1,\"L\":-0.5,\"R\":-0.5}";
      break;
      
    case 3: // GAUCHE
      Serial.println("GAUCHE");
      lastDirection = "GAUCHE";
      jsonCommand = "{\"T\":1,\"L\":-0.5,\"R\":0.5}";
      break;
      
    case 4: // DROITE
      Serial.println("DROITE");
      lastDirection = "DROITE";
      jsonCommand = "{\"T\":1,\"L\":0.5,\"R\":-0.5}";
      break;
      
    default:
      Serial.print("COMMANDE INCONNUE (");
      Serial.print(commandValue);
      Serial.println(")");
      Serial.println("   Valeurs acceptées: 0=IDLE, 1=HAUT, 2=BAS, 3=GAUCHE, 4=DROITE");
      lastDirection = "INCONNUE";
      return; // Ne pas envoyer de commande HTTP si invalide
  }
  
  // Envoyer la commande au rover via HTTP si WiFi connecté
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("📤 Envoi HTTP au rover: ");
    Serial.println(jsonCommand);
    sendHTTPCommandToRover(jsonCommand.c_str());
  } else {
    Serial.println("❌ WiFi non connecté - Commande non envoyée");
  }
  
  Serial.println("");
}

// Fonction pour encoder une chaîne en URL (pour les caractères spéciaux du JSON)
String urlEncode(String str) {
  String encoded = "";
  char c;
  size_t len = str.length();
  for (size_t i = 0; i < len; i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encoded += "%20";
    } else if (c == '"') {
      encoded += "%22";
    } else if (c == '{') {
      encoded += "%7B";
    } else if (c == '}') {
      encoded += "%7D";
    } else if (c == ':') {
      encoded += "%3A";
    } else if (c == ',') {
      encoded += "%2C";
    } else {
      encoded += c;
    }
  }
  return encoded;
}

// Fonction pour envoyer une commande JSON au rover via HTTP
// Retourne la réponse sous forme de String
String sendHTTPCommandToRover(const char* jsonCommand) {
  WiFiClient client;
  HTTPClient http;
  String response = "";
  
  String jsonStr = String(jsonCommand);
  String encodedJson = urlEncode(jsonStr);
  
  String url = "http://";
  url += roverIP;
  url += "/js?json=";
  url += encodedJson;
  
  http.begin(client, url);
  http.setTimeout(5000); // Timeout de 5 secondes
  
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    if (httpCode == 200) {
      Serial.println("✅ Commande envoyée avec succès (HTTP 200)");
    } else {
      Serial.print("⚠️ Réponse HTTP: ");
      Serial.println(httpCode);
    }
    response = http.getString();
    // Optionnel: afficher la réponse si nécessaire
    // Serial.println("Réponse: " + response);
  } else {
    Serial.print("❌ Erreur HTTP: ");
    Serial.println(httpCode);
  }
  
  http.end();
  return response;
}

void displayStatus() {
  // Afficher le statut en continu
  Serial.println("═══════════════════════════════════════");
  Serial.println("📊 STATUT EN TEMPS RÉEL");
  Serial.println("═══════════════════════════════════════");
  
  // Statut WiFi
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("📶 WiFi: CONNECTÉ (");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm)");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("📶 WiFi: DÉCONNECTÉ");
  }
  
  // Statut des commandes
  Serial.print("📥 Dernière commande: ");
  Serial.println(lastCommand);
  Serial.print("🎯 Direction: ");
  Serial.println(lastDirection);
  Serial.print("🔢 Valeur: ");
  Serial.println(commandValue);
  Serial.print("📈 Total commandes: ");
  Serial.println(commandCount);
  Serial.print("⏱️  Temps: ");
  Serial.print(millis() / 1000);
  Serial.println(" secondes");
  
  if (lastUpdateTime > 0) {
    Serial.print("🕐 Dernière mise à jour: il y a ");
    Serial.print((millis() - lastUpdateTime) / 1000);
    Serial.println(" secondes");
  }
  
  Serial.println("═══════════════════════════════════════");
  Serial.println("");
}
