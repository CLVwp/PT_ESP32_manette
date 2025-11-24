#include <Arduino.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h> 
#else
  #include <WiFi.h>
  #include <HTTPClient.h>
#endif

#include <WiFiClient.h>

// Configuration UART
// Sur l'ESP-01S, on utilise le Serial natif (TX=GPIO1, RX=GPIO3)
// Cela correspond aux broches physiques TX/RX du module.
#define UART_BAUD 9600 

// Configuration WiFi
const char* ssid = "RoArm";
const char* password = "12345678";

// Configuration du rover
const char* roverIP = "192.168.4.1"; 

// Buffer pour recevoir les données
String receivedData = "";
String lastCommand = "Aucune";
int commandValue = -1;
const int BUFFER_SIZE = 256;
char buffer[BUFFER_SIZE];
unsigned long lastUpdateTime = 0;

// Déclaration des fonctions
void processCommand(String command);
String sendHTTPCommandToRover(const char* jsonCommand);
String urlEncode(String str);
bool checkEmergency(); 

void setup() {
  // Initialisation du Serial (Sera connecté au RX/TX de l'Arduino)
  Serial.begin(UART_BAUD);
  // Note: On évite les Serial.println de debug au démarrage car l'Arduino écoute déjà.
  // Si vous voulez debuguer, débranchez l'Arduino et regardez le moniteur série.
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Attente connexion WiFi
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    wifiAttempts++;
  }
  
  // Si connecté, on peut éventuellement envoyer un signal, mais attention au parser de l'Arduino
}

void loop() {
  // --- LOGIQUE DE SURVEILLANCE ET RETOUR VERS ARDUINO ---
  static unsigned long lastCheckTime = 0;
  // Vérification toutes les 500ms
  if (millis() - lastCheckTime >= 500) {
    lastCheckTime = millis();

    // 1. Vérifier l'arrêt d'urgence via une requête au rover
    bool isEmergency = checkEmergency();
    
    // 2. Récupérer la force du signal (0 si non connecté)
    int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
    
    // 3. Construire le packet JSON
    int type = isEmergency ? 150 : 1; 
    String jsonPacket = "{\"T\":" + String(type) + ",\"S\":" + String(rssi) + "}";

    // 4. Envoyer à l'Arduino via le Serial natif
    Serial.println(jsonPacket);
  }

  // --- LECTURE UART (Venant de l'Arduino) ---
  if (Serial.available() > 0) {
    // Lire les données disponibles
    int bytesRead = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
    
    if (bytesRead > 0) {
      buffer[bytesRead] = '\0';  // Terminer la chaîne
      receivedData = String(buffer);
      receivedData.trim();  // Supprimer les espaces et retours à la ligne
      
      // Traiter la commande reçue
      processCommand(receivedData);
      lastCommand = receivedData;
    }
  }
  
  // Vérifier la connexion WiFi périodiquement
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck >= 10000) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
    lastWiFiCheck = millis();
  }
  
  delay(10);
}

// --- FONCTIONS ---

bool checkEmergency() {
  if (WiFi.status() != WL_CONNECTED) {
    return false; 
  }

  // Envoi de la requête au rover
  String response = sendHTTPCommandToRover("{\"T\":150}");
  
  if (response.indexOf("true") != -1) {
    return true;
  }
  return false; 
}

void processCommand(String command) {
  commandValue = command.toInt();
  String jsonCommand = "";
  
  switch(commandValue) {
    case 0: // IDLE/STOP
      jsonCommand = "{\"T\":1,\"L\":0,\"R\":0}";
      break; 
    case 1: // HAUT/AVANCER
      jsonCommand = "{\"T\":1,\"L\":0.5,\"R\":0.5}";
      break;  
    case 2: // BAS/RECULER
      jsonCommand = "{\"T\":1,\"L\":-0.5,\"R\":-0.5}";
      break;  
    case 3: // GAUCHE
      jsonCommand = "{\"T\":1,\"L\":-0.5,\"R\":0.5}";
      break;   
    case 4: // DROITE
      jsonCommand = "{\"T\":1,\"L\":0.5,\"R\":-0.5}";
      break;   
    default:
      return; 
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    sendHTTPCommandToRover(jsonCommand.c_str());
  }
}

String urlEncode(String str) {
  String encoded = "";
  char c;
  size_t len = str.length();
  for (size_t i = 0; i < len; i++) {
    c = str.charAt(i);
    if (c == ' ') encoded += "%20";
    else if (c == '"') encoded += "%22";
    else if (c == '{') encoded += "%7B";
    else if (c == '}') encoded += "%7D";
    else if (c == ':') encoded += "%3A";
    else if (c == ',') encoded += "%2C";
    else encoded += c;
  }
  return encoded;
}

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
  
  // Sur ESP8266, begin prend (client, url)
  http.begin(client, url);
  http.setTimeout(5000);
  
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    response = http.getString();
  }
  
  http.end();
  return response;
}