# ESP32 LiDAR LD06 - Interface Web

## Description

Interface web pour visualiser les données du LiDAR LD06 en temps réel avec l'ESP32.

## Structure du projet

```
PT_ESP32/
├── data/                 # Fichiers web (SPIFFS)
│   ├── index.html       # Page principale
│   ├── style.css        # Styles CSS
│   └── script.js        # Logique JavaScript
├── src/
│   └── main.cpp         # Code principal ESP32
├── platformio.ini       # Configuration PlatformIO
└── README.md           # Ce fichier
```

## Installation

### 1. Configuration PlatformIO

Le fichier `platformio.ini` est déjà configuré avec :

- Support SPIFFS activé
- Partition par défaut
- Vitesse de monitoring 115200

### 2. Upload des fichiers web

```bash
# Upload du code ESP32
pio run --target upload

# Upload des fichiers web vers SPIFFS
pio run --target uploadfs
```

### 3. Connexion

1. Alimentez l'ESP32
2. Connectez-vous au WiFi "ESP32_LiDAR" (mot de passe: 12345678)
3. Ouvrez http://192.168.4.1 dans votre navigateur

## Fonctionnalités

### Interface Web

- **Radar en temps réel** : Visualisation 360° des données LiDAR
- **Contrôles ajustables** : Distance max, vitesse d'actualisation, persistance
- **Statut système** : Connexion LiDAR, distance moyenne, points valides
- **Design responsive** : Compatible mobile et desktop

### API Endpoints

- `GET /` : Page principale
- `GET /data` : Données de statut
- `GET /lidar360` : Données radar 360°
- `GET /style.css` : Fichier CSS
- `GET /script.js` : Fichier JavaScript

## Configuration

### Paramètres LiDAR

- **Vitesse** : 3000+ RPM
- **Port UART** : UART1 (GPIO 16/17)
- **Baud rate** : 230400
- **Contrôle PWM** : GPIO 25

### Paramètres Web

- **Actualisation** : 200ms par défaut
- **Persistance** : 500ms par défaut
- **Distance max** : 12m par défaut
- **Filtre** : 50mm - 12000mm

## Optimisations

### Gestion mémoire

- Image 360° construite avant envoi
- Vérification mémoire libre
- Timeout mutex réduit
- Gestion d'erreur robuste

### Performance

- Accumulation de 300 points pour image complète
- Envoi d'image pré-construite
- Fallback sur image partielle
- Gestion d'erreur 503 si surcharge

## Dépannage

### Erreur "ERR_INSUFFICIENT_RESOURCES"

- Vérifiez la mémoire libre
- Réduisez la fréquence d'actualisation
- Attendez que l'image soit complète

### Radar ne s'affiche pas

- Vérifiez la connexion LiDAR
- Vérifiez les LEDs bleues
- Consultez le moniteur série

### Fichiers web non trouvés

- Exécutez `pio run --target uploadfs`
- Vérifiez que le dossier `data/` existe
- Redémarrez l'ESP32

## Développement

### Modification des fichiers web

1. Modifiez les fichiers dans `data/`
2. Exécutez `pio run --target uploadfs`
3. Rechargez la page web

### Modification du code ESP32

1. Modifiez `src/main.cpp`
2. Exécutez `pio run --target upload`
3. Redémarrez l'ESP32

## Licence

Projet open source - Libre d'utilisation et modification.
