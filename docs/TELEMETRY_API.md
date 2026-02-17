# API Télémétrie & Interface Web

> **Version:** 1.0
> **Serveur:** ESPAsyncWebServer
> **Protocole:** HTTP REST
> **Format:** JSON

---

## Table des Matières

1. [Configuration Réseau](#1-configuration-réseau)
2. [Architecture Logicielle](#2-architecture-logicielle-thread-safety)
3. [API REST (Endpoints)](#3-api-rest-endpoints)
4. [Commandes de Réglage (Tuning)](#4-commandes-de-réglage-tuning)
5. [Frontend (Interface Web)](#5-frontend-interface-web)
6. [Intégration Applications Tierces](#6-intégration-applications-tierces)

---

## 1. Configuration Réseau

### 1.1 Mode WiFi

| Paramètre | Valeur |
|-----------|--------|
| **Mode** | Access Point (AP) |
| **Librairie** | ESPAsyncWebServer + AsyncTCP |

Le drone crée son propre réseau WiFi auquel vous devez vous connecter.

### 1.2 Identifiants par Défaut

```cpp
// Extrait de telemetry.cpp (lignes 10-11)
const char* ssid = "Drone_ESP32";
const char* password = "password123";
```

| Paramètre | Valeur |
|-----------|--------|
| **SSID** | `Drone_ESP32` |
| **Mot de passe** | `password123` |
| **Sécurité** | WPA2-PSK |

### 1.3 Adresse IP

| Paramètre | Valeur |
|-----------|--------|
| **IP Passerelle** | `192.168.4.1` |
| **Port HTTP** | `80` |
| **Masque** | `255.255.255.0` |
| **URL Dashboard** | `http://192.168.4.1/` |

> **Note :** L'IP `192.168.4.1` est l'adresse par défaut de l'ESP32 en mode Access Point.

---

## 2. Architecture Logicielle (Thread-Safety)

### 2.1 Isolation des Tâches FreeRTOS

```cpp
// Extrait de telemetry.cpp (ligne 1803)
xTaskCreatePinnedToCore(telemetryTask, "WifiTask", 8192, NULL, 1, NULL, 1);
```

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| **Nom de la tâche** | `WifiTask` | Identifiant FreeRTOS |
| **Core** | **1** | Séparé de l'IMU (Core 0) |
| **Priorité** | **1** (basse) | Ne bloque pas la boucle de vol |
| **Stack** | 8192 bytes | Augmenté pour le WiFi |

### 2.2 Répartition des Cores

| Core | Tâches | Priorité |
|------|--------|----------|
| **Core 0** | IMU (MPU6050), AltIMU, Radio S.BUS | Haute (3-4) |
| **Core 1** | WiFi/Télémétrie, Loop principale | Basse (1) |

### 2.3 Accès Concurrent aux Données

**Mécanisme utilisé : Pointeur partagé direct (pas de Mutex)**

```cpp
// telemetry.cpp ligne 13
DroneState* drone_data;

// Lecture directe dans les handlers HTTP
drone_data->angle_roll, drone_data->angle_pitch, ...
```

| Aspect | Implémentation |
|--------|----------------|
| **Mutex/Semaphore** | **Non** - Accès direct |
| **Copie atomique** | **Non** |
| **Risque de lecture incohérente** | Faible (lectures rapides, valeurs float atomiques sur ESP32) |

> **Avertissement :** L'absence de mutex signifie qu'en théorie, une lecture pourrait capturer des valeurs mi-à-jour. En pratique, les float 32-bit sont atomiques sur ESP32 et le polling à 5Hz (200ms) minimise ce risque.

### 2.4 Cycle de Vie de la Tâche

```
start_telemetry_task()
        │
        ▼
┌───────────────────┐
│   telemetryTask   │
│                   │
│  1. WiFi.softAP() │
│  2. server.on()   │  ← Configuration des routes
│  3. server.begin()│
│  4. vTaskDelete() │  ← Auto-destruction
└───────────────────┘
```

La tâche se termine après avoir lancé le serveur. Le serveur HTTP est ensuite géré par les interruptions WiFi.

---

## 3. API REST (Endpoints)

### 3.1 Vue d'Ensemble

| Méthode | URL | Description |
|---------|-----|-------------|
| GET | `/` | Page HTML du dashboard |
| GET | `/data` | Télémétrie temps réel (JSON) |
| GET | `/reset_max` | Reset des compteurs de timing |
| GET | `/get_pid` | Lecture des paramètres PID |
| GET | `/set_pid` | Modification des paramètres PID |
| GET | `/motor` | Test moteur individuel |
| GET | `/stop` | Arrêt d'urgence |

---

### 3.2 GET `/` - Dashboard HTML

**Description :** Retourne la page HTML complète de l'interface de pilotage.

| Paramètre | Valeur |
|-----------|--------|
| Content-Type | `text/html` |
| Taille | ~50 KB (embarqué en PROGMEM) |

---

### 3.3 GET `/data` - Télémétrie Temps Réel

**Description :** Retourne toutes les données de vol en temps réel.

**Fréquence de polling recommandée :** 200ms (5 Hz)

#### Réponse JSON Complète

```json
{
  "ar": 1.25,
  "ap": -0.50,
  "ay": 45.30,
  "r1": 1500,
  "r2": 1500,
  "r3": 1000,
  "r4": 1500,
  "lt": 3200,
  "mr": 150,
  "mi": 450,
  "ci": 380,
  "mp": 120,
  "gy": -0.50,
  "poy": 0.00,
  "ax": 0.0123,
  "ay": -0.0045,
  "az": 0.9876,
  "vb": 11.8,
  "alt_ar": 1.30,
  "alt_ap": -0.45,
  "alt_ax": 0.0130,
  "alt_ay": -0.0040,
  "alt_az": 0.9900,
  "alt_gr": 0.15,
  "alt_gp": -0.20,
  "alt_gy": 0.05,
  "alt_ayw": 125.5
}
```

#### Dictionnaire des Clés JSON

| Clé | Type | Unité | Description |
|-----|------|-------|-------------|
| `ar` | float | degrés | Angle Roll (MPU6050) |
| `ap` | float | degrés | Angle Pitch (MPU6050) |
| `ay` | float | degrés | Angle Yaw intégré (MPU6050) |
| `r1` | int | µs | Canal Radio 1 - Roll (1000-2000) |
| `r2` | int | µs | Canal Radio 2 - Pitch (1000-2000) |
| `r3` | int | µs | Canal Radio 3 - Throttle (1000-2000) |
| `r4` | int | µs | Canal Radio 4 - Yaw (1000-2000) |
| `lt` | ulong | µs | Temps de boucle principale |
| `mr` | ulong | µs | Temps max Radio |
| `mi` | ulong | µs | Temps max IMU |
| `ci` | ulong | µs | Temps IMU actuel (tâche) |
| `mp` | ulong | µs | Temps max PID/Moteurs |
| `gy` | float | °/s | Gyro Yaw Input (MPU6050) |
| `poy` | float | - | PID Output Yaw |
| `ax` | float | G | Accélération X (MPU6050) |
| `ay` | float | G | Accélération Y (MPU6050) |
| `az` | float | G | Accélération Z (MPU6050) |
| `vb` | float | V | Tension batterie |
| `alt_ar` | float | degrés | Angle Roll (AltIMU-10) |
| `alt_ap` | float | degrés | Angle Pitch (AltIMU-10) |
| `alt_ax` | float | G | Accélération X (AltIMU-10) |
| `alt_ay` | float | G | Accélération Y (AltIMU-10) |
| `alt_az` | float | G | Accélération Z (AltIMU-10) |
| `alt_gr` | float | °/s | Gyro Roll (AltIMU-10) |
| `alt_gp` | float | °/s | Gyro Pitch (AltIMU-10) |
| `alt_gy` | float | °/s | Gyro Yaw (AltIMU-10) |
| `alt_ayw` | float | degrés | Cap magnétique (AltIMU-10 magnétomètre) |

---

### 3.4 GET `/reset_max` - Reset Compteurs

**Description :** Remet à zéro les compteurs de temps maximum (diagnostics).

#### Requête

```
GET /reset_max
```

#### Réponse

```
200 OK
Content-Type: text/plain

OK
```

#### Effet

```cpp
drone_data->max_time_radio = 0;
drone_data->max_time_imu = 0;
drone_data->max_time_pid = 0;
```

---

### 3.5 GET `/get_pid` - Lecture PID

**Description :** Retourne les paramètres PID actuels.

#### Réponse JSON

```json
{
  "ppr": 1.5000,
  "ipr": 0.00000,
  "dpr": 5.0000,
  "py": 0.0000,
  "iy": 0.00000,
  "dy": 0.0000,
  "ffpr": 0.0000,
  "ffy": 0.0000,
  "pl": 3.0000,
  "ph": 0.0000
}
```

#### Dictionnaire des Clés PID

| Clé | Description | Variable DroneState |
|-----|-------------|---------------------|
| `ppr` | P Pitch/Roll | `p_pitch_roll` |
| `ipr` | I Pitch/Roll | `i_pitch_roll` |
| `dpr` | D Pitch/Roll | `d_pitch_roll` |
| `py` | P Yaw | `p_yaw` |
| `iy` | I Yaw | `i_yaw` |
| `dy` | D Yaw | `d_yaw` |
| `ffpr` | FeedForward Pitch/Roll | `ff_pitch_roll` |
| `ffy` | FeedForward Yaw | `ff_yaw` |
| `pl` | P Level (auto-niveau) | `p_level` |
| `ph` | P Heading (maintien cap) | `p_heading` |

---

### 3.6 GET `/set_pid` - Modification PID

**Description :** Modifie les paramètres PID en temps réel.

#### Requête (Query String)

```
GET /set_pid?ppr=1.8&ipr=0.01&dpr=6.0&py=2.0&iy=0.02&dy=3.0&pl=3.5&ph=0.5&ffpr=0.1&ffy=0.05
```

#### Paramètres Acceptés

| Paramètre | Type | Description |
|-----------|------|-------------|
| `ppr` | float | P Pitch/Roll |
| `ipr` | float | I Pitch/Roll |
| `dpr` | float | D Pitch/Roll |
| `py` | float | P Yaw |
| `iy` | float | I Yaw |
| `dy` | float | D Yaw |
| `ffpr` | float | FeedForward Pitch/Roll |
| `ffy` | float | FeedForward Yaw |
| `pl` | float | P Level |
| `ph` | float | P Heading |

#### Réponse

```
200 OK
Content-Type: text/plain

OK
```

#### Exemple cURL

```bash
# Modifier uniquement le P du Roll/Pitch
curl "http://192.168.4.1/set_pid?ppr=2.0"

# Modifier plusieurs paramètres
curl "http://192.168.4.1/set_pid?ppr=1.8&dpr=7.0&pl=4.0"
```

---

### 3.7 GET `/motor` - Test Moteur Individuel

**Description :** Contrôle un moteur individuellement pour les tests au sol.

> **DANGER : Retirer les hélices avant d'utiliser cette fonction !**

#### Prérequis

Le drone doit être en `MODE_SAFE` ou `MODE_WEB_TEST`.

#### Requête

```
GET /motor?m=1&val=1100
```

| Paramètre | Type | Plage | Description |
|-----------|------|-------|-------------|
| `m` | int | 1-4 | Numéro du moteur |
| `val` | int | 1000-2000 | Signal PWM (µs) |

#### Comportement

1. Si le mode est `MODE_SAFE` → Passe en `MODE_WEB_TEST`
2. Tous les autres moteurs sont mis à 1000 (OFF)
3. Le moteur sélectionné reçoit la valeur demandée

#### Réponse

```
200 OK
Content-Type: text/plain

OK
```

#### Exemple

```bash
# Tester le moteur 3 à 1150µs
curl "http://192.168.4.1/motor?m=3&val=1150"
```

---

### 3.8 GET `/stop` - Arrêt d'Urgence

**Description :** Coupe tous les moteurs immédiatement.

> **SÉCURITÉ : Utilisez cette route en cas d'urgence !**

#### Requête

```
GET /stop
```

#### Comportement

1. Passe le drone en `MODE_SAFE`
2. Appelle `motors_stop()` (tous les moteurs à 1000µs)
3. Reset les valeurs de test web à 1000

#### Réponse

```
200 OK
Content-Type: text/plain

STOPPED
```

---

## 4. Commandes de Réglage (Tuning)

### 4.1 Workflow de Tuning PID

```
┌─────────────────┐
│ 1. GET /get_pid │  ← Lire valeurs actuelles
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────┐
│ 2. Modifier les valeurs dans l'UI  │
└────────┬────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────┐
│ 3. GET /set_pid?ppr=X&ipr=Y&dpr=Z...           │
└────────┬────────────────────────────────────────┘
         │
         ▼
┌─────────────────┐
│ 4. Test en vol  │
└─────────────────┘
```

### 4.2 Payload de Modification PID

#### Exemple Complet

```bash
curl "http://192.168.4.1/set_pid?ppr=1.8&ipr=0.005&dpr=6.5&py=2.5&iy=0.01&dy=4.0&ffpr=0.08&ffy=0.05&pl=3.5&ph=0.2"
```

#### Modification Partielle

Seuls les paramètres fournis sont modifiés :

```bash
# Modifier uniquement le P et D du Roll/Pitch
curl "http://192.168.4.1/set_pid?ppr=2.0&dpr=8.0"
```

### 4.3 Persistance des Paramètres

| Aspect | Statut |
|--------|--------|
| **Stockage** | **RAM uniquement** |
| **EEPROM** | **Non** - Les PID ne sont pas sauvegardés |
| **Persistance au reboot** | **Non** - Retour aux valeurs de `config.h` |

> **Important :** Les paramètres PID modifiés via l'API sont perdus au redémarrage. Pour les rendre permanents, modifiez les valeurs dans `config.h` et recompilez.

#### Valeurs par Défaut (config.h)

```cpp
#define PID_P_ROLL     1.5
#define PID_I_ROLL     0
#define PID_D_ROLL     5
#define PID_P_PITCH    1.5
#define PID_I_PITCH    0
#define PID_D_PITCH    5
#define PID_P_YAW      0
#define PID_I_YAW      0
#define PID_D_YAW      0
```

---

## 5. Frontend (Interface Web)

### 5.1 Technologies Utilisées

| Composant | Technologie |
|-----------|-------------|
| **HTML** | HTML5 |
| **CSS** | Variables CSS, Flexbox, Grid |
| **JavaScript** | Vanilla JS (ES6) |
| **Stockage** | PROGMEM (Flash ESP32) |

### 5.2 Fonctionnalités du Dashboard

#### Carte Attitude
- Affichage Roll, Pitch, Yaw en temps réel
- Barre de temps de boucle avec code couleur (vert < 3ms, orange < 4ms, rouge > 4ms)

#### Carte Diagnostics
- Temps Radio (RX)
- Temps IMU (actuel et max)
- Temps PID/Moteurs
- Tension batterie avec alertes couleur
- Bouton de réinitialisation des compteurs

#### Moniteur Radio
- 4 barres de progression pour Gaz, Yaw, Pitch, Roll
- Valeurs numériques en temps réel (1000-2000 µs)

#### Réglage PID
- Champs numériques pour tous les gains PID
- Section Pitch/Roll : P, I, D, FeedForward
- Section Yaw : P, I, D, FeedForward
- Section Auto-niveau : P Level, P Heading
- Bouton "Appliquer PID" avec feedback visuel

#### Test Moteurs
- Grille 2x2 pour sélection moteur (M1-M4)
- Slider de puissance (1000-1300 µs)
- Bouton d'arrêt d'urgence (rouge)

#### Comparaison IMU
- Données MPU6050 (principal) : Roll, Pitch, Gyro Yaw
- Données AltIMU-10 (secondaire) : Roll, Pitch, Yaw magnétique
- Écart calculé (ΔRoll, ΔPitch) avec alerte si > 5°

#### Graphique Accéléromètre
- Graphique temps réel des axes X, Y, Z
- Échelle configurable (±0.01G à ±2G)
- Calcul du bruit (écart-type σ)
- Bouton auto-échelle

#### Preuve de Dérive Inertielle (PoC)
- Visualisation 3D avec cube représentant le drone
- Intégration double de l'accéléromètre
- Démonstration de la dérive sans GPS/VIO
- Trail de position avec échelle configurable

### 5.3 Polling et Mise à Jour

```javascript
// Extrait du JavaScript embarqué
setInterval(() => {
  fetch('/data').then(res => res.json()).then(data => {
    // Mise à jour de l'UI...
  });
}, 200);  // 5 Hz
```

| Paramètre | Valeur |
|-----------|--------|
| **Fréquence** | 5 Hz (200ms) |
| **Méthode** | Polling HTTP GET |
| **Format** | JSON |

---

## 6. Intégration Applications Tierces

### 6.1 Connexion au Drone

```python
import requests
import time

DRONE_IP = "192.168.4.1"
BASE_URL = f"http://{DRONE_IP}"

# Lecture télémétrie
def get_telemetry():
    response = requests.get(f"{BASE_URL}/data")
    return response.json()

# Boucle de lecture
while True:
    data = get_telemetry()
    print(f"Roll: {data['ar']:.1f}° Pitch: {data['ap']:.1f}° Bat: {data['vb']:.1f}V")
    time.sleep(0.2)
```

### 6.2 Modification PID via Script

```python
def set_pid(p_roll=None, i_roll=None, d_roll=None):
    params = {}
    if p_roll is not None: params['ppr'] = p_roll
    if i_roll is not None: params['ipr'] = i_roll
    if d_roll is not None: params['dpr'] = d_roll

    response = requests.get(f"{BASE_URL}/set_pid", params=params)
    return response.status_code == 200

# Exemple: augmenter le P
set_pid(p_roll=2.0)
```

### 6.3 Test Moteur Automatisé

```python
def test_motor(motor_num, power):
    """
    motor_num: 1-4
    power: 1000-1300 (recommandé pour tests)
    """
    requests.get(f"{BASE_URL}/motor", params={'m': motor_num, 'val': power})

def emergency_stop():
    requests.get(f"{BASE_URL}/stop")

# Test séquentiel des 4 moteurs
for m in range(1, 5):
    print(f"Test moteur {m}...")
    test_motor(m, 1100)
    time.sleep(2)
    emergency_stop()
    time.sleep(1)
```

### 6.4 Mapping des Clés JSON pour Développeurs

```javascript
// Mapping complet pour applications tierces
const TELEMETRY_KEYS = {
  // Attitude (MPU6050)
  angle_roll: 'ar',
  angle_pitch: 'ap',
  angle_yaw: 'ay',

  // Radio
  channel_roll: 'r1',
  channel_pitch: 'r2',
  channel_throttle: 'r3',
  channel_yaw: 'r4',

  // Timing
  loop_time: 'lt',
  max_radio: 'mr',
  max_imu: 'mi',
  current_imu: 'ci',
  max_pid: 'mp',

  // Accéléromètre (G)
  accel_x: 'ax',
  accel_y: 'ay',  // Note: conflit avec angle_yaw!
  accel_z: 'az',

  // Batterie
  voltage: 'vb',

  // AltIMU-10
  alt_angle_roll: 'alt_ar',
  alt_angle_pitch: 'alt_ap',
  alt_heading: 'alt_ayw'  // Cap magnétique
};
```

> **Note :** Il existe un conflit de nommage entre `ay` (angle yaw) et `ay` (accélération Y) dans la réponse JSON. Utilisez le contexte pour distinguer.

---

## Notes de Sécurité

> **DANGER - TEST MOTEURS**
> - Toujours retirer les hélices avant d'utiliser `/motor`
> - Le slider est limité à 1300µs par sécurité
> - Utilisez `/stop` en cas de problème

> **ATTENTION - RÉSEAU WiFi**
> - Le mot de passe par défaut est faible (`password123`)
> - Changez-le en production dans `telemetry.cpp`
> - Le réseau n'est pas chiffré au-delà de WPA2

> **INFO - PERSISTANCE**
> - Les modifications PID ne survivent pas au redémarrage
> - Notez vos valeurs optimales pour les coder en dur dans `config.h`

---

*Documentation générée à partir de l'analyse du code source `telemetry.cpp` v1.0*
