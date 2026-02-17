# ESP32 Flight Controller

<div align="center">

![ESP32](https://img.shields.io/badge/ESP32-FireBeetle32-blue?style=for-the-badge&logo=espressif)
![PlatformIO](https://img.shields.io/badge/PlatformIO-5.0+-orange?style=for-the-badge&logo=platformio)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Development-yellow?style=for-the-badge)

**Contrôleur de vol open-source pour quadcopter basé sur ESP32 avec télémétrie WiFi intégrée**

*Projet UMons 2025-2026*

</div>

---

## Aperçu du Projet

Ce projet implémente un contrôleur de vol complet pour quadcopter utilisant un microcontrôleur **ESP32 FireBeetle32**. Il intègre une stabilisation par filtre de Kalman, une interface de télémétrie WiFi temps réel, et supporte une configuration double IMU pour la redondance et la comparaison des capteurs.

### Caractéristiques Principales

- **Stabilisation avancée** : Filtre de Kalman pour fusion gyroscope/accéléromètre
- **Double IMU** : MPU6050 (principal) + AltIMU-10 v2 (secondaire avec magnétomètre)
- **Télémétrie WiFi** : Dashboard web temps réel accessible depuis tout appareil
- **FreeRTOS** : Architecture multi-tâches pour performances optimales
- **Radio S.BUS** : Compatible avec la plupart des récepteurs RC modernes
- **Réglage PID en vol** : Modification des paramètres via l'interface web

---

## Spécifications Hardware

### Plateforme

| Composant | Spécification |
|-----------|---------------|
| **Frame** | Quadcopter X (450mm recommandé) |
| **Batterie** | LiPo 3S 11.1V 2500mAh |
| **Autonomie** | ~10-15 min (selon charge) |
| **Poids total** | ~800g - 1.2kg |

### Électronique

| Composant | Modèle | Rôle |
|-----------|--------|------|
| **MCU** | ESP32 FireBeetle32 | Contrôleur principal (240MHz Dual-Core) |
| **IMU Principal** | MPU6050 | Gyroscope + Accéléromètre 6 axes |
| **IMU Secondaire** | Pololu AltIMU-10 v2 | L3GD20 (gyro) + LSM303DLHC (accel/mag) |
| **ESC** | 4x ESC 30A (OneShot125 compatible) | Contrôle moteurs |
| **Moteurs** | 4x Brushless 2212-2300KV | Propulsion |
| **Hélices** | 10x4.5" ou 9x4.7" | Adaptées aux moteurs |
| **Radio RX** | Récepteur S.BUS (FrSky, etc.) | Réception radiocommande |
| **Buzzer** | Optionnel | Alertes sonores |

### Batterie

```
╔═══════════════════════════════════════════════════════════╗
║                    LiPo 3S 2500mAh                        ║
╠═══════════════════════════════════════════════════════════╣
║  Configuration    : 3S (3 cellules en série)              ║
║  Tension nominale : 11.1V                                 ║
║  Tension max      : 12.6V (chargée)                       ║
║  Tension min      : 10.5V (seuil alarme)                  ║
║  Capacité         : 2500mAh                               ║
║  Décharge         : 25C-35C recommandé                    ║
╚═══════════════════════════════════════════════════════════╝
```

---

## Architecture Logicielle

### Organisation des Tâches FreeRTOS

```
                    ┌─────────────────────────────────────────┐
                    │              ESP32 Dual-Core            │
                    ├────────────────────┬────────────────────┤
                    │      CORE 0        │      CORE 1        │
                    │   (Temps Réel)     │   (Communication)  │
                    ├────────────────────┼────────────────────┤
                    │                    │                    │
                    │  ┌──────────────┐  │  ┌──────────────┐  │
                    │  │  IMU Task    │  │  │  WiFi Task   │  │
                    │  │  (250 Hz)    │  │  │  (Async)     │  │
                    │  │  Priority: 4 │  │  │  Priority: 1 │  │
                    │  └──────────────┘  │  └──────────────┘  │
                    │                    │                    │
                    │  ┌──────────────┐  │  ┌──────────────┐  │
                    │  │ AltIMU Task  │  │  │  Main Loop   │  │
                    │  │  (25 Hz)     │  │  │  (250 Hz)    │  │
                    │  │  Priority: 1 │  │  │  PID + Mix   │  │
                    │  └──────────────┘  │  └──────────────┘  │
                    │                    │                    │
                    │  ┌──────────────┐  │                    │
                    │  │  Radio Task  │  │                    │
                    │  │  (1 kHz)     │  │                    │
                    │  │  Priority: 3 │  │                    │
                    │  └──────────────┘  │                    │
                    │                    │                    │
                    └────────────────────┴────────────────────┘
```

### Modules du Firmware

| Fichier | Description |
|---------|-------------|
| `main.cpp` | Boucle principale, gestion des modes, sécurités |
| `imu.cpp` | Lecture MPU6050, filtre Kalman, tâche FreeRTOS |
| `alt_imu.cpp` | Lecture AltIMU-10 (L3GD20 + LSM303DLHC), magnétomètre |
| `radio.cpp` | Décodage S.BUS, tâche asynchrone |
| `pid.cpp` | Contrôleur PID triple axe, auto-niveau |
| `motors.cpp` | PWM 250Hz, mixage quadcopter X |
| `telemetry.cpp` | Serveur web, API REST, dashboard HTML |
| `kalman.cpp` | Implémentation filtre de Kalman |
| `yaw_fusion.cpp` | Fusion gyro/magnétomètre pour le cap |
| `esc_calibrate.cpp` | Procédure de calibration ESC |
| `setup_wizard.cpp` | Assistant de configuration radio |

---

## Câblage

### Pinout ESP32 FireBeetle32

```
                         ┌─────────────────┐
                         │   ESP32         │
                         │   FireBeetle32  │
                         ├─────────────────┤
                   3V3 ──┤ 3V3         VIN ├── 5V (USB/Régulateur)
                   GND ──┤ GND         GND ├── GND
                         │                 │
        Moteur 1 (FR) ◄──┤ IO27       IO34 ├── (ADC uniquement)
        Moteur 2 (RR) ◄──┤ IO13       IO35 ├── (ADC uniquement)
        Moteur 3 (RL) ◄──┤ IO25       IO32 ├──
        Moteur 4 (FL) ◄──┤ IO26       IO33 ├──
                         │                 │
          S.BUS RX ────►─┤ IO4        IO14 ├──
           LED ────────►─┤ IO5        IO12 ├──
                         │                 │
       I2C SDA ◄───────►─┤ IO21       IO2  ├──
       I2C SCL ◄───────►─┤ IO22       IO15 ├──
                         │                 │
     Batterie (ADC) ───►─┤ IO34            │
                         └─────────────────┘
```

### Configuration des Moteurs (Quadcopter X)

```
           AVANT
             │
     M4      │      M1
    (FL)     │     (FR)
      ╲      │      ╱
       ╲     │     ╱
        ╲    │    ╱
         ╲   │   ╱
          ╲  │  ╱
           ╲ │ ╱
            ╳│╳
           ╱ │ ╲
          ╱  │  ╲
         ╱   │   ╲
        ╱    │    ╲
       ╱     │     ╲
      ╱      │      ╲
     M3      │      M2
    (RL)     │     (RR)
             │
          ARRIÈRE

┌────────┬────────┬──────────┬───────────────────────────┐
│ Moteur │ GPIO   │ Rotation │ Mixage PID                │
├────────┼────────┼──────────┼───────────────────────────┤
│ M1 FR  │ IO27   │ CCW ↺    │ -Pitch +Roll -Yaw        │
│ M2 RR  │ IO13   │ CW  ↻    │ +Pitch +Roll +Yaw        │
│ M3 RL  │ IO25   │ CCW ↺    │ +Pitch -Roll -Yaw        │
│ M4 FL  │ IO26   │ CW  ↻    │ -Pitch -Roll +Yaw        │
└────────┴────────┴──────────┴───────────────────────────┘
```

### Bus I2C (400 kHz)

| Capteur | Adresse | Pins |
|---------|---------|------|
| MPU6050 | `0x68` | SDA: IO21, SCL: IO22 |
| L3GD20 (Gyro) | `0x6B` | SDA: IO21, SCL: IO22 |
| LSM303 Accel | `0x19` | SDA: IO21, SCL: IO22 |
| LSM303 Mag | `0x1E` | SDA: IO21, SCL: IO22 |

### Diviseur de Tension Batterie

```
     Batterie LiPo 3S
          (+)
           │
          ┌┴┐
          │ │ R1 = 10kΩ
          └┬┘
           ├────────► IO34 (ADC)
          ┌┴┐
          │ │ R2 = 2.2kΩ
          └┬┘
           │
          GND

  Vout = Vin × R2/(R1+R2) = Vin × 0.18
  Facteur d'échelle: BAT_SCALE = 5.88
```

---

## Installation

### Prérequis

- [PlatformIO](https://platformio.org/) (extension VS Code recommandée)
- Câble USB-C pour le FireBeetle32
- Python 3.x (pour PlatformIO)

### Étapes

1. **Cloner le dépôt**
   ```bash
   git clone <url-du-repo>
   cd ESP32FlightController-main_vtest
   ```

2. **Ouvrir dans VS Code avec PlatformIO**
   ```bash
   code .
   ```

3. **Compiler le firmware**
   ```bash
   pio run
   ```

4. **Flasher l'ESP32**
   ```bash
   pio run --target upload
   ```

5. **Moniteur série** (optionnel)
   ```bash
   pio device monitor --baud 115200
   ```

### Dépendances (automatiques via PlatformIO)

```ini
lib_deps =
    bolderflight/Bolder Flight Systems SBUS @ ^8.1.4
    https://github.com/me-no-dev/ESPAsyncWebServer.git
    https://github.com/me-no-dev/AsyncTCP.git
    adafruit/Adafruit MPU6050 @ ^2.2.4
    adafruit/Adafruit Unified Sensor @ ^1.1.9
    adafruit/Adafruit BusIO @ ^1.14.1
    rfetick/MPU6050_light @ ^1.1.0
```

---

## Utilisation Rapide

### 1. Première Mise Sous Tension

1. **Sans hélices**, brancher la batterie
2. Attendre la séquence de calibration (~45 secondes total)
3. La LED fixe indique que le drone est prêt (MODE_SAFE)

### 2. Connexion à la Télémétrie

| Paramètre | Valeur |
|-----------|--------|
| **Réseau WiFi** | `Drone_ESP32` |
| **Mot de passe** | `password123` |
| **URL Dashboard** | `http://192.168.4.1` |

### 3. Armement

```
Gaz en BAS + Yaw à GAUCHE pendant 1 seconde
→ LED clignote lentement = ARMÉ
```

### 4. Désarmement / Urgence

```
Gaz en BAS + Yaw à DROITE pendant 1 seconde
→ LED fixe = DÉSARMÉ
```

---

## Télémétrie WiFi

Le drone crée son propre réseau WiFi. Connectez-vous et accédez au dashboard via votre navigateur.

### Fonctionnalités du Dashboard

| Carte | Description |
|-------|-------------|
| **Attitude** | Roll, Pitch, Yaw en temps réel |
| **Diagnostics** | Temps de boucle, tension batterie |
| **Moniteur Radio** | Valeurs des 4 canaux RC |
| **Réglage PID** | Modification en temps réel |
| **Test Moteurs** | Contrôle individuel (hélices retirées !) |
| **Comparaison IMU** | MPU6050 vs AltIMU-10 |
| **Graphique Accel** | Visualisation accéléromètre |
| **Dérive Inertielle** | Démonstration du drift sans GPS |

### API REST

| Endpoint | Description |
|----------|-------------|
| `GET /data` | Données télémétrie JSON |
| `GET /get_pid` | Lecture paramètres PID |
| `GET /set_pid?...` | Modification PID |
| `GET /motor?m=X&val=Y` | Test moteur |
| `GET /stop` | Arrêt d'urgence |

---

## Documentation

| Document | Description |
|----------|-------------|
| [`docs/USER_GUIDE.md`](docs/USER_GUIDE.md) | Manuel de pilotage complet |
| [`docs/TELEMETRY_API.md`](docs/TELEMETRY_API.md) | Documentation API REST |

---

## Configuration (config.h)

```cpp
// Pins Moteurs
#define PIN_MOTOR_1    27  // Avant-Droit
#define PIN_MOTOR_2    13  // Arrière-Droit
#define PIN_MOTOR_3    25  // Arrière-Gauche
#define PIN_MOTOR_4    26  // Avant-Gauche

// Fréquence ESC
#define ESC_FREQ       250  // Hz

// Radio S.BUS
#define PIN_SBUS_RX    4

// LED Status
#define PIN_LED        5

// I2C
#define I2C_SPEED      400000  // 400 kHz

// Boucle principale
#define LOOP_TIME_US   4000    // 250 Hz

// PID par défaut
#define PID_P_ROLL     1.5
#define PID_I_ROLL     0
#define PID_D_ROLL     5
```

---

## Sécurité

> **AVERTISSEMENT : Ce projet implique des composants potentiellement dangereux (hélices en rotation, batteries LiPo). Suivez toujours les consignes de sécurité.**

- **Toujours retirer les hélices** pour les tests au sol
- **Ne jamais laisser une LiPo sans surveillance** en charge
- **Voler dans un espace dégagé** et respecter la réglementation locale
- **Vérifier les connexions** avant chaque vol
- **Ne pas voler avec une batterie faible** (< 10.5V)

---

## Roadmap

- [x] Stabilisation Kalman Roll/Pitch
- [x] Télémétrie WiFi temps réel
- [x] Double IMU (MPU6050 + AltIMU-10)
- [x] Calibration magnétomètre
- [x] Réglage PID via interface web
- [ ] Fusion Yaw gyro/magnétomètre
- [ ] Mode Altitude Hold (baromètre)
- [ ] Intégration GPS
- [ ] Mode Return-To-Home
- [ ] Failsafe avancé

---

## Contributeurs

Projet développé dans le cadre du cursus **UMons 2025-2026**.

---

## Licence

Ce projet est distribué sous licence MIT. Voir le fichier `LICENSE` pour plus de détails.

---

<div align="center">

**[Documentation](docs/) · [Issues](../../issues) · [Discussions](../../discussions)**

</div>
