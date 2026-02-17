# Manuel de Pilotage & Procédures

> **Version:** 1.0
> **Contrôleur:** ESP32 Flight Controller
> **Date:** Janvier 2026

---

## Table des Matières

1. [Séquence de Démarrage (Boot)](#1-séquence-de-démarrage-boot)
2. [Armement & Désarmement](#2-armement--désarmement)
3. [Modes de Vol](#3-modes-de-vol)
4. [Sécurités & Failsafes](#4-sécurités--failsafes-critique)
5. [Calibration des ESC](#5-calibration-des-esc)
6. [Référence Rapide](#6-référence-rapide)

---

## 1. Séquence de Démarrage (Boot)

### 1.1 Branchement de la Batterie

Lorsque vous branchez la batterie LiPo, le contrôleur de vol exécute la séquence suivante :

#### Étape 1 : Attente du Signal Radio (0-15 secondes)

| Indicateur | Signification |
|------------|---------------|
| LED clignote **très rapidement** (50ms) | En attente d'un signal radio valide |

- Le système attend que le récepteur envoie un signal valide (Throttle > 900µs)
- **Timeout de sécurité** : Si aucun signal après 15 secondes, le système continue quand même
- Durant cette phase, les ESC reçoivent le signal maximum (2000µs) en préparation

#### Étape 2 : Décision Automatique

Le système détecte la position du stick de gaz :

| Position Gaz | Action |
|--------------|--------|
| **Gaz > 1900** (stick en haut) | → Mode Calibration ESC (voir Section 5) |
| **Gaz < 1900** (stick en bas) | → Séquence normale de calibration IMU |

---

### 1.2 Calibration Gyroscope/Accéléromètre (Phase 1)

> **CRITIQUE : NE PAS BOUGER LE DRONE PENDANT CETTE PHASE !**

| Durée | LED | Action Requise |
|-------|-----|----------------|
| 5 secondes (MPU6050) | Clignote rapidement (50ms on/off) | Drone **immobile** sur surface plane |
| 5 secondes (L3GD20) | Clignote rapidement (50ms on/off) | Drone **immobile** sur surface plane |

**Ce qui se passe :**
- Le système collecte ~2500 échantillons du gyroscope
- Ces mesures servent à calculer les offsets de calibration (drift au repos)
- Tout mouvement pendant cette phase **faussera la calibration**

**Après la Phase 1 :** LED éteinte pendant 1 seconde (pause visuelle)

---

### 1.3 Calibration Magnétomètre (Phase 2)

> **ACTION REQUISE : TOURNER LE DRONE DANS TOUS LES SENS**

| Durée | LED | Action Requise |
|-------|-----|----------------|
| 20 secondes | Clignote **lentement** (500ms on/off) | Faire pivoter le drone en Pitch, Roll et Yaw |

**Procédure recommandée :**
1. Tenir le drone en main
2. Effectuer des rotations lentes sur les 3 axes
3. Couvrir toutes les orientations possibles (360° sur chaque axe)
4. Le système enregistre les valeurs min/max pour calibrer le compas

---

### 1.4 Système Prêt (MODE_SAFE)

| Indicateur | Signification |
|------------|---------------|
| LED **fixe** (allumée en continu) | Drone prêt, moteurs désarmés |

Le drone est maintenant en mode **SAFE** :
- Tous les capteurs sont calibrés
- Les moteurs sont coupés
- Le système attend l'armement

---

## 2. Armement & Désarmement

### 2.1 Procédure d'Armement

> **DANGER : Avant d'armer, vérifiez que la zone est dégagée et que les hélices sont libres de tout obstacle.**

**Combinaison des sticks :**

```
    THROTTLE (Gaz)              YAW
         ^                       ^
         |                       |
    [  MIN  ] ←── Stick Bas     [ GAUCHE ] ←── Stick Gauche
         |                       |
         v                       v
```

| Stick | Position | Valeur |
|-------|----------|--------|
| **Throttle (CH3)** | Tout en bas | < 1010 |
| **Yaw (CH4)** | Tout à gauche | < 1200 |
| **Durée** | Maintenir | **1 seconde** |

**Séquence :**
1. Mettre le stick de gaz **tout en bas**
2. Mettre le stick de yaw **tout à gauche**
3. Maintenir cette position pendant **1 seconde**
4. Le drone passe en **MODE_ARMED**

---

### 2.2 Indication Visuelle de l'Armement

| Mode | Comportement LED |
|------|------------------|
| **MODE_SAFE** | LED fixe (allumée) |
| **MODE_ARMED** | LED clignote lentement (100ms ON / 400ms OFF) |
| **MODE_FLYING** | LED éteinte |

---

### 2.3 Désarmement Standard

**Combinaison des sticks :**

```
    THROTTLE (Gaz)              YAW
         ^                       ^
         |                       |
    [  MIN  ] ←── Stick Bas     [ DROITE ] ←── Stick Droite
         |                       |
         v                       v
```

| Stick | Position | Valeur |
|-------|----------|--------|
| **Throttle (CH3)** | Tout en bas | < 1010 |
| **Yaw (CH4)** | Tout à droite | > 1800 |
| **Durée** | Maintenir | **1 seconde** |

**Fonctionne depuis :** MODE_ARMED ou MODE_FLYING

---

### 2.4 Désarmement d'Urgence

> **URGENCE : Cette procédure coupe immédiatement les moteurs !**

La même combinaison (Gaz bas + Yaw droite pendant 1 seconde) fonctionne **même en vol**.

**En cas de crash ou situation dangereuse :**
1. **Gaz tout en bas** immédiatement
2. **Yaw tout à droite**
3. Maintenir **1 seconde**
4. Les moteurs se coupent, retour en MODE_SAFE

---

## 3. Modes de Vol

### 3.1 Définition des Modes

Le système utilise les modes suivants (définis dans `types.h`) :

| Mode | Description | Moteurs | LED |
|------|-------------|---------|-----|
| `MODE_SAFE` | Sécurité, moteurs coupés | OFF | Fixe |
| `MODE_ARMED` | Armé, prêt au décollage | Ralenti (1000µs) | Clignote lent |
| `MODE_FLYING` | Vol actif, PID actif | Actifs | Éteinte |
| `MODE_CALIBRATION` | Calibration ESC | Passthrough | Clignote lent |
| `MODE_WEB_TEST` | Test via interface WiFi | Manuel | - |

---

### 3.2 Mode Stabilisé (Comportement Actuel)

Le contrôleur utilise un **filtre de Kalman** pour la fusion gyroscope/accéléromètre :

- **Auto-nivellement** : Le drone tend à revenir à l'horizontale lorsque les sticks sont au neutre
- **Angle limité** : Les commandes pilote sont interprétées comme des angles cibles
- **Adaptatif** : Le filtre ajuste sa confiance selon le mode :
  - En MODE_SAFE avec gaz bas : Confiance accrue dans l'accéléromètre (R=0.01)
  - En vol : Confiance réduite dans l'accéléromètre (R=0.05)

---

### 3.3 Transition de Vol

| Condition | Transition |
|-----------|------------|
| MODE_ARMED + Throttle > 1040 | → MODE_FLYING |
| MODE_FLYING + Désarmement manuel | → MODE_SAFE |

---

### 3.4 Trim Accéléromètre

> **Note :** Les offsets de trim sont actuellement codés en dur dans `imu.cpp` :

```cpp
angle_roll_acc  += 0.8f;   // Compensation Roll
angle_pitch_acc += -3.7f;  // Compensation Pitch
```

**Pas de calibration en vol via sticks disponible dans cette version.**

Pour ajuster le trim :
1. Modifier les valeurs dans `imu.cpp` (lignes 255-256)
2. Recompiler et flasher le firmware

---

## 4. Sécurités & Failsafes (CRITIQUE)

### 4.1 Protection au Démarrage

| Protection | Description |
|------------|-------------|
| **Signal Radio Requis** | Le système attend un signal valide (CH3 > 900) avant de continuer |
| **Anti-Glitch S.BUS** | Les valeurs S.BUS < 50 sont rejetées (câble débranché/glitch) |
| **Armement Sécurisé** | Impossible d'armer si les gaz ne sont pas à zéro |

---

### 4.2 Protection Perte Radio

> **ATTENTION : Failsafe Radio Actif**

| Condition | Délai | Action |
|-----------|-------|--------|
| Throttle < 1010 en vol | **60 secondes** | Retour en MODE_ARMED |

**Comportement :**
- Si le signal de gaz reste à zéro pendant 60 secondes en MODE_FLYING
- Le système passe en MODE_ARMED (moteurs au ralenti)
- `error_code = 2` (PERTE RADIO)
- La LED clignote rapidement (50ms) pour signaler l'erreur

**Limitation actuelle :** Ce n'est pas un vrai failsafe de coupure radio, mais une détection de "gaz à zéro prolongé".

---

### 4.3 Détection de Crash par Angle

> **Note Importante :** La variable `angle_security_timer` est déclarée dans `main.cpp` mais **n'est pas utilisée** dans la version actuelle du code.

**Pas de coupure automatique des moteurs en cas d'angle excessif.**

---

### 4.4 Codes d'Erreur

| Code | Signification | Indication LED |
|------|---------------|----------------|
| 0 | OK, pas d'erreur | Comportement normal |
| 1 | CRASH ANGLE (non implémenté) | Clignotement rapide |
| 2 | PERTE RADIO | Clignotement rapide |

Quand `error_code > 0`, la LED clignote rapidement (25ms ON / 25ms OFF).

---

### 4.5 Limites de Sécurité (config.h)

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| `MAX_THROTTLE_FLIGHT` | 1700 | Gaz maximum en vol |
| `MIN_THROTTLE_IDLE` | 1050 | Gaz minimum (ralenti) |
| `MOTOR_OFF` | 1000 | Signal moteur coupé |
| `PID_MAX_ROLL` | 400 | Sortie PID max Roll |
| `PID_MAX_PITCH` | 400 | Sortie PID max Pitch |
| `PID_MAX_YAW` | 400 | Sortie PID max Yaw |

---

## 5. Calibration des ESC

> **DANGER : RETIRER TOUTES LES HÉLICES AVANT CETTE PROCÉDURE !**

### 5.1 Quand Calibrer les ESC ?

- Premiers ESC neufs
- Changement d'ESC
- Comportement moteur anormal (démarrage non synchronisé)

---

### 5.2 Procédure de Calibration

#### Étape 1 : Préparation

1. **RETIRER TOUTES LES HÉLICES**
2. Placer le drone sur un support stable
3. S'assurer que la radiocommande est allumée et connectée

#### Étape 2 : Entrée en Mode Calibration

1. **Mettre le stick de gaz TOUT EN HAUT** (position max)
2. **Brancher la batterie** en maintenant les gaz en haut
3. Le système détecte `channel_3 > 1900` et entre en `MODE_CALIBRATION`

#### Étape 3 : Séquence de Calibration

| Étape | Action | Résultat Attendu |
|-------|--------|------------------|
| 1 | Gaz en haut + Brancher batterie | LED clignote lentement (500ms) |
| 2 | Attendre les bips ESC (signal haut enregistré) | Bips courts des ESC |
| 3 | Mettre le stick de gaz **TOUT EN BAS** | Bips longs des ESC (signal bas enregistré) |
| 4 | Calibration terminée | ESC prêts |

---

### 5.3 Comportement en Mode Calibration

Le mode calibration agit en **passthrough** :
- Le signal Throttle (CH3) est copié directement vers les 4 moteurs
- Aucun mixage PID, aucune stabilisation
- Plage forcée entre 1000-2000µs

```cpp
// Extrait de esc_calibrate.cpp
int throt = drone->channel_3;
if(throt < 1000) throt = 1000;
if(throt > 2000) throt = 2000;
motors_write_direct(throt, throt, throt, throt);
```

---

### 5.4 Sortie du Mode Calibration

**Le mode calibration reste actif jusqu'au redémarrage.**

Pour revenir en mode normal :
1. Débrancher la batterie
2. Mettre les gaz en bas
3. Rebrancher la batterie

---

## 6. Référence Rapide

### 6.1 Carte des Commandes

```
╔═══════════════════════════════════════════════════════════════╗
║                    COMMANDES DES STICKS                       ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║   ARMEMENT:           Gaz BAS + Yaw GAUCHE (1 sec)            ║
║   DÉSARMEMENT:        Gaz BAS + Yaw DROITE (1 sec)            ║
║   DÉCOLLAGE:          Throttle > 1040 (depuis MODE_ARMED)     ║
║   CALIBRATION ESC:    Gaz HAUT + Brancher batterie            ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
```

### 6.2 Signification des LEDs

| Pattern LED | Signification |
|-------------|---------------|
| Clignotement très rapide (50ms) | Attente signal radio OU calibration gyro |
| Clignotement lent (500ms) | Calibration magnétomètre OU Mode ESC Calibration |
| Fixe (allumée) | MODE_SAFE - Prêt, moteurs désarmés |
| Clignotement court (100ms ON / 400ms OFF) | MODE_ARMED - Prêt au décollage |
| Éteinte | MODE_FLYING - En vol |
| Clignotement très rapide (25ms) | ERREUR (perte radio ou crash) |

### 6.3 Chronologie de Démarrage

```
[0s]     Batterie branchée
         │
         ▼
[0-15s]  Attente signal radio (LED rapide)
         │
         ├─── Gaz > 1900 → MODE CALIBRATION ESC
         │
         ▼
[15s]    Calibration Gyro MPU6050 (5s, LED rapide)
         │
         ▼
[20s]    Calibration Gyro L3GD20 (5s, LED rapide)
         │
         ▼
[25s]    Pause (1s, LED éteinte)
         │
         ▼
[26s]    Calibration Magnétomètre (20s, LED lente)
         │
         ▼
[46s]    PRÊT - MODE_SAFE (LED fixe)
```

---

## Avertissements de Sécurité

> **DANGER - HÉLICES**
> - Ne JAMAIS approcher les mains d'un drone armé
> - Toujours retirer les hélices pour les tests au sol
> - Vérifier l'état des hélices avant chaque vol

> **DANGER - BATTERIE**
> - Ne jamais laisser une LiPo sans surveillance en charge
> - Stocker les batteries dans un sac ignifugé
> - Ne pas utiliser une batterie gonflée ou endommagée

> **ATTENTION - CALIBRATION**
> - Ne pas bouger le drone pendant la calibration gyro
> - Effectuer la calibration sur une surface plane
> - Recalibrer après tout changement de configuration

---

*Document généré à partir de l'analyse du code source ESP32 Flight Controller.*
