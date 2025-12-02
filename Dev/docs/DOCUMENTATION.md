# 📚 Documentation - Projet WALL-II
## Coupe de France de Robotique

---

## 📁 Arborescence du Projet

```
Dev/
├── src/                          # Code source principal
│   ├── main.cpp                  # Point d'entrée du programme
│   ├── board.cpp                 # Gestion du plateau de jeu (Singleton)
│   ├── Slamtec.cpp              # Implémentation driver LIDAR Slamtec
│   ├── vision.cpp               # Module de vision (traitement LIDAR)
│   ├── strategy.cpp             # Module de stratégie (vide pour l'instant)
│   └── UnityDebug.cpp           # Communication avec Unity (debug visuel)
│
├── includes/                     # Fichiers d'en-tête
│   ├── Board.hpp                # Définition du singleton Board
│   ├── ILidar.h                 # Interface abstraite pour LIDAR
│   ├── Slamtec.h                # Header du driver Slamtec
│   ├── vision.h                 # Header du module vision
│   ├── Constant.h               # Constantes globales du projet
│   └── color.h                  # Codes ANSI pour couleurs terminal
│
├── build/                        # Fichiers compilés
│   ├── bin/                     # Exécutables
│   │   ├── wallII               # Version officielle (optimisée)
│   │   └── wallII_debug         # Version debug
│   └── obj/                     # Fichiers objets (.o)
│       └── official/
│
├── lib/                          # Bibliothèques externes
│   ├── bin/
│   │   └── libsl_lidar_sdk.a    # Bibliothèque statique LIDAR Slamtec
│   └── include/                 # Headers SDK LIDAR
│       ├── sl_lidar.h
│       ├── sl_lidar_driver.h
│       └── ...
│
├── data/                         # Données et configuration
│   └── start_map.json           # Configuration initiale du plateau
│
├── test_blink/                   # Tests Arduino
│   ├── test_blink.ino
│   └── workng.ino
│
├── makefile                      # Compilation du projet
├── lidar_data.csv               # Données LIDAR exportées
└── test.html                    # Visualisation web des données
```

---

## 🎯 Fonctionnement Global du Projet

### **Objectif**
Ce projet est le code embarqué d'un robot participant à la **Coupe de France de Robotique**. Le robot doit :
- Se déplacer sur un plateau de jeu de manière autonome
- Détecter son environnement via un capteur LIDAR
- Ramasser et déposer des éléments de jeu (cleats/palets)
- Éviter les collisions avec le robot adversaire

---

## 🏗️ Architecture du Code

### **1. Point d'entrée : `main.cpp`**

#### Responsabilités :
- Initialisation du système
- Création des threads principaux
- Gestion du timing de match (100 secondes)
- Bouton d'arrêt d'urgence
- Coordination des différents modules

#### Flux d'exécution :
```
1. Initialisation
   ├── Création du singleton Board
   ├── Chargement de la configuration (config.json)
   └── Création des threads

2. Threads lancés
   ├── Thread Vision (acquisition LIDAR)
   ├── Thread Input (arrêt d'urgence)
   └── Boucle principale (timing)

3. Boucle de match (100 secondes)
   ├── Mise à jour du temps
   ├── Vérification arrêt d'urgence
   └── Sleep 1ms (éviter surcharge CPU)

4. Fin de partie
   ├── Arrêt de tous les threads
   ├── Déconnexion LIDAR
   └── Attente de terminaison propre
```

---

### **2. Singleton Board : `Board.hpp` / `board.cpp`**

#### Concept :
Le **Board** est un **Singleton** (une seule instance pour tout le programme) qui centralise **toutes les données de l'état de jeu**.

#### Contenu :
```cpp
- Robot myRobot          // Notre robot
- Robot enemyRobot       // Robot adversaire
- Map map                // Plateau de jeu
  ├── storagesZones[8]   // Zones de ramassage
  ├── depositsZones[10]  // Zones de dépôt
  └── nid                // Zone centrale
- int timeLeft           // Temps restant (secondes)
- GameState state        // État du match
- Team myTeam            // Notre équipe (Orange/Blue)
```

#### Thread-Safety :
Toutes les fonctions utilisent un **mutex** pour garantir la sécurité en environnement multi-thread.

#### Méthodes principales :
- **Lecture** : `getMyRobot()`, `getMap()`, `getTimeLeft()`
- **Écriture** : `moveMyRobot()`, `setStorageZoneState()`, `setState()`
- **Validation** : `isInsideMap()` (vérification position valide)

---

### **3. Module Vision : `vision.cpp`**

#### Rôle :
- Acquérir les données du capteur LIDAR
- Traiter les informations de distance/angle
- Détecter obstacles et robots adverses

#### Architecture :
```
vision(stop)
  ├── Initialisation LIDAR (/dev/ttyUSB0)
  ├── Connexion au capteur
  ├── Démarrage du scan
  └── runLoop() ──┐
                  │
                  ├─> grabData() (bloquant)
                  ├─> Traitement des points
                  ├─> Exportation CSV (debug)
                  └─> Mise à jour Board (TODO)
```

#### Fonctionnement du LIDAR :
1. **grabData()** : Bloquant, attend de nouvelles données
2. **MAX_NODES** : 8192 points maximum par scan
3. **Format** : `{angle_deg, dist_mm}` pour chaque point
4. **Fréquence** : ~10 Hz (dépend du LIDAR)

---

### **4. Driver LIDAR : `Slamtec.cpp` / `ILidar.h`**

#### Pattern utilisé : **Interface abstraite**
```cpp
ILidar (interface)
  └── Slamtec (implémentation concrète)
```

#### Machine à états :
```
disconnected → connecting → ready → working → stopped
       ↓                       ↓         ↓
     error ←──────────────────┴─────────┘
```

#### Méthodes principales :
- `connect()` : Ouvre le port série, initialise le driver
- `startScan()` : Lance le scan continu
- `grabData()` : Récupère un scan complet (bloquant)
- `disconnect()` : Ferme proprement la connexion

#### Configuration :
- **Port** : `/dev/ttyUSB0` (par défaut)
- **Baudrate** : 115200
- **Timeout** : 0.5 secondes

---

### **5. Stratégie : `strategy.cpp`**

**État actuel : Non implémenté (fichier vide)**

#### Rôle prévu :
- Calcul du chemin optimal
- Décision des actions à effectuer
- Priorisation des objectifs
- Réaction aux événements (robot adversaire, temps)

---

### **6. Debug Unity : `UnityDebug.cpp`**

**État actuel : Stub (fonction vide)**

#### Rôle prévu :
- Conversion du Board en JSON
- Envoi des données à Unity
- Visualisation temps réel du match
- Debug visuel de la stratégie

---

## 🔧 Compilation et Utilisation

### **Makefile**

#### Cibles disponibles :
```bash
make official    # Compile version optimisée (-O2)
make clean       # Supprime fichiers objets
make distclean   # Supprime tout le dossier build
```

#### Dépendances :
- **Compilateur** : g++ (C++17)
- **Bibliothèque externe** : `libsl_lidar_sdk.a` (SDK Slamtec LIDAR)
- **Includes** : SDK LIDAR dans `/home/rapha/testLidar/rplidar_sdk/`

⚠️ **Note** : Les chemins dans le Makefile sont actuellement configurés pour l'utilisateur `rapha` et devront être adaptés.

---

## 📊 Structures de Données

### **Position**
```cpp
struct Position {
    double x;  // horizontal (en mètres)
    double y;  // vertical (en mètres)
}
```

### **Robot**
```cpp
struct Robot {
    Position position;
    Size size;
    std::array<Cleat, 4> cleatsHeld;  // Max 4 palets
}
```

### **Zone**
```cpp
struct Zone {
    Position position;
    Size size;
    ZoneState state;  // Empty/Filled/UsedByMe/UsedByEnemy
}
```

### **ScanPoint (LIDAR)**
```cpp
struct ScanPoint {
    float angle_deg;  // Angle en degrés (0-360)
    float dist_mm;    // Distance en millimètres
}
```

---

## ⚙️ Constantes Importantes

**Fichier : `Constant.h`**
```cpp
GAME_DURATION_MS = 100'000  // 100 secondes de match
```

**Fichier : `Slamtec.h`**
```cpp
BAUDRATE = 115200           // Vitesse communication LIDAR
MAX_NODES = 8192            // Points max par scan
SCAN_TIMEOUT = 0.5          // Timeout en secondes
```

---

## 🔄 Cycle de Vie du Programme

```
1. [INIT] Initialisation
   ├── Création Board singleton
   ├── Chargement configuration
   └── Création threads

2. [WAITING] Attente démarrage
   └── TODO: Attendre capteur de départ

3. [STARTED] Match en cours
   ├── Thread Vision : Acquisition LIDAR
   ├── Thread Stratégie : Décisions (TODO)
   ├── Boucle principale : Timing
   └── Mise à jour Board

4. [FINISHED] Fin de match
   ├── Arrêt threads
   ├── Déconnexion capteurs
   └── TODO: Mode standby
```

---

## 🚧 Points TODO Identifiés

### **Priorité Haute**
1. ⚠️ Adapter les chemins du Makefile (actuellement `/home/rapha/...`)
2. ⚠️ Implémenter `Board::initialiseData()` (lecture config.json)
3. ⚠️ Traitement des données LIDAR dans `vision.cpp`
4. ⚠️ Attente capteur de départ dans `main.cpp`

### **Priorité Moyenne**
5. 📌 Implémenter `strategy.cpp` (logique de jeu)
6. 📌 Communication avec Unity (`UnityDebug.cpp`)
7. 📌 Gestion des états de match dans Board
8. 📌 Détection robot adversaire via LIDAR

### **Priorité Basse**
9. 🔧 Ajout logs structurés
10. 🔧 Gestion erreurs robuste
11. 🔧 Tests unitaires

---

## 🛠️ Technologies Utilisées

| Composant | Technologie |
|-----------|-------------|
| Langage | C++17 |
| Build | GNU Make |
| Threading | std::thread, std::atomic |
| Capteur | LIDAR Slamtec (RPLidar SDK) |
| Communication | Port série (USB) |
| Debug | Export CSV + HTML (prévu Unity) |

---

## 📝 Conventions de Code

### **Nommage**
- **Classes** : `PascalCase` (ex: `Board`, `Slamtec`)
- **Fonctions** : `camelCase` (ex: `moveMyRobot()`)
- **Variables** : `camelCase` (ex: `timeLeft`)
- **Constantes** : `UPPER_SNAKE_CASE` (ex: `GAME_DURATION_MS`)
- **Enums** : `PascalCase` (ex: `GameState::Started`)

### **Commentaires**
- Sections importantes délimitées par `// ====...====`
- TODO marqués explicitement avec `//todo:` ou `//Todo:`
- Explications en français dans les commentaires

### **Couleurs Terminal**
Utilisation extensive de `color.h` pour feedback utilisateur :
- 🔴 **RED** : Erreurs
- 🟢 **GREEN** : Succès
- 🟡 **YELLOW** : Informations
- 🔵 **BOLDBLUE** : Messages importants
- 🟣 **MAGENTA** : Instructions utilisateur

---

## 🐛 Debugging

### **Export données LIDAR**
Les données sont automatiquement exportées dans `lidar_data.csv` :
```csv
angle_deg,dist_mm
0.12,1523.50
0.75,1520.25
...
```

### **Visualisation**
Utiliser `test.html` pour visualiser les données LIDAR dans un navigateur.

### **Logs Console**
Messages colorés selon la gravité :
- Connexion LIDAR
- État des threads
- Erreurs d'exécution
- Timing de match

---

## 📖 Ressources Externes

- **SDK LIDAR** : RPLidar SDK (Slamtec)
- **Documentation Coupe** : Règlement officiel Coupe de France de Robotique
- **Bibliothèque LIDAR** : `libsl_lidar_sdk.a` (compilée séparément)

---

## ✅ Points Forts du Projet

1. ✨ **Architecture modulaire** : Séparation claire des responsabilités
2. 🔒 **Thread-safe** : Utilisation de mutex pour Board
3. 🎨 **Interface abstraite** : ILidar permet de changer de capteur facilement
4. 📊 **Singleton Board** : État centralisé et cohérent
5. 🎯 **Pattern orienté objet** : Encapsulation, héritage, polymorphisme

---

## 🔮 Évolutions Futures

### **Court terme**
- Finaliser le module stratégie
- Implémenter détection obstacles
- Communication Arduino (moteurs)

### **Moyen terme**
- Intégration caméra (reconnaissance objets)
- SLAM (Simultaneous Localization And Mapping)
- Communication WiFi avec station de contrôle

### **Long terme**
- IA/Machine Learning pour optimisation stratégie
- Simulation complète avant tournoi
- Rejeu de match (logs détaillés)

---

**Dernière mise à jour** : 22 novembre 2025  
**Version du projet** : WALL-II (Coupe de France de Robotique)  
**Auteurs** : Équipe WALL-II
