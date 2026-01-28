### RPI

Thread 1: LIDAR Vision
  └─> Détection adversaire → Board (shared state)

Thread 2: Décisionnel (Strategy)
  ├─> Lit état plateau (Board) et décide action suivante
  └─> Envoie ordres aux µC

Thread 3: Communication Chassis UART
  ├─> Envoie consignes (x, y, θ, vitesse)
  └─> Reçoit position odométrique

Thread 4: Communication Corps UART
  ├─> Envoie commandes actions
  └─> Reçoit états servos/capteurs

Thread 5: Timing Match
  └─> Gestion 100s


### µC Chassis

Loop principale (1kHz):
  ├─> Lit encodeurs
  ├─> Calcule position (x, y, θ)
  ├─> Contrôle PID moteurs
  ├─> Envoie position au RPI
  └─> Reçoit consignes RPI


### µC Corps

Loop principale (100Hz):
  ├─> Reçoit ordres RPI
  ├─> Exécute séquences servos
  ├─> Traite image caméra (couleur)
  ├─> Lit capteurs états
  └─> Envoie statuts au RPI