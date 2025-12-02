#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

// --- Paramètres ---
// Utilisation de UART0 sur Raspberry Pi 5
// UART0: GPIO 14 (TXD) = Pin 8, GPIO 15 (RXD) = Pin 10
#define SERIAL_PORT "/dev/serial0" 
#define BAUD_RATE 9600  // Vitesse compatible avec Arduino (9600 bauds)
#define DELAY_MS 50 // Délai entre chaque envoi (en millisecondes)

// Configuration du terminal pour lecture clavier non-bloquante
struct termios orig_termios;

void disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enableRawMode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disableRawMode);
    
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

int main()
{
    int fd; // Descripteur de fichier pour le port série

    // 1. Initialisation de WiringPi
    printf("Initialisation de WiringPi...\n");
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "Erreur : Échec de l'initialisation de WiringPi.\n");
        return 1;
    }

    // 2. Ouverture du port série
    printf("Ouverture du port série %s à %d bauds...\n", SERIAL_PORT, BAUD_RATE);
    if ((fd = serialOpen(SERIAL_PORT, BAUD_RATE)) < 0) {
        fprintf(stderr, "Erreur : Échec de l'ouverture du port série. Assurez-vous d'utiliser 'sudo'.\n");
        return 1;
    }

    printf("Port série ouvert avec succès. Descripteur : %d\n", fd);
    printf("Envoi des données de direction à %d bauds...\n", BAUD_RATE);
    printf("Format: dX,dY\\n (valeurs entre -1.0 et 1.0)\n\n");
    
    // 3. Activation du mode clavier non-bloquant
    enableRawMode();
    
    printf("=== CONTRÔLE CLAVIER ZQSD ACTIVÉ ===\n");
    printf("Z = Avancer      S = Reculer\n");
    printf("Q = Gauche       D = Droite\n");
    printf("E = Avant-Droite R = Avant-Gauche\n");
    printf("X = Stop         ESC = Quitter\n");
    printf("=====================================\n\n");

    // --- Boucle de transmission (infinie) ---
    double dX = 0.0, dY = 0.0;
    double lastDX = 0.0, lastDY = 0.0;
    char buffer[32];
    char c;
    int running = 1;
    int frameCount = 0;
    
    printf("[DEBUG] Début de la boucle principale\n");
    
    while (running)
    {
        frameCount++;
        
        // Lire le clavier (non-bloquant)
        c = getchar();
        
        // Debug : afficher le caractère lu (si valide)
        if (c != -1 && c != 255) {
            printf("[DEBUG] Touche détectée: '%c' (code: %d)\n", c, (int)c);
        }
        
        // Garder les dernières valeurs par défaut
        dX = lastDX;
        dY = lastDY;
        
        // Traiter les touches
        switch(c) {
            // Z = Avancer
            case 'z':
            case 'Z':
                dY = 1.0;
                dX = 0.0;
                printf("[COMMANDE] ↑ AVANCER (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // S = Reculer
            case 's':
            case 'S':
                dY = -1.0;
                dX = 0.0;
                printf("[COMMANDE] ↓ RECULER (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // Q = Gauche
            case 'q':
            case 'Q':
                dX = -1.0;
                dY = 0.0;
                printf("[COMMANDE] ← GAUCHE (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // D = Droite
            case 'd':
            case 'D':
                dX = 1.0;
                dY = 0.0;
                printf("[COMMANDE] → DROITE (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // E = Avant-Droite (diagonale)
            case 'e':
            case 'E':
                dX = 0.8;
                dY = 0.8;
                printf("[COMMANDE] ↗ AVANT-DROITE (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // R = Avant-Gauche (diagonale)
            case 'r':
            case 'R':
                dX = -0.8;
                dY = 0.8;
                printf("[COMMANDE] ↖ AVANT-GAUCHE (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // C = Arrière-Droite (diagonale)
            case 'c':
            case 'C':
                dX = 0.8;
                dY = -0.8;
                printf("[COMMANDE] ↘ ARRIÈRE-DROITE (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // V = Arrière-Gauche (diagonale)
            case 'v':
            case 'V':
                dX = -0.8;
                dY = -0.8;
                printf("[COMMANDE] ↙ ARRIÈRE-GAUCHE (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // X = Stop
            case 'x':
            case 'X':
            case ' ':  // Espace aussi pour stop
                dX = 0.0;
                dY = 0.0;
                printf("[COMMANDE] ⏸ STOP (dX=%.2f, dY=%.2f)\n", dX, dY);
                break;
            
            // ESC = Quitter
            case 27:  // Code ESC
                printf("\n[DEBUG] === ARRÊT DU PROGRAMME ===\n");
                running = 0;
                dX = 0.0;
                dY = 0.0;
                break;
            
            // Pas de touche ou touche inconnue = garder dernière valeur
            default:
                // Ne rien afficher si pas de touche (éviter spam)
                break;
        }
        
        // Sauvegarder les valeurs pour la prochaine itération
        lastDX = dX;
        lastDY = dY;
        
        // Format : "dX,dY\n" - exemple : "0.50,-0.75\n"
        snprintf(buffer, sizeof(buffer), "%.2f,%.2f\n", dX, dY);
        serialPuts(fd, buffer);
        
        // Debug périodique (toutes les 20 frames = ~1 sec)
        if (frameCount % 20 == 0) {
            printf("[DEBUG] Frame %d - Envoyé: dX=%.2f, dY=%.2f\n", frameCount, dX, dY);
        }
        
        // Pause entre chaque envoi
        delay(DELAY_MS);
    }

    // Envoyer un dernier stop avant de quitter
    printf("[DEBUG] Envoi commande STOP finale...\n");
    snprintf(buffer, sizeof(buffer), "0.00,0.00\n");
    serialPuts(fd, buffer);
    delay(100);
    
    // 5. Fermeture du port
    printf("[DEBUG] Fermeture du port série...\n");
    serialClose(fd);
    
    printf("[DEBUG] Port série fermé. Au revoir !\n");
    
    return 0;
}