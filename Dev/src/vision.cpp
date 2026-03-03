// script responsable de toute la gestion de la vision (lancé dans un thread séparé)


// includes
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

#include "../includes/vision.h"
#include "../includes/color.h"
#include "../includes/ILidar.h"
#include "../includes/Slamtec.h"
#include "../includes/LidarProcessing.h"
#include "../includes/Board.hpp"

// namespaces
using namespace std::chrono;

// headers
void runLoop(std::atomic<bool>* stop);


std::unique_ptr<Slamtec> lidarTop;

void vision(std::atomic<bool>* stop){
    // lancement du module de vision
    lidarTop = std::make_unique<Slamtec>("/dev/ttyUSB0"); // TODO : remplacer chemin par constante

    if (lidarTop->connect()){ // si on à réussi à se connecter
        if (lidarTop->startScan()){ // si on à réussi à lancer le scan
            runLoop(stop);

            // une fois la boucle terminé on déconnecte le lidar
            lidarTop->disconnect();
        }
    }
}


void runLoop(std::atomic<bool>* stop) {
    Board& board = Board::instance();

    while (!*stop) {
        std::vector<ScanPoint> points;

        // grabData est bloquant, ne renvoie les points que quand il y à des nouveaux
        if (lidarTop->grabData(points)) {
            // Export CSV pour debug (peut être désactivé en production)
            LidarUtils::writeScanToCSV(points);

            // Récupérer la position et l'orientation actuelles de notre robot
            Robot myRobot = board.getMyRobot();
            double orientation = board.getMyRobotOrientation();

            // Lancer le pipeline de traitement LIDAR
            LidarProcessingResult result = LidarProcessing::processScan(
                points,
                myRobot.position,
                orientation
            );

            // Log de debug
            if (result.enemyDetected) {
                std::cout << YELLOW 
                    << "[LIDAR] Ennemi détecté à (" 
                    << result.enemyPosition.x_cm << ", " 
                    << result.enemyPosition.y_cm << ") cm"
                    << " | cluster: " << result.clusterSize << " pts"
                    << RESET << std::endl;
            }
        }
    }
}
