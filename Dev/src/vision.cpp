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

// namespaces
using namespace std::chrono;

// headers
void runLoop(std::atomic<bool>* stop);


std::unique_ptr<Slamtec> lidarTop;

void vision(std::atomic<bool>* stop){
    // lancement du module de vision
    lidarTop = std::make_unique<Slamtec>("/dev/ttyUSB0");

    if (lidarTop->connect()){ // si on à réussi à se connecter
        if (lidarTop->startScan()){ // si on à réussi à lancer le scan
            runLoop(stop); // 10 fois par secondes


            // une fois la boucle terminé on déconnecte le lidar
            lidarTop->disconnect();

        }
    }

    

}


void runLoop(std::atomic<bool>* stop) {
    while (!*stop) {
        std::vector<ScanPoint> points;

        // grabData est bloquant, ne renvoie les points que quand il y à des nouveaux
        if (lidarTop->grabData(points)) {
            LidarUtils::writeScanToCSV(points);
            //Todo : traitement des données du Lidar
        }
    }
}
