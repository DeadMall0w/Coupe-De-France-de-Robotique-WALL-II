// Thread caméra : acquisition vidéo + traitement + publication de l'ordre dans Board
// Fonctionne en parallèle du thread LIDAR (vision.cpp)

#include <iostream>
#include <atomic>
#include <string>

#include "../includes/camera.h"
#include "../includes/CameraProcessing.h"
#include "../includes/Board.hpp"
#include "../includes/color.h"

// Identifiant de la caméra
// -1 = RPi Camera CSI via GStreamer (libcamerasrc) — défaut pour le robot
//  0 = caméra USB classique via V4L2 (/dev/video0)
static constexpr int CAMERA_ID = -1;

// Conversion enum → chaîne pour le log
static const char* orderToString(CameraOrder order) {
    switch (order) {
        case CameraOrder::Recherche:         return "RECHERCHE";
        case CameraOrder::Avancer:           return "AVANCER";
        case CameraOrder::Reculer:           return "RECULER";
        case CameraOrder::Gauche:            return "GAUCHE";
        case CameraOrder::Droite:            return "DROITE";
        case CameraOrder::TourneHoraire:     return "TOURNE HORAIRE";
        case CameraOrder::TourneAntiHoraire: return "TOURNE ANTI-HORAIRE";
        case CameraOrder::Parfait:           return "PARFAIT";
        default:                             return "INCONNU";
    }
}

static const char* colorToString(DetectedColor c) {
    switch (c) {
        case DetectedColor::Yellow: return "JAUNE";
        case DetectedColor::Blue:   return "BLEU";
        default:                    return "?";
    }
}

void camera(std::atomic<bool>* stop) {
    std::cout << BOLDBLUE << "[Camera] Démarrage du thread caméra..." << RESET << std::endl;

    CameraProcessing camProc;

    if (!camProc.init(CAMERA_ID)) {
        std::cerr << RED << "[Camera] Impossible d'initialiser la caméra, arrêt du thread." << RESET << std::endl;
        return;
    }

    Board& board = Board::instance();

    while (!*stop) {
        CameraResult result = camProc.processNextFrame();

        // Publier le résultat dans le Board (accessible par strategy, communication, etc.)
        board.setCameraResult(result);

        // Log de debug
        if (result.objectDetected) {
            std::cout << CYAN
                << "[Camera] " << result.crates.size() << " tasseau(x) | "
                << orderToString(result.groupOrder)
                << " | Grp Z:" << (int)result.groupCenterZ
                << " X:" << (int)result.groupCenterX
                << " cap:" << (int)result.groupAngle
                << RESET << std::endl;
        }

        // La boucle tourne aussi vite que la caméra fournit des images (~30 fps)
        // Pas de sleep nécessaire : cap >> frame est bloquant
    }

    camProc.release();
    std::cout << BOLDBLUE << "[Camera] Thread caméra terminé." << RESET << std::endl;
}
