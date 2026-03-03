// Test standalone du module caméra
// Compile : g++ -std=c++17 -O2 test_camera.cpp ../../Dev/src/CameraProcessing.cpp -I../../Dev/includes -I../../Dev/lib/include $(pkg-config --cflags --libs opencv4) -o test_camera
// Usage  : ./test_camera [gstreamer|v4l2]

#include <iostream>
#include <chrono>
#include <string>
#include "../../Dev/includes/CameraProcessing.h"
#include "../../Dev/includes/color.h"

using namespace std;
using namespace std::chrono;

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
        default:                    return "AUCUN";
    }
}

int main(int argc, char* argv[]) {
    cout << BOLDBLUE << "=== Test module caméra WALL-II ===" << RESET << endl;
    cout << YELLOW << "Objets recherchés : tasseaux 15×4×3 cm (jaune / bleu)" << RESET << endl;

    CameraProcessing camProc;

    bool useGstreamer = true;
    if (argc > 1 && string(argv[1]) == "v4l2") {
        useGstreamer = false;
    }

    bool ok;
    if (useGstreamer) {
        cout << YELLOW << "Mode : GStreamer (libcamerasrc)" << RESET << endl;
        ok = camProc.init(-1);
    } else {
        cout << YELLOW << "Mode : V4L2 (/dev/video0)" << RESET << endl;
        ok = camProc.init(0);
    }

    if (!ok) {
        cerr << RED << "Échec d'ouverture de la caméra." << RESET << endl;
        cerr << YELLOW << "Vérifiez avec : libcamera-hello --list-cameras" << RESET << endl;
        return 1;
    }

    cout << GREEN << "Caméra ouverte ! Traitement en cours (Ctrl+C pour arrêter)..." << RESET << endl;

    int frameCount = 0;
    auto startTime = steady_clock::now();

    while (true) {
        auto t1 = steady_clock::now();
        CameraResult result = camProc.processNextFrame();
        auto t2 = steady_clock::now();

        double ms = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
        frameCount++;

        // Afficher toutes les 10 frames
        if (frameCount % 10 == 0) {
            double elapsed = duration_cast<seconds>(t2 - startTime).count();
            double fps = (elapsed > 0) ? frameCount / elapsed : 0;

            if (result.objectDetected) {
                cout << GREEN << "[Frame " << frameCount << "] "
                     << BOLDGREEN << colorToString(result.color) << " " << orderToString(result.order) << RESET
                     << GREEN
                     << " | Z=" << result.smoothZ << "cm"
                     << " X=" << result.smoothX << "cm"
                     << " Ang=" << result.smoothAngle << "°"
                     << " R=" << result.smoothRatio
                     << " | " << ms << "ms | " << fps << " fps"
                     << RESET << endl;
            } else {
                cout << YELLOW << "[Frame " << frameCount << "] "
                     << "RECHERCHE... | " << ms << "ms | " << fps << " fps"
                     << RESET << endl;
            }
        }
    }

    camProc.release();
    return 0;
}
