// Test standalone du module caméra
// Compile : g++ -std=c++17 -O2 test_camera.cpp ../../Dev/src/CameraProcessing.cpp -I../../Dev/includes -I../../Dev/lib/include $(pkg-config --cflags --libs opencv4) -o test_camera
// Usage  : ./test_camera [gstreamer|v4l2] [--no-display]

#include <iostream>
#include <chrono>
#include <string>
#include <opencv2/highgui.hpp>
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
    bool showDisplay = true;

    for (int i = 1; i < argc; i++) {
        string arg(argv[i]);
        if (arg == "v4l2")        useGstreamer = false;
        if (arg == "--no-display") showDisplay = false;
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

    // Activer le mode debug (annotations sur la frame)
    camProc.setDebug(showDisplay);

    if (showDisplay) {
        cout << GREEN << "Fenêtre debug activée (appuyez sur 'q' ou ESC pour quitter)" << RESET << endl;
    }

    cout << GREEN << "Caméra ouverte ! Traitement en cours..." << RESET << endl;

    int frameCount = 0;
    auto startTime = steady_clock::now();

    while (true) {
        auto t1 = steady_clock::now();
        CameraResult result = camProc.processNextFrame();
        auto t2 = steady_clock::now();

        double ms = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
        frameCount++;

        // Affichage visuel debug
        if (showDisplay) {
            cv::Mat dbg = camProc.getDebugFrame();
            if (!dbg.empty()) {
                cv::imshow("WALL-II Camera Debug", dbg);
                int key = cv::waitKey(1) & 0xFF;
                if (key == 'q' || key == 27) {  // 'q' ou ESC
                    cout << YELLOW << "Arrêt demandé par l'utilisateur." << RESET << endl;
                    break;
                }
            }
        }

        // Afficher en console toutes les 30 frames
        if (frameCount % 30 == 0) {
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

    if (showDisplay) {
        cv::destroyAllWindows();
    }

    camProc.release();
    return 0;
}
