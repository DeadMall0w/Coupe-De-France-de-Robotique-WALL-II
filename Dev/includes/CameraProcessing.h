#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

// ================================
// Ordres caméra vers Teensy
// ================================

// Enum représentant les ordres déduits par la caméra
enum class CameraOrder {
    Recherche,           // aucun objet détecté, en recherche
    Avancer,             // robot trop loin de la cible → avancer
    Reculer,             // robot trop proche → reculer
    Gauche,              // objet à droite du centre → décaler à gauche
    Droite,              // objet à gauche du centre → décaler à droite
    TourneHoraire,       // pivoter dans le sens horaire pour s'aligner avec le tasseau
    TourneAntiHoraire,   // pivoter dans le sens anti-horaire
    Parfait              // distance, centrage et alignement OK → prêt pour la prise
};

// Couleur détectée
enum class DetectedColor {
    None,
    Yellow,
    Blue
};

// Résultat complet du traitement caméra
struct CameraResult {
    CameraOrder order = CameraOrder::Recherche;
    bool objectDetected = false;
    DetectedColor color = DetectedColor::None;  // couleur du tasseau détecté
    double smoothZ  = 0.0;      // distance lissée (cm) — calculée via dimensions connues du tasseau
    double smoothX  = 0.0;      // décalage latéral lissé (cm)
    double smoothAngle = 0.0;   // angle du tasseau lissé (degrés, 0 = vertical)
    double smoothRatio = 0.0;   // ratio long/court lissé
};

// ================================
// Classe de traitement caméra
// ================================
// Détecte des tasseaux colorés (15×4×3 cm, jaune ou bleu)
// en exploitant leurs dimensions connues pour :
//   - estimer la distance (taille réelle → pixels)
//   - filtrer les faux positifs (ratio d'aspect attendu)
//   - déterminer l'orientation (minAreaRect)

class CameraProcessing {
public:
    CameraProcessing();
    ~CameraProcessing();

    // Initialise la caméra (retourne true si succès)
    // cameraId < 0 → GStreamer (RPi Camera CSI)
    // cameraId >= 0 → V4L2 (/dev/videoX)
    bool init(int cameraId = -1);

    // Libère la caméra
    void release();

    // Traite une frame et retourne le résultat (ordre + données)
    CameraResult processNextFrame();

    // Vérifie si la caméra est ouverte
    bool isOpened() const;

private:
    cv::VideoCapture cap;

    // Éléments structurants pré-calculés
    cv::Mat kernelOpen;
    cv::Mat kernelClose;

    // État du lissage EMA
    double smooth_z     = 0.0;
    double smooth_x     = 0.0;
    double smooth_r     = 0.0;
    double smooth_angle = 0.0;
    bool firstDetection = true;
};
