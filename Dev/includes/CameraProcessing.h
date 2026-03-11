#pragma once

#include <string>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

// ================================
// Ordres caméra vers Teensy
// ================================

enum class CameraOrder {
    Recherche,           // aucun tag détecté
    Avancer,             // trop loin de la cible → avancer
    Reculer,             // trop proche → reculer
    Gauche,              // caisse décalée à droite → déplacer à gauche
    Droite,              // caisse décalée à gauche → déplacer à droite
    TourneHoraire,       // cap positif → pivoter horaire
    TourneAntiHoraire,   // cap négatif → pivoter anti-horaire
    Parfait              // distance, centrage et alignement OK
};

// Zone identifiée par le tag ArUco
enum class DetectedColor {
    None,
    Yellow,   // tag ID 47
    Blue      // tag ID 36
};

// ================================
// Infos d'une caisse détectée
// ================================
struct DetectedCrate {
    int           tagId          = -1;
    DetectedColor color          = DetectedColor::None;
    CameraOrder   order          = CameraOrder::Recherche;
    double        smoothZ        = 0.0;   // distance avant lissée (mm)
    double        smoothX        = 0.0;   // décalage latéral lissé (mm, + = droite)
    double        smoothAngle    = 0.0;   // cap de la caisse lissé (degrés)
    double        smoothRatio    = 0.0;   // taille tag en pixels / TAG_SIZE_MM (échelle)
};

// Résultat complet du traitement caméra (multi-tasseaux)
struct CameraResult {
    bool                       objectDetected = false;   // au moins un tasseau détecté
    std::vector<DetectedCrate> crates;                   // tous les tasseaux détectés

    // Ordre global basé sur le CENTRE DU GROUPE (pour approcher le bloc de 4 tasseaux)
    CameraOrder groupOrder    = CameraOrder::Recherche;
    double      groupCenterZ  = 0.0;   // distance moyenne du groupe (mm)
    double      groupCenterX  = 0.0;   // décalage latéral moyen du groupe (mm)
    double      groupAngle    = 0.0;   // orientation moyenne du groupe (degrés)

    // Accès rapide : le tasseau le plus proche (plus grand tag en pixels)
    // Retourne nullptr si aucun tasseau détecté
    const DetectedCrate* closest() const {
        if (crates.empty()) return nullptr;
        const DetectedCrate* best = &crates[0];
        for (const auto& c : crates)
            if (c.smoothRatio > best->smoothRatio) best = &c;
        return best;
    }

    // Nombre de tasseaux d'une couleur donnée
    int countByColor(DetectedColor col) const {
        int n = 0;
        for (const auto& c : crates) if (c.color == col) ++n;
        return n;
    }
};

// ================================
// Classe de traitement caméra
// ================================
// Détecte les caisses de noisettes (150×50×30 mm) par tags ArUco 4×4 :
//   - ID 36 → zone bleue
//   - ID 47 → zone jaune
// Supporte la détection simultanée de plusieurs caisses (jusqu'à 4+).
// Modèle géométrique plan sagittal :
//   α = pitch + atan((py − cy) / focal)
//   Z = (h_cam − h_tag) / tan(α)          [mm]
//   X = Z * (px − cx) / focal             [mm]

class CameraProcessing {
public:
    CameraProcessing();
    ~CameraProcessing();

    // Initialise la caméra (retourne true si succès)
    // cameraId < 0 → GStreamer (RPi Camera CSI)
    // cameraId >= 0 → V4L2 (/dev/videoX)
    bool init(int cameraId = -1);

    void release();
    CameraResult processNextFrame();
    bool isOpened() const;

    void    setDebug(bool enabled);
    cv::Mat getDebugFrame() const;

private:
    cv::VideoCapture cap;

    // Détecteur ArUco
    cv::aruco::Dictionary        arucoDict;
    cv::aruco::DetectorParameters arucoParams;
    cv::aruco::ArucoDetector     detector;

    bool    debugEnabled = false;
    cv::Mat debugFrame;

    // État du lissage EMA par tag ID (permet de suivre chaque caisse indépendamment)
    struct EmaState {
        double z     = 0.0;
        double x     = 0.0;
        double r     = 0.0;
        double angle = 0.0;
        bool   first = true;
    };
    std::map<int, EmaState> emaStates;
};
