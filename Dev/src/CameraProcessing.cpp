// Traitement caméra : détection de tasseaux colorés (15×4×3 cm, jaune/bleu)
// posés à plat au sol, vus par une caméra montée à l'horizontale, 
// légèrement inclinée vers l'avant.
//
// Méthode :
//   - Distance : projection au sol (hauteur caméra + horizon)
//   - Forme   : minAreaRect sur les contours, filtrage par ratio
//   - Angle   : orientation du tasseau au sol (dans le plan horizontal)
//   - Couleur : masques HSV séparés jaune / bleu

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#include "../includes/CameraProcessing.h"
#include "../includes/Constant.h"
#include "../includes/color.h"

using namespace cv;
using namespace std;

// ==========================================
// CONSTANTES — À calibrer sur le robot
// ==========================================

// Optique / Géométrie caméra
static constexpr double FOCAL_LENGTH = 600.0;   // longueur focale estimée (pixels)
static constexpr double CAM_HEIGHT_CM = 10.0;   // hauteur de la caméra par rapport au sol (cm)
static constexpr double HORIZON_Y    = 240.0;   // ligne d'horizon dans l'image (pixels, moitié de 480)
// Note : HORIZON_Y correspond au point de fuite. Les objets au sol
// apparaissent SOUS cette ligne. Plus un objet est bas dans l'image,
// plus il est proche.

// Dimensions physiques du tasseau (cm)
// Toujours posé à plat : le côté de 3 cm est la hauteur hors du sol
static constexpr double TASSEAU_LONG  = 15.0;   // côté long (visible si tasseau perpendiculaire)
static constexpr double TASSEAU_SHORT = 4.0;    // côté court (profondeur vue de face)
static constexpr double TASSEAU_HEIGHT = 3.0;   // épaisseur (hauteur hors du sol)

// Ratio d'aspect visible du tasseau dans l'image
// De face (long côté perpendiculaire à la caméra) : ~15/3 = 5.0
// De bout (court côté face à la caméra) : ~4/3 = 1.33
// On accepte une plage large car la perspective déforme
static constexpr double RATIO_MIN = 1.2;
static constexpr double RATIO_MAX = 8.0;

// Cible et tolérances pour les ordres
static constexpr double TARGET_Z  = 30.0;  // distance cible pour prise (cm)
static constexpr double TOL_Z     = 5.0;   // tolérance distance (cm)
static constexpr double TOL_X     = 4.0;   // tolérance latérale (cm)
static constexpr double ANGLE_TOL = 10.0;  // tolérance d'alignement au sol (degrés)
// 0° = tasseau perpendiculaire à l'axe de la caméra (long côté horizontal dans l'image)
// L'objectif est d'aligner le robot pour que le préhenseur soit parallèle au tasseau

// Lissage EMA
static constexpr double ALPHA = 0.2;

// Plages de couleurs HSV (à affiner sur le terrain)
static const Scalar LOWER_YELLOW(15, 100, 100);
static const Scalar UPPER_YELLOW(35, 255, 255);
static const Scalar LOWER_BLUE(90, 60, 50);
static const Scalar UPPER_BLUE(130, 255, 255);

// Surface minimale d'un contour valide (pixels²)
static constexpr double MIN_CONTOUR_AREA = 120.0;


// ================================
// Constructeur / Destructeur
// ================================

CameraProcessing::CameraProcessing() {
    kernelOpen  = getStructuringElement(MORPH_RECT, Size(3, 3));
    kernelClose = getStructuringElement(MORPH_RECT, Size(15, 15));
}

CameraProcessing::~CameraProcessing() {
    release();
}

// ================================
// Initialisation / Libération
// ================================

// Pipelines GStreamer pour RPi Camera Module (CSI) sur RPi5
// Essayés dans l'ordre jusqu'à ce qu'un fonctionne.
// Le format NV12 est le natif de la plupart des capteurs RPi Camera.
static const vector<pair<string, string>> GST_PIPELINES = {
    // 1. libcamerasrc avec format natif NV12 + capsfilter explicite
    { "libcamerasrc (NV12)",
      "libcamerasrc ! capsfilter caps=video/x-raw,format=NV12,width=640,height=480,framerate=30/1 ! "
      "videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true" },

    // 2. libcamerasrc auto-négociation (certaines versions)
    { "libcamerasrc (auto)",
      "libcamerasrc ! video/x-raw,width=640,height=480 ! "
      "videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true" },

    // 3. rpicam-vid en mode TCP vers OpenCV (si gstreamer libcamerasrc absent)
    { "rpicam-vid (pipe)",
      "rpicam-vid -t 0 --width 640 --height 480 --framerate 30 --codec mjpeg -o - ! "
      "jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true" },
};

bool CameraProcessing::init(int cameraId) {
    if (cameraId < 0) {
        // Mode RPi Camera CSI : essayer les pipelines GStreamer un par un
        for (const auto& [name, pipeline] : GST_PIPELINES) {
            cout << YELLOW << "[Camera] Essai pipeline : " << name << "..." << RESET << endl;
            cap.open(pipeline, CAP_GSTREAMER);

            if (cap.isOpened()) {
                // Tester qu'on récupère bien une frame
                Mat testFrame;
                cap >> testFrame;
                if (!testFrame.empty()) {
                    cout << GREEN << "[Camera] OK avec : " << name 
                         << " (" << testFrame.cols << "x" << testFrame.rows << ")"
                         << RESET << endl;
                    return true;
                }
                // Frame vide → ce pipeline ne marche pas vraiment
                cap.release();
                cerr << YELLOW << "[Camera] Pipeline " << name 
                     << " ouvert mais frames vides, essai suivant..." << RESET << endl;
            }
        }

        // Dernier recours : essayer V4L2 sur /dev/video0
        // (fonctionne si le module bcm2835-v4l2 est chargé)
        cout << YELLOW << "[Camera] Aucun pipeline GStreamer OK, essai V4L2 /dev/video0..." << RESET << endl;
        cap.open(0, CAP_V4L2);
        if (cap.isOpened()) {
            cap.set(CAP_PROP_FRAME_WIDTH, 640);
            cap.set(CAP_PROP_FRAME_HEIGHT, 480);
            Mat testFrame;
            cap >> testFrame;
            if (!testFrame.empty()) {
                cout << GREEN << "[Camera] OK avec V4L2 /dev/video0" << RESET << endl;
                return true;
            }
            cap.release();
        }

        cerr << RED << "[Camera] Erreur : aucune méthode n'a fonctionné" << RESET << endl;
        cerr << RED << "[Camera] Diagnostic :" << RESET << endl;
        cerr << RED << "  1. Vérifiez la caméra : rpicam-hello --list-cameras" << RESET << endl;
        cerr << RED << "  2. Vérifiez GStreamer : gst-inspect-1.0 libcamerasrc" << RESET << endl;
        cerr << RED << "  3. Installez si nécessaire : sudo apt install gstreamer1.0-libcamera" << RESET << endl;
        return false;

    } else {
        // Mode V4L2 explicite (USB ou /dev/videoX)
        cout << YELLOW << "[Camera] Ouverture V4L2 (ID: " << cameraId << ")..." << RESET << endl;
        cap.open(cameraId, CAP_V4L2);

        if (!cap.isOpened()) {
            cerr << RED << "[Camera] Erreur : impossible d'ouvrir /dev/video" << cameraId << RESET << endl;
            return false;
        }
        cout << GREEN << "[Camera] Caméra V4L2 ouverte (ID: " << cameraId << ")" << RESET << endl;
        return true;
    }
}

void CameraProcessing::release() {
    if (cap.isOpened()) {
        cap.release();
        cout << YELLOW << "[Camera] Caméra libérée" << RESET << endl;
    }
}

bool CameraProcessing::isOpened() const {
    return cap.isOpened();
}

void CameraProcessing::setDebug(bool enabled) {
    debugEnabled = enabled;
}

Mat CameraProcessing::getDebugFrame() const {
    return debugFrame;
}

// Conversion ordre → texte pour affichage debug
static const char* orderStr(CameraOrder o) {
    switch (o) {
        case CameraOrder::Recherche:         return "RECHERCHE";
        case CameraOrder::Avancer:           return "AVANCER";
        case CameraOrder::Reculer:           return "RECULER";
        case CameraOrder::Gauche:            return "GAUCHE";
        case CameraOrder::Droite:            return "DROITE";
        case CameraOrder::TourneHoraire:     return "TOURNE ->";
        case CameraOrder::TourneAntiHoraire: return "TOURNE <-";
        case CameraOrder::Parfait:           return "PARFAIT";
        default:                             return "?";
    }
}

// ================================
// Traitement d'une frame
// ================================

CameraResult CameraProcessing::processNextFrame() {
    CameraResult result;

    Mat frame, hsv;
    cap >> frame;
    if (frame.empty()) {
        cerr << RED << "[Camera] Erreur : frame vide" << RESET << endl;
        return result;
    }

    double cx = frame.cols / 2.0;

    // Copie pour l'affichage debug (avant blur)
    if (debugEnabled) {
        debugFrame = frame.clone();
    }

    // --- Pré-traitement ---
    GaussianBlur(frame, frame, Size(5, 5), 0);
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // --- Masques couleur séparés ---
    Mat maskYellow, maskBlue;
    inRange(hsv, LOWER_YELLOW, UPPER_YELLOW, maskYellow);
    inRange(hsv, LOWER_BLUE, UPPER_BLUE, maskBlue);

    // Morphologie sur chaque masque
    morphologyEx(maskYellow, maskYellow, MORPH_OPEN, kernelOpen);
    morphologyEx(maskYellow, maskYellow, MORPH_CLOSE, kernelClose);
    morphologyEx(maskBlue, maskBlue, MORPH_OPEN, kernelOpen);
    morphologyEx(maskBlue, maskBlue, MORPH_CLOSE, kernelClose);

    // --- Structure pour un candidat tasseau ---
    struct Candidate {
        RotatedRect rotRect;
        double longSide;       // côté le plus long (pixels)
        double shortSide;      // côté le plus court (pixels)
        double ratio;          // longSide / shortSide
        double distanceCm;     // distance estimée via projection sol (cm)
        double centerX;        // centre X dans l'image (pixels)
        double angleDeg;       // orientation au sol (0° = perpendiculaire à la caméra)
        DetectedColor color;
    };

    vector<Candidate> candidates;

    // --- Détection sur chaque masque couleur ---
    auto detectOnMask = [&](const Mat& mask, DetectedColor col) {
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (const auto& cnt : contours) {
            if (contourArea(cnt) < MIN_CONTOUR_AREA) continue;

            RotatedRect rr = minAreaRect(cnt);

            double side1 = rr.size.width;
            double side2 = rr.size.height;
            double longSide  = max(side1, side2);
            double shortSide = min(side1, side2);

            if (shortSide < 1.0) continue;

            double ratio = longSide / shortSide;
            if (ratio < RATIO_MIN || ratio > RATIO_MAX) continue;

            // --- Distance via projection au sol ---
            // Le point de contact au sol = le bas du rectangle dans l'image
            // On prend le point le plus bas (y le plus grand) du rotated rect
            Point2f vertices[4];
            rr.points(vertices);
            float bottomY = 0;
            for (int i = 0; i < 4; i++) {
                bottomY = max(bottomY, vertices[i].y);
            }

            // Seuls les objets SOUS l'horizon sont au sol devant nous
            if (bottomY <= HORIZON_Y) continue;

            double distCm = (FOCAL_LENGTH * CAM_HEIGHT_CM) / (bottomY - HORIZON_Y);

            // --- Angle d'orientation au sol ---
            // On veut l'angle du côté LONG par rapport à l'horizontale de l'image
            // Horizontale dans l'image = perpendiculaire à la direction de la caméra
            // → 0° = tasseau perpendiculaire à la caméra (idéal pour prise frontale)
            double angle = rr.angle;
            if (side1 > side2) {
                // width > height → le côté long est la largeur
                // angle est déjà l'angle de la largeur par rapport à l'horizontal
                // pas de correction nécessaire
            } else {
                // width < height → le côté long est la hauteur
                // on ajoute 90° pour obtenir l'angle du côté long
                angle += 90.0;
            }
            // Normaliser dans [-90, 90]
            if (angle > 90.0)  angle -= 180.0;
            if (angle < -90.0) angle += 180.0;

            Candidate c;
            c.rotRect    = rr;
            c.longSide   = longSide;
            c.shortSide  = shortSide;
            c.ratio      = ratio;
            c.distanceCm = distCm;
            c.centerX    = rr.center.x;
            c.angleDeg   = angle;
            c.color      = col;
            candidates.push_back(c);
        }
    };

    detectOnMask(maskYellow, DetectedColor::Yellow);
    detectOnMask(maskBlue, DetectedColor::Blue);

    // --- Debug : dessiner tous les candidats ---
    if (debugEnabled && !debugFrame.empty()) {
        // Ligne d'horizon
        line(debugFrame, Point(0, (int)HORIZON_Y), Point(debugFrame.cols, (int)HORIZON_Y),
             Scalar(100, 100, 100), 1, LINE_AA);
        putText(debugFrame, "horizon", Point(5, (int)HORIZON_Y - 5),
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(100, 100, 100), 1);
        // Ligne centrale
        line(debugFrame, Point((int)cx, 0), Point((int)cx, debugFrame.rows),
             Scalar(100, 100, 100), 1, LINE_AA);

        for (const auto& c : candidates) {
            // Couleur du contour selon la couleur détectée
            Scalar drawColor = (c.color == DetectedColor::Yellow)
                ? Scalar(0, 255, 255)   // jaune en BGR
                : Scalar(255, 100, 0);  // bleu en BGR

            // Dessiner le rectangle orienté
            Point2f pts[4];
            c.rotRect.points(pts);
            for (int i = 0; i < 4; i++) {
                line(debugFrame, pts[i], pts[(i+1)%4], drawColor, 2, LINE_AA);
            }

            // Label : distance + angle
            char label[64];
            snprintf(label, sizeof(label), "%.0fcm %.0fdeg", c.distanceCm, c.angleDeg);
            putText(debugFrame, label,
                    Point((int)c.rotRect.center.x - 40, (int)c.rotRect.center.y - 10),
                    FONT_HERSHEY_SIMPLEX, 0.45, drawColor, 1, LINE_AA);
        }
    }

    // --- Sélection du meilleur candidat (le plus proche) ---
    if (candidates.empty()) {
        firstDetection = true;
        result.objectDetected = false;
        result.order = CameraOrder::Recherche;

        if (debugEnabled && !debugFrame.empty()) {
            putText(debugFrame, "RECHERCHE...", Point(10, 30),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, LINE_AA);
        }
        return result;
    }

    auto& best = *min_element(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) { return a.distanceCm < b.distanceCm; });

    result.objectDetected = true;
    result.color = best.color;

    // --- Debug : surligner le meilleur candidat ---
    if (debugEnabled && !debugFrame.empty()) {
        Scalar bestColor(0, 255, 0); // vert pour le meilleur
        Point2f pts[4];
        best.rotRect.points(pts);
        for (int i = 0; i < 4; i++) {
            line(debugFrame, pts[i], pts[(i+1)%4], bestColor, 3, LINE_AA);
        }
        // Croix au centre
        int ccx = (int)best.rotRect.center.x;
        int ccy = (int)best.rotRect.center.y;
        drawMarker(debugFrame, Point(ccx, ccy), bestColor, MARKER_CROSS, 15, 2);
    }

    double raw_z = best.distanceCm;
    double raw_x = (best.centerX - cx) * raw_z / FOCAL_LENGTH;
    double raw_r = best.ratio;
    double raw_angle = best.angleDeg;

    // --- Lissage EMA ---
    if (firstDetection) {
        smooth_z = raw_z; smooth_x = raw_x; smooth_r = raw_r; smooth_angle = raw_angle;
        firstDetection = false;
    } else {
        smooth_z     = ALPHA * raw_z     + (1.0 - ALPHA) * smooth_z;
        smooth_x     = ALPHA * raw_x     + (1.0 - ALPHA) * smooth_x;
        smooth_r     = ALPHA * raw_r     + (1.0 - ALPHA) * smooth_r;
        smooth_angle = ALPHA * raw_angle + (1.0 - ALPHA) * smooth_angle;
    }

    result.smoothZ     = smooth_z;
    result.smoothX     = smooth_x;
    result.smoothAngle = smooth_angle;
    result.smoothRatio = smooth_r;

    // --- Génération de l'ordre ---
    // Le tasseau est toujours à plat au sol.
    // Objectif : s'aligner, se centrer, puis avancer/reculer à la bonne distance.
    //
    // Angle ≈ 0° → le tasseau est perpendiculaire à la caméra (côté long horizontal)
    //            → c'est la position idéale pour une prise frontale
    // Angle > 0° → tasseau tourné vers la droite → pivoter horaire
    // Angle < 0° → tasseau tourné vers la gauche → pivoter anti-horaire

    bool isAligned  = abs(smooth_angle) < ANGLE_TOL;
    bool isCentered = abs(smooth_x) < TOL_X;
    bool isGoodDist = abs(smooth_z - TARGET_Z) < TOL_Z;

    // Priorité : alignement > centrage > distance
    if (!isAligned) {
        result.order = (smooth_angle > 0) ? CameraOrder::TourneHoraire
                                          : CameraOrder::TourneAntiHoraire;
    } else if (!isCentered) {
        result.order = (smooth_x > TOL_X) ? CameraOrder::Gauche : CameraOrder::Droite;
    } else if (!isGoodDist) {
        result.order = (smooth_z > TARGET_Z + TOL_Z) ? CameraOrder::Avancer : CameraOrder::Reculer;
    } else {
        result.order = CameraOrder::Parfait;
    }

    // --- Debug : afficher l'ordre et les valeurs lissées ---
    if (debugEnabled && !debugFrame.empty()) {
        const char* colorName = (result.color == DetectedColor::Yellow) ? "JAUNE" : "BLEU";
        Scalar txtColor = (result.order == CameraOrder::Parfait)
            ? Scalar(0, 255, 0) : Scalar(0, 140, 255);

        char info[128];
        snprintf(info, sizeof(info), "%s %s | Z:%.1f X:%.1f Ang:%.1f",
                 colorName, orderStr(result.order), smooth_z, smooth_x, smooth_angle);
        putText(debugFrame, info, Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.65, txtColor, 2, LINE_AA);
    }

    return result;
}
