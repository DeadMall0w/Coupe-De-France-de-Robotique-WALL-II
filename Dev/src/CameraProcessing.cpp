// Traitement caméra : détection des tasseaux par tags ArUco
// Tasseaux 150 mm × 50 mm × 30 mm, tag ArUco 40 mm sur le dessus (face 150×50)
// 4 tasseaux côte à côte → bloc de 150 × 200 × 30 mm
//   - ID 36 → zone bleue
//   - ID 47 → zone jaune
//
// Modèle géométrique (plan sagittal) :
//   α = pitch + atan((py − cy) / focal)
//   Z = (h_cam − h_tag) / tan(α)          [mm, distance avant]
//   X = Z * (px − cx) / focal             [mm, décalage latéral]

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
// CONSTANTES PHYSIQUES (en mm)
// ==========================================

static constexpr double TASSEAU_LONG_MM      = 150.0;  // longueur d'un tasseau
static constexpr double TASSEAU_WIDTH_MM     =  50.0;  // largeur d'un tasseau
static constexpr double TASSEAU_HEIGHT_MM    =  30.0;  // hauteur d'un tasseau
static constexpr double TAG_CENTER_HEIGHT_MM =  TASSEAU_HEIGHT_MM; // tag sur le dessus → 30 mm du sol
static constexpr double TAG_SIZE_MM          =  40.0;

// Nombre de tasseaux attendus côte à côte (bloc 150 × 200 mm)
static constexpr int    TASSEAUX_PAR_GROUPE  = 4;

static constexpr int TAG_ID_BLUE   = 36;
static constexpr int TAG_ID_YELLOW = 47;

// ==========================================
// PARAMÈTRES CAMÉRA — À calibrer sur le robot
// ==========================================
//  CAM_HEIGHT_MM : hauteur du centre optique au-dessus du sol (mm)
//  CAM_PITCH_DEG : inclinaison vers l'avant (deg, > 0 = vers le bas)
//  FOCAL_PX      : focale estimée en pixels
//                  Calibration : caisse à distance D (mm), largeur W_px mesurée
//                  → focal = W_px * D / CRATE_LONG_MM
//  CAM_CX / CY   : centre optique (pixels)

static constexpr double CAM_HEIGHT_MM = 200.0; // ← À AJUSTER
static constexpr double CAM_PITCH_DEG =  30.0; // ← À AJUSTER
static constexpr double FOCAL_PX      = 600.0; // ← À AJUSTER
static constexpr double CAM_CX        = 320.0;
static constexpr double CAM_CY        = 240.0;

// ==========================================
// TOLÉRANCES POUR LES ORDRES (en mm / degrés)
// ==========================================

static constexpr double TARGET_Z     = 300.0; // distance cible pour prise (mm)
static constexpr double TOL_Z        =  50.0; // tolérance distance (mm)
static constexpr double TOL_X        =  40.0; // tolérance latérale (mm)
static constexpr double TARGET_ANGLE =  90.0; // angle cible du tag (degrés, ±90° = tasseau perpendiculaire)
static constexpr double ANGLE_TOL    =  5.0; // tolérance alignement (degrés)

// Lissage EMA
static constexpr double ALPHA = 0.2;

// ==========================================
// PIPELINES GSTREAMER (RPi Camera CSI)
// ==========================================

static const vector<pair<string, string>> GST_PIPELINES = {
    { "libcamerasrc (NV12)",
      "libcamerasrc ! capsfilter caps=video/x-raw,format=NV12,width=640,height=480,framerate=30/1 ! "
      "videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true" },

    { "libcamerasrc (auto)",
      "libcamerasrc ! video/x-raw,width=640,height=480 ! "
      "videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true" },

    { "rpicam-vid (pipe)",
      "rpicam-vid -t 0 --width 640 --height 480 --framerate 30 --codec mjpeg -o - ! "
      "jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true" },
};

// ==========================================
// UTILITAIRES GÉOMÉTRIQUES (internes)
// ==========================================

static Point2f tagCenter(const vector<Point2f>& c) {
    return (c[0] + c[1] + c[2] + c[3]) * 0.25f;
}

static double tagPixelSize(const vector<Point2f>& c) {
    return (norm(c[1]-c[0]) + norm(c[2]-c[1]) + norm(c[3]-c[2]) + norm(c[0]-c[3])) / 4.0;
}

static double tagAngleRad(const vector<Point2f>& c) {
    return atan2(c[1].y - c[0].y, c[1].x - c[0].x);
}

// Position 3D réelle depuis les coins du tag
struct WorldPos { double Z_mm, X_mm, yaw_deg; bool valid; };

static WorldPos tagToWorldPos(const vector<Point2f>& corners) {
    WorldPos wp{};
    Point2f tc = tagCenter(corners);

    const double pitch_rad = CAM_PITCH_DEG * CV_PI / 180.0;
    const double delta_v   = atan2(tc.y - CAM_CY, FOCAL_PX);
    const double alpha     = pitch_rad + delta_v;

    if (alpha <= 0.005) { wp.valid = false; return wp; }

    const double h_eff = (CAM_HEIGHT_MM > TAG_CENTER_HEIGHT_MM)
                         ? CAM_HEIGHT_MM - TAG_CENTER_HEIGHT_MM
                         : CAM_HEIGHT_MM;
    wp.Z_mm    = h_eff / tan(alpha);
    wp.X_mm    = wp.Z_mm * (tc.x - CAM_CX) / FOCAL_PX;
    wp.yaw_deg = tagAngleRad(corners) * 180.0 / CV_PI;
    wp.valid   = true;
    return wp;
}

// Contour image d'un tasseau déduit du tag (vue du dessus : 150 × 50 mm)
static RotatedRect deduceTasseauRect(const vector<Point2f>& corners) {
    const double pxPerMm = tagPixelSize(corners) / TAG_SIZE_MM;
    return RotatedRect(
        tagCenter(corners),
        Size2f(static_cast<float>(TASSEAU_LONG_MM  * pxPerMm),
               static_cast<float>(TASSEAU_WIDTH_MM * pxPerMm)),
        static_cast<float>(tagAngleRad(corners) * 180.0 / CV_PI)
    );
}

// ==========================================
// Conversion ordre → texte pour debug
// ==========================================

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
// Constructeur / Destructeur
// ================================

CameraProcessing::CameraProcessing()
    : arucoDict(aruco::getPredefinedDictionary(aruco::DICT_4X4_50))
    , detector(arucoDict, arucoParams)
{
    emaStates.clear();
}

CameraProcessing::~CameraProcessing() {
    release();
}

// ================================
// Initialisation / Libération
// ================================

bool CameraProcessing::init(int cameraId) {
    if (cameraId < 0) {
        for (const auto& [name, pipeline] : GST_PIPELINES) {
            cout << YELLOW << "[Camera] Essai pipeline : " << name << "..." << RESET << endl;
            cap.open(pipeline, CAP_GSTREAMER);
            if (cap.isOpened()) {
                Mat testFrame;
                cap >> testFrame;
                if (!testFrame.empty()) {
                    cout << GREEN << "[Camera] OK avec : " << name
                         << " (" << testFrame.cols << "x" << testFrame.rows << ")"
                         << RESET << endl;
                    return true;
                }
                cap.release();
                cerr << YELLOW << "[Camera] Pipeline " << name
                     << " ouvert mais frames vides, essai suivant..." << RESET << endl;
            }
        }

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

bool CameraProcessing::isOpened() const { return cap.isOpened(); }
void CameraProcessing::setDebug(bool enabled) { debugEnabled = enabled; }
Mat  CameraProcessing::getDebugFrame() const  { return debugFrame; }

// ================================
// Traitement d'une frame (multi-caisses)
// ================================

CameraResult CameraProcessing::processNextFrame() {
    CameraResult result;

    Mat frame;
    cap >> frame;
    if (frame.empty()) {
        cerr << RED << "[Camera] Erreur : frame vide" << RESET << endl;
        return result;
    }

    if (debugEnabled)
        debugFrame = frame.clone();

    // --- Détection ArUco ---
    vector<vector<Point2f>> corners, rejected;
    vector<int> ids;
    detector.detectMarkers(frame, corners, ids, rejected);

    // Rappel paramètres en incrustation debug
    if (debugEnabled && !debugFrame.empty()) {
        putText(debugFrame,
                "h=" + to_string((int)CAM_HEIGHT_MM) +
                "mm  pitch=" + to_string((int)CAM_PITCH_DEG) +
                "deg  f=" + to_string((int)FOCAL_PX) + "px",
                Point(8, 18), FONT_HERSHEY_SIMPLEX, 0.42, Scalar(220,220,220), 1, LINE_AA);
    }

    // --- Collecter les indices des tags valides + leur position image ---
    struct ValidTag { size_t idx; int id; };
    vector<ValidTag> validTags;
    for (size_t i = 0; i < ids.size(); i++) {
        int id = ids[i];
        if (id == TAG_ID_BLUE || id == TAG_ID_YELLOW)
            validTags.push_back({i, id});
    }

    // --- Traiter chaque tag valide ---
    for (const auto& vt : validTags) {
        size_t i = vt.idx;
        int    id = vt.id;

        WorldPos wp = tagToWorldPos(corners[i]);
        if (!wp.valid) continue;

        const double raw_z     = wp.Z_mm;
        const double raw_x     = wp.X_mm;
        const double raw_angle = wp.yaw_deg;
        const double raw_r     = tagPixelSize(corners[i]) / TAG_SIZE_MM;

        // --- Lissage EMA par tag ID ---
        EmaState& ema = emaStates[id];
        if (ema.first) {
            ema.z = raw_z; ema.x = raw_x;
            ema.r = raw_r; ema.angle = raw_angle;
            ema.first = false;
        } else {
            ema.z     = ALPHA * raw_z     + (1.0 - ALPHA) * ema.z;
            ema.x     = ALPHA * raw_x     + (1.0 - ALPHA) * ema.x;
            ema.r     = ALPHA * raw_r     + (1.0 - ALPHA) * ema.r;
            ema.angle = ALPHA * raw_angle + (1.0 - ALPHA) * ema.angle;
        }

        DetectedCrate crate;
        crate.tagId       = id;
        crate.color       = (id == TAG_ID_BLUE) ? DetectedColor::Blue : DetectedColor::Yellow;
        crate.smoothZ     = ema.z;
        crate.smoothX     = ema.x;
        crate.smoothAngle = ema.angle;
        crate.smoothRatio = ema.r;
        // L'ordre individuel est calculé plus bas après le groupe
        result.crates.push_back(crate);

        // --- Debug visuel pour chaque tasseau ---
        if (debugEnabled && !debugFrame.empty()) {
            const bool   isBlue   = (id == TAG_ID_BLUE);
            const Scalar colTag   = isBlue ? Scalar(255,80,0)  : Scalar(0,200,255);
            const Scalar colCrate = isBlue ? Scalar(180,30,0)  : Scalar(0,160,200);
            const string label    = isBlue ? "BLEU  [36]"      : "JAUNE [47]";

            // Dessiner le tag
            aruco::drawDetectedMarkers(debugFrame,
                                       vector<vector<Point2f>>{corners[i]},
                                       vector<int>{id}, colTag);

            // Contour tasseau (vue du dessus : 150 × 50 mm)
            RotatedRect tasseauRect = deduceTasseauRect(corners[i]);
            Point2f pts[4]; tasseauRect.points(pts);
            for (int j = 0; j < 4; j++)
                line(debugFrame, pts[j], pts[(j+1)%4], colCrate, 2, LINE_AA);
            circle(debugFrame, tasseauRect.center, 4, Scalar(255,255,255), -1, LINE_AA);

            // Flèche axe longitudinal
            double pxSize = tagPixelSize(corners[i]);
            const double yaw_rad = tagAngleRad(corners[i]);
            const double halfL   = (TASSEAU_LONG_MM / TAG_SIZE_MM) * pxSize * 0.5;
            Point2f axis(static_cast<float>(cos(yaw_rad) * halfL),
                         static_cast<float>(sin(yaw_rad) * halfL));
            Point2f tc = tagCenter(corners[i]);
            arrowedLine(debugFrame, tc - axis, tc + axis, Scalar(255,255,0), 1, LINE_AA, 0, 0.08);

            // Annotation compacte par tasseau
            int tx = (int)tc.x - 50;
            int ty = (int)tc.y;
            putText(debugFrame, label,
                    Point(tx, ty-30), FONT_HERSHEY_SIMPLEX, 0.38, colCrate, 1, LINE_AA);
            string sPos = "Z=" + to_string((int)ema.z) + " X=" +
                          (ema.x >= 0 ? "+" : "") + to_string((int)ema.x);
            putText(debugFrame, sPos, Point(tx, ty-16), FONT_HERSHEY_SIMPLEX, 0.35, colCrate, 1, LINE_AA);
        }
    }

    result.objectDetected = !result.crates.empty();

    // ===================================================================
    // ORDRE GLOBAL : basé sur le centre du groupe de tasseaux
    // ===================================================================
    if (!result.crates.empty()) {
        double sumZ = 0, sumX = 0, sumAngle = 0;
        for (const auto& c : result.crates) {
            sumZ     += c.smoothZ;
            sumX     += c.smoothX;
            sumAngle += c.smoothAngle;
        }
        int n = (int)result.crates.size();
        result.groupCenterZ = sumZ / n;
        result.groupCenterX = sumX / n;
        result.groupAngle   = sumAngle / n;

        // Erreur angulaire par rapport à ±90° (on prend le plus proche)
        double angleErr = abs(result.groupAngle) - TARGET_ANGLE; // >0 = dépasse 90°, <0 = en dessous
        bool isAligned  = abs(angleErr)                       < ANGLE_TOL;
        bool isCentered = abs(result.groupCenterX)            < TOL_X;
        bool isGoodDist = abs(result.groupCenterZ - TARGET_Z) < TOL_Z;

        if (!isAligned) {
            // angleErr > 0 : |angle| trop grand (> 90°+tol) → tourner pour réduire |angle|
            // angleErr < 0 : |angle| trop petit (< 90°-tol) → tourner pour augmenter |angle|
            // Le sens de correction dépend du signe de l'angle mesuré :
            //   angle > 0 et trop petit → TourneHoraire (augmenter)
            //   angle > 0 et trop grand → TourneAntiHoraire (diminuer)
            //   angle < 0 et trop petit (abs) → TourneAntiHoraire (angle plus négatif)
            //   angle < 0 et trop grand (abs) → TourneHoraire (angle moins négatif)
            bool needMore = (angleErr < 0); // |angle| trop petit, besoin d'augmenter
            if (result.groupAngle > 0)
                result.groupOrder = needMore ? CameraOrder::TourneHoraire : CameraOrder::TourneAntiHoraire;
            else
                result.groupOrder = needMore ? CameraOrder::TourneAntiHoraire : CameraOrder::TourneHoraire;
        } else if (!isCentered) {
            result.groupOrder = (result.groupCenterX > TOL_X)
                ? CameraOrder::Gauche : CameraOrder::Droite;
        } else if (!isGoodDist) {
            result.groupOrder = (result.groupCenterZ > TARGET_Z + TOL_Z)
                ? CameraOrder::Avancer : CameraOrder::Reculer;
        } else {
            result.groupOrder = CameraOrder::Parfait;
        }

        // Copier l'ordre global dans chaque crate (l'ordre individuel n'a plus de sens)
        for (auto& c : result.crates)
            c.order = result.groupOrder;
    }

    // --- Debug : résumé groupe en haut de l'écran ---
    if (debugEnabled && !debugFrame.empty()) {
        if (result.crates.empty()) {
            putText(debugFrame, "RECHERCHE...", Point(10, 50),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255), 2, LINE_AA);
        } else {
            // Ligne 1 : ordre global + position groupe
            Scalar txtColor = (result.groupOrder == CameraOrder::Parfait)
                ? Scalar(0,255,0) : Scalar(0,140,255);
            char info[256];
            snprintf(info, sizeof(info), "%s | Grp Z:%.0fmm X:%+.0fmm cap:%.1fdeg  [%d/%d tass.]",
                     orderStr(result.groupOrder),
                     result.groupCenterZ, result.groupCenterX, result.groupAngle,
                     (int)result.crates.size(), TASSEAUX_PAR_GROUPE);
            putText(debugFrame, info, Point(10, 45),
                    FONT_HERSHEY_SIMPLEX, 0.52, txtColor, 2, LINE_AA);

            // Dessiner le contour global du groupe (englobant tous les tags)
            // Collecte de tous les coins de tous les tasseaux détectés
            vector<Point2f> allPts;
            for (const auto& vt : validTags) {
                RotatedRect rr = deduceTasseauRect(corners[vt.idx]);
                Point2f pts[4]; rr.points(pts);
                for (int j = 0; j < 4; j++) allPts.push_back(pts[j]);
            }
            if (allPts.size() >= 4) {
                RotatedRect groupRect = minAreaRect(allPts);
                Point2f gpts[4]; groupRect.points(gpts);
                for (int j = 0; j < 4; j++)
                    line(debugFrame, gpts[j], gpts[(j+1)%4], Scalar(0,255,255), 2, LINE_AA);
                // Croix au centre du groupe
                Point2f gc = groupRect.center;
                line(debugFrame, gc + Point2f(-8,0), gc + Point2f(8,0), Scalar(0,255,255), 2, LINE_AA);
                line(debugFrame, gc + Point2f(0,-8), gc + Point2f(0,8), Scalar(0,255,255), 2, LINE_AA);
            }
        }
    }

    // Réinitialiser le lissage EMA des tags qui ne sont plus visibles
    // (s'ils réapparaissent, le lissage reprend de zéro)
    vector<int> toRemove;
    for (auto& [tagId, ema] : emaStates) {
        bool found = false;
        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] == tagId) { found = true; break; }
        }
        if (!found) toRemove.push_back(tagId);
    }
    for (int id : toRemove) emaStates.erase(id);

    return result;
}

