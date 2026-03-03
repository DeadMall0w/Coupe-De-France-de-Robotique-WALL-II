// Module de traitement des données LIDAR
// Pipeline : filtrage angles morts → conversion polaire→absolu → filtrage hors plateau → clustering → estimation position adverse → check zones

#pragma once

#include <vector>
#include <cmath>

#include "ILidar.h"
#include "Board.hpp"
#include "Constant.h"

// ================================
// Structures
// ================================

// Point en coordonnées absolues sur le plateau (en cm)
struct AbsolutePoint {
    double x_cm;
    double y_cm;
};

// Cluster de points (représente un objet détecté)
struct Cluster {
    std::vector<AbsolutePoint> points;
    AbsolutePoint centroid; // centroïde du cluster (calculé après regroupement)
};

// Résultat complet du traitement d'un scan
struct LidarProcessingResult {
    bool enemyDetected = false;         // true si un robot adverse a été détecté
    AbsolutePoint enemyPosition = {0, 0}; // position estimée du robot adverse
    int pointsInMap = 0;                // nombre de points dans le plateau (après filtrage)
    int pointsFiltered = 0;             // nombre de points filtrés (hors plateau + angles morts)
    int clusterSize = 0;                // nombre de points dans le cluster ennemi
};

// ================================
// Classe LidarProcessing
// ================================

class LidarProcessing {
public:
    // Traite un scan complet et retourne le résultat
    // robotPos : position actuelle de notre robot sur le plateau (en cm)
    // robotOrientation : orientation de notre robot (en degrés, 0° = axe X+)
    static LidarProcessingResult processScan(
        const std::vector<ScanPoint>& rawPoints,
        const Position& robotPos,
        double robotOrientation_deg
    );

private:
    // ① Filtre les points dans les angles morts du LIDAR
    static std::vector<ScanPoint> filterBlindSpots(const std::vector<ScanPoint>& points);

    // ② Convertit un point polaire (relatif au robot) en coordonnées absolues sur le plateau
    static AbsolutePoint polarToAbsolute(
        const ScanPoint& point,
        const Position& robotPos,
        double robotOrientation_deg
    );

    // ③ Vérifie si un point absolu est dans les limites du plateau
    static bool isInsideMap(const AbsolutePoint& point);

    // ④ Regroupe les points en clusters par proximité
    //    Retourne le cluster le plus gros (celui du robot adverse)
    static Cluster clusterPoints(const std::vector<AbsolutePoint>& points);

    // ⑤ Vérifie quelles zones le robot adverse a pu visiter
    static void checkZoneVisits(const AbsolutePoint& enemyPos, Board& board);

    // Utilitaire : distance euclidienne entre deux points
    static double distance(const AbsolutePoint& a, const AbsolutePoint& b);

    // Utilitaire : vérifie si un angle est dans un angle mort
    static bool isInBlindSpot(double angle_deg);

    // Utilitaire : normalise un angle entre 0 et 360
    static double normalizeAngle(double angle_deg);
};
