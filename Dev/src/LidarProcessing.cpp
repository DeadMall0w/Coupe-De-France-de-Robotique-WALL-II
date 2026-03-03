// Implémentation du module de traitement des données LIDAR
// Voir LidarProcessing.h pour la documentation du pipeline

#include "../includes/LidarProcessing.h"
#include "../includes/color.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

// constante mathématique
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;


// ================================
// Pipeline principal
// ================================
LidarProcessingResult LidarProcessing::processScan(
    const std::vector<ScanPoint>& rawPoints,
    const Position& robotPos,
    double robotOrientation_deg)
{
    LidarProcessingResult result;

    // ① Filtrer les angles morts
    std::vector<ScanPoint> filtered = filterBlindSpots(rawPoints);

    // ② + ③ Convertir en absolu et filtrer hors plateau
    std::vector<AbsolutePoint> mapPoints;
    mapPoints.reserve(filtered.size());

    for (const auto& sp : filtered) {
        // ignorer les points à distance nulle (pas de retour)
        if (sp.dist_mm < 1.0f) continue;

        AbsolutePoint ap = polarToAbsolute(sp, robotPos, robotOrientation_deg);

        if (isInsideMap(ap)) {
            mapPoints.push_back(ap);
        }
    }

    result.pointsInMap = static_cast<int>(mapPoints.size());
    result.pointsFiltered = static_cast<int>(rawPoints.size()) - result.pointsInMap;

    // si pas assez de points dans le plateau, pas de détection possible
    if (static_cast<int>(mapPoints.size()) < MIN_CLUSTER_POINTS) {
        result.enemyDetected = false;
        return result;
    }

    // ④ Clustering : trouver le robot adverse
    Cluster mainCluster = clusterPoints(mapPoints);

    if (static_cast<int>(mainCluster.points.size()) >= MIN_CLUSTER_POINTS) {
        result.enemyDetected = true;
        result.enemyPosition = mainCluster.centroid;
        result.clusterSize = static_cast<int>(mainCluster.points.size());

        // ⑤ Vérifier les zones visitées par l'ennemi
        Board& board = Board::instance();
        checkZoneVisits(mainCluster.centroid, board);

        // Mettre à jour la position du robot adverse dans le Board
        board.moveEnemyRobot({mainCluster.centroid.x_cm, mainCluster.centroid.y_cm});
    } else {
        result.enemyDetected = false;
    }

    return result;
}


// ================================
// ① Filtrage des angles morts
// ================================
std::vector<ScanPoint> LidarProcessing::filterBlindSpots(const std::vector<ScanPoint>& points) {
    std::vector<ScanPoint> filtered;
    filtered.reserve(points.size());

    for (const auto& p : points) {
        if (!isInBlindSpot(p.angle_deg)) {
            filtered.push_back(p);
        }
    }

    return filtered;
}

bool LidarProcessing::isInBlindSpot(double angle_deg) {
    double a = normalizeAngle(angle_deg);

    for (const auto& [start, end] : LIDAR_BLIND_SPOTS) {
        double s = normalizeAngle(start);
        double e = normalizeAngle(end);

        if (s <= e) {
            // cas normal : la plage ne traverse pas 0°
            if (a >= s && a <= e) return true;
        } else {
            // cas où la plage traverse 0° (ex: 350° → 10°)
            if (a >= s || a <= e) return true;
        }
    }

    return false;
}


// ================================
// ② Conversion polaire → absolu
// ================================
AbsolutePoint LidarProcessing::polarToAbsolute(
    const ScanPoint& point,
    const Position& robotPos,
    double robotOrientation_deg)
{
    // l'angle absolu = orientation du robot + angle du point LIDAR
    double absoluteAngle_rad = (robotOrientation_deg + point.angle_deg) * DEG_TO_RAD;

    // conversion mm → cm
    double dist_cm = point.dist_mm / 10.0;

    return {
        robotPos.x + dist_cm * std::cos(absoluteAngle_rad),
        robotPos.y + dist_cm * std::sin(absoluteAngle_rad)
    };
}


// ================================
// ③ Vérification dans le plateau
// ================================
bool LidarProcessing::isInsideMap(const AbsolutePoint& point) {
    return point.x_cm >= 0.0 && point.x_cm <= MAP_WIDTH_CM
        && point.y_cm >= 0.0 && point.y_cm <= MAP_HEIGHT_CM;
}


// ================================
// ④ Clustering simple par proximité
// ================================
// Algorithme : on trie les points, puis on fait un parcours greedy.
// On regroupe les points proches (< CLUSTER_MAX_GAP_CM) et on retourne le plus gros cluster.
// C'est suffisant car on s'attend à un seul objet principal (le robot adverse).
Cluster LidarProcessing::clusterPoints(const std::vector<AbsolutePoint>& points) {
    if (points.empty()) return {};

    // On utilise un algorithme simple de clustering :
    // Pour chaque point non assigné, on crée un cluster et on y ajoute
    // tous les points à distance < CLUSTER_MAX_GAP_CM de manière récursive (BFS).

    std::vector<bool> visited(points.size(), false);
    Cluster bestCluster;

    for (size_t i = 0; i < points.size(); i++) {
        if (visited[i]) continue;

        // Nouveau cluster à partir du point i
        Cluster current;
        std::vector<size_t> queue;
        queue.push_back(i);
        visited[i] = true;

        size_t front = 0;
        while (front < queue.size()) {
            size_t idx = queue[front++];
            current.points.push_back(points[idx]);

            // Chercher les voisins non visités
            for (size_t j = 0; j < points.size(); j++) {
                if (!visited[j] && distance(points[idx], points[j]) < CLUSTER_MAX_GAP_CM) {
                    visited[j] = true;
                    queue.push_back(j);
                }
            }
        }

        // Garder le plus gros cluster
        if (current.points.size() > bestCluster.points.size()) {
            bestCluster = current;
        }
    }

    // Calculer le centroïde du meilleur cluster
    if (!bestCluster.points.empty()) {
        double sumX = 0, sumY = 0;
        for (const auto& p : bestCluster.points) {
            sumX += p.x_cm;
            sumY += p.y_cm;
        }
        bestCluster.centroid = {
            sumX / bestCluster.points.size(),
            sumY / bestCluster.points.size()
        };
    }

    return bestCluster;
}


// ================================
// ⑤ Vérification des zones visitées
// ================================
void LidarProcessing::checkZoneVisits(const AbsolutePoint& enemyPos, Board& board) {
    Map currentMap = board.getMap();

    // Vérifier les zones de stockage
    for (size_t i = 0; i < currentMap.storagesZones.size(); i++) {
        const Zone& zone = currentMap.storagesZones[i];
        double zoneCenterX = zone.position.x + zone.size.width / 2.0;
        double zoneCenterY = zone.position.y + zone.size.length / 2.0;

        double dist = distance(enemyPos, {zoneCenterX, zoneCenterY});
        if (dist < ZONE_PROXIMITY_CM) {
            board.setStorageZoneState(i, ZoneState::UsedByEnemy);
        }
    }

    // Vérifier les zones de dépôt
    for (size_t i = 0; i < currentMap.depositsZones.size(); i++) {
        const Zone& zone = currentMap.depositsZones[i];
        double zoneCenterX = zone.position.x + zone.size.width / 2.0;
        double zoneCenterY = zone.position.y + zone.size.length / 2.0;

        double dist = distance(enemyPos, {zoneCenterX, zoneCenterY});
        if (dist < ZONE_PROXIMITY_CM) {
            board.setDepositZoneState(i, ZoneState::UsedByEnemy);
        }
    }

    // Vérifier le nid
    {
        const Zone& nid = currentMap.nid;
        double nidCenterX = nid.position.x + nid.size.width / 2.0;
        double nidCenterY = nid.position.y + nid.size.length / 2.0;

        double dist = distance(enemyPos, {nidCenterX, nidCenterY});
        if (dist < ZONE_PROXIMITY_CM) {
            board.setNidState(ZoneState::UsedByEnemy);
        }
    }
}


// ================================
// Utilitaires
// ================================
double LidarProcessing::distance(const AbsolutePoint& a, const AbsolutePoint& b) {
    double dx = a.x_cm - b.x_cm;
    double dy = a.y_cm - b.y_cm;
    return std::sqrt(dx * dx + dy * dy);
}

double LidarProcessing::normalizeAngle(double angle_deg) {
    double a = std::fmod(angle_deg, 360.0);
    if (a < 0) a += 360.0;
    return a;
}
