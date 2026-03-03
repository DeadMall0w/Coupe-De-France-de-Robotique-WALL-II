// Test visuel du LIDAR avec export JSON pour visualisation HTML
// Lance le LIDAR, traite les données via le pipeline LidarProcessing,
// et écrit un fichier JSON contenant toutes les données utiles pour la visualisation.
//
// Usage : ./test_lidar_visual [robot_x] [robot_y] [robot_orientation_deg]
//   Par défaut : robot à (50, 100) cm, orientation 0°

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <csignal>

#include "../../Dev/includes/ILidar.h"
#include "../../Dev/includes/Slamtec.h"
#include "../../Dev/includes/Board.hpp"
#include "../../Dev/includes/Constant.h"
#include "../../Dev/includes/LidarProcessing.h"
#include "../../Dev/includes/color.h"

// ================================
// Gestion signal (Ctrl+C)
// ================================
// Pointeur global vers le flag stop : le handler de signal ne peut pas capturer
// de variables locales, on passe donc par un pointeur global.
static std::atomic<bool>* g_stop = nullptr;

void signalHandler(int signum) {
    (void)signum;
    if (g_stop) {
        g_stop->store(true);
    }
}

// ================================
// Export JSON
// ================================

// Écrit toutes les données d'un scan dans un fichier JSON
void writeDebugJSON(
    const std::string& filename,
    const Position& robotPos,
    double robotOrientation_deg,
    const std::vector<ScanPoint>& rawPoints,
    const std::vector<AbsolutePoint>& filteredPoints,
    const std::vector<AbsolutePoint>& outsidePoints,
    const LidarProcessingResult& result,
    const Board& board,
    int scanCount)
{
    std::ofstream out(filename, std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << RED << "[ERREUR] Impossible d'écrire " << filename << RESET << std::endl;
        return;
    }

    out << std::fixed << std::setprecision(2);

    out << "{\n";

    // Infos scan
    out << "  \"scanNumber\": " << scanCount << ",\n";

    // Plateau
    out << "  \"plateau\": { \"width\": " << MAP_WIDTH_CM << ", \"height\": " << MAP_HEIGHT_CM << " },\n";

    // Notre robot
    out << "  \"myRobot\": { \"x\": " << robotPos.x
        << ", \"y\": " << robotPos.y
        << ", \"orientation_deg\": " << robotOrientation_deg << " },\n";

    // Ennemi
    out << "  \"enemy\": { \"detected\": " << (result.enemyDetected ? "true" : "false");
    if (result.enemyDetected) {
        out << ", \"x\": " << result.enemyPosition.x_cm
            << ", \"y\": " << result.enemyPosition.y_cm
            << ", \"clusterSize\": " << result.clusterSize;
    }
    out << " },\n";

    // Stats
    out << "  \"stats\": { \"rawPoints\": " << rawPoints.size()
        << ", \"pointsInMap\": " << result.pointsInMap
        << ", \"pointsFiltered\": " << result.pointsFiltered
        << ", \"outsideMapPoints\": " << outsidePoints.size()
        << " },\n";

    // Angles morts
    out << "  \"blindSpots\": [";
    for (size_t i = 0; i < BLIND_SPOT_COUNT; i++) {
        if (i > 0) out << ", ";
        out << "[" << LIDAR_BLIND_SPOTS[i].first << ", " << LIDAR_BLIND_SPOTS[i].second << "]";
    }
    out << "],\n";

    // Points bruts (polaires, pour affichage radar)
    out << "  \"rawPoints\": [";
    for (size_t i = 0; i < rawPoints.size(); i++) {
        if (i > 0) out << ",";
        if (i % 10 == 0) out << "\n    ";
        out << "{\"a\":" << rawPoints[i].angle_deg << ",\"d\":" << rawPoints[i].dist_mm << "}";
    }
    out << "\n  ],\n";

    // Points filtrés (dans le plateau, en absolu)
    out << "  \"filteredPoints\": [";
    for (size_t i = 0; i < filteredPoints.size(); i++) {
        if (i > 0) out << ",";
        if (i % 10 == 0) out << "\n    ";
        out << "{\"x\":" << filteredPoints[i].x_cm << ",\"y\":" << filteredPoints[i].y_cm << "}";
    }
    out << "\n  ],\n";

    // Points hors plateau (en absolu, pour debug)
    out << "  \"outsidePoints\": [";
    for (size_t i = 0; i < outsidePoints.size(); i++) {
        if (i > 0) out << ",";
        if (i % 10 == 0) out << "\n    ";
        out << "{\"x\":" << outsidePoints[i].x_cm << ",\"y\":" << outsidePoints[i].y_cm << "}";
    }
    out << "\n  ],\n";

    // Zones (stockage)
    Map currentMap = const_cast<Board&>(board).getMap();
    out << "  \"zones\": {\n";
    out << "    \"storage\": [";
    for (size_t i = 0; i < currentMap.storagesZones.size(); i++) {
        if (i > 0) out << ", ";
        const Zone& z = currentMap.storagesZones[i];
        out << "{\"x\":" << z.position.x << ",\"y\":" << z.position.y
            << ",\"w\":" << z.size.width << ",\"h\":" << z.size.length
            << ",\"state\":" << static_cast<int>(z.state) << "}";
    }
    out << "],\n";

    // Zones (dépôt)
    out << "    \"deposit\": [";
    for (size_t i = 0; i < currentMap.depositsZones.size(); i++) {
        if (i > 0) out << ", ";
        const Zone& z = currentMap.depositsZones[i];
        out << "{\"x\":" << z.position.x << ",\"y\":" << z.position.y
            << ",\"w\":" << z.size.width << ",\"h\":" << z.size.length
            << ",\"state\":" << static_cast<int>(z.state) << "}";
    }
    out << "],\n";

    // Nid
    out << "    \"nid\": {\"x\":" << currentMap.nid.position.x
        << ",\"y\":" << currentMap.nid.position.y
        << ",\"w\":" << currentMap.nid.size.width
        << ",\"h\":" << currentMap.nid.size.length
        << ",\"state\":" << static_cast<int>(currentMap.nid.state) << "}\n";

    out << "  }\n";
    out << "}\n";

    out.flush();
}


// ================================
// Version modifiée du pipeline qui retourne aussi les points intermédiaires
// ================================
struct DetailedScanResult {
    LidarProcessingResult result;
    std::vector<AbsolutePoint> filteredPoints;  // points dans le plateau
    std::vector<AbsolutePoint> outsidePoints;   // points hors plateau (pour debug)
};

constexpr double PI_LOCAL = 3.14159265358979323846;
constexpr double DEG_TO_RAD_LOCAL = PI_LOCAL / 180.0;

DetailedScanResult processDetailed(
    const std::vector<ScanPoint>& rawPoints,
    const Position& robotPos,
    double robotOrientation_deg)
{
    DetailedScanResult detailed;

    // On refait le pipeline manuellement pour capturer les résultats intermédiaires
    // mais on appelle aussi le vrai pipeline pour les effets (Board update)
    detailed.result = LidarProcessing::processScan(rawPoints, robotPos, robotOrientation_deg);

    // Recalculer les points absolus pour l'export (filteredPoints + outsidePoints)
    for (const auto& sp : rawPoints) {
        if (sp.dist_mm < 1.0f) continue;

        double absoluteAngle_rad = (robotOrientation_deg + sp.angle_deg) * DEG_TO_RAD_LOCAL;
        double dist_cm = sp.dist_mm / 10.0;

        AbsolutePoint ap = {
            robotPos.x + dist_cm * std::cos(absoluteAngle_rad),
            robotPos.y + dist_cm * std::sin(absoluteAngle_rad)
        };

        if (ap.x_cm >= 0.0 && ap.x_cm <= MAP_WIDTH_CM &&
            ap.y_cm >= 0.0 && ap.y_cm <= MAP_HEIGHT_CM) {
            detailed.filteredPoints.push_back(ap);
        } else {
            detailed.outsidePoints.push_back(ap);
        }
    }

    return detailed;
}


// ================================
// Main
// ================================
int main(int argc, char* argv[]) {
    // Position par défaut du robot (modifiable via arguments)
    double robotX = 50.0;
    double robotY = 100.0;
    double robotOrient = 0.0;

    if (argc >= 3) {
        robotX = std::stod(argv[1]);
        robotY = std::stod(argv[2]);
    }
    if (argc >= 4) {
        robotOrient = std::stod(argv[3]);
    }

    std::cout << BOLDBLUE << "=== Test Visuel LIDAR ===" << RESET << std::endl;
    std::cout << YELLOW << "Robot position : (" << robotX << ", " << robotY << ") cm" << RESET << std::endl;
    std::cout << YELLOW << "Robot orientation : " << robotOrient << "°" << RESET << std::endl;
    std::cout << YELLOW << "Plateau : " << MAP_WIDTH_CM << " x " << MAP_HEIGHT_CM << " cm" << RESET << std::endl;
    std::cout << std::endl;

    // Initialiser le Board avec la position du robot
    Board& board = Board::instance();
    board.initialiseData("../../Dev/data/config.json");
    board.moveMyRobot({robotX, robotY});
    board.setMyRobotOrientation(robotOrient);

    // Connexion au LIDAR
    auto lidar = std::make_unique<Slamtec>("/dev/ttyUSB0");

    if (!lidar->connect()) {
        std::cerr << RED << "[ERREUR] Impossible de se connecter au LIDAR" << RESET << std::endl;
        return 1;
    }

    if (!lidar->startScan()) {
        std::cerr << RED << "[ERREUR] Impossible de lancer le scan" << RESET << std::endl;
        lidar->disconnect();
        return 1;
    }

    std::cout << GREEN << "[OK] LIDAR connecté et scan lancé" << RESET << std::endl;
    std::cout << MAGENTA << "Appuie sur Entrée pour arrêter..." << RESET << std::endl;
    std::cout << YELLOW << "Ouvre lidar_viewer.html dans un navigateur pour la visualisation" << RESET << std::endl;
    std::cout << std::endl;

    // Thread d'arrêt sur Entrée
    std::atomic<bool> stop = false;

    // Enregistrer le pointeur global pour les handlers de signaux
    g_stop = &stop;

    // Intercepter Ctrl+C (SIGINT) et kill (SIGTERM) → arrêt propre
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Thread optionnel : appuyer sur Entrée arrête aussi le programme
    std::thread inputThread([&stop]() {
        std::cin.get();
        stop = true;
    });
    // On détache ce thread : si c'est le signal qui arrête, cin.get() resterait
    // bloqué indéfiniment sinon. L'OS s'occupera du thread en fin de processus.
    inputThread.detach();

    int scanCount = 0;

    // Boucle principale
    while (!stop) {
        std::vector<ScanPoint> points;

        if (lidar->grabData(points)) {
            scanCount++;

            Position robotPos = board.getMyRobot().position;
            double orientation = board.getMyRobotOrientation();

            // Traitement détaillé (pipeline + données intermédiaires)
            DetailedScanResult detailed = processDetailed(points, robotPos, orientation);

            // Export JSON
            writeDebugJSON(
                "lidar_debug.json",
                robotPos,
                orientation,
                points,
                detailed.filteredPoints,
                detailed.outsidePoints,
                detailed.result,
                board,
                scanCount
            );

            // Log console
            std::cout << "\r" << YELLOW
                << "[Scan #" << scanCount << "] "
                << "Bruts: " << points.size()
                << " | Dans plateau: " << detailed.result.pointsInMap
                << " | Filtrés: " << detailed.result.pointsFiltered;

            if (detailed.result.enemyDetected) {
                std::cout << " | " << GREEN << "ENNEMI (" 
                    << detailed.result.enemyPosition.x_cm << ", " 
                    << detailed.result.enemyPosition.y_cm << ") "
                    << detailed.result.clusterSize << "pts";
            } else {
                std::cout << " | Pas d'ennemi";
            }

            std::cout << RESET << std::flush;
        }
    }

    std::cout << std::endl;
    std::cout << BOLDBLUE << "Arrêt..." << RESET << std::endl;

    lidar->disconnect();
    // inputThread est détaché, pas besoin de join

    std::cout << GREEN << "Terminé. " << scanCount << " scans effectués." << RESET << std::endl;
    return 0;
}
