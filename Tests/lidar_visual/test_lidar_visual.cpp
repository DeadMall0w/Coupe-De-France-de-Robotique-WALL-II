// Test visuel du LIDAR avec export JSON pour visualisation HTML
// Lance le LIDAR, traite les données via le pipeline LidarProcessing,
// et écrit un fichier JSON contenant toutes les données utiles pour la visualisation.
//
// Usage : ./test_lidar_visual [x_cm y_cm orientation_deg]
//   Par défaut : robot à (25, 175) cm, orientation -90°

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <algorithm>
#include <string>
#include <mutex>
#include <cstdint>

#include "../../Dev/includes/ILidar.h"
#include "../../Dev/includes/Slamtec.h"
#include "../../Dev/includes/Board.hpp"
#include "../../Dev/includes/Constant.h"
#include "../../Dev/includes/LidarProcessing.h"
#include "../../Dev/includes/chassis.h"
#include "../../Dev/includes/color.h"
#include "../../Dev/lib/include/json.hpp"

using json = nlohmann::json;

// ================================
// Gestion signal (Ctrl+C)
// ================================
// Pointeur global vers le flag stop : le handler de signal ne peut pas capturer
// de variables locales, on passe donc par un pointeur global.
static std::atomic<bool>* g_stop = nullptr;
static std::mutex g_consoleMtx;

void safePrintLine(const std::string& text)
{
    std::lock_guard<std::mutex> lock(g_consoleMtx);
    std::cout << text << std::endl;
}

void safePrintStatus(const std::string& text)
{
    std::lock_guard<std::mutex> lock(g_consoleMtx);
    std::cout << "\r" << text << "   " << std::flush;
}

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

bool applyTeensyPoseToBoard(Board& board, const ChassisPose& pose)
{
    Robot myRobot = board.getMyRobot();

    Position clampedPos = pose.positionCm;
    const double maxX = std::max(0.0, MAP_WIDTH_CM - myRobot.size.width);
    const double maxY = std::max(0.0, MAP_HEIGHT_CM - myRobot.size.length);

    clampedPos.x = std::clamp(clampedPos.x, 0.0, maxX);
    clampedPos.y = std::clamp(clampedPos.y, 0.0, maxY);

    const bool moved = board.moveMyRobot(clampedPos);
    board.setMyRobotOrientation(pose.headingDeg);
    return moved;
}

void printConsoleHelp()
{
    safePrintLine(std::string(CYAN) +
                  "\nCommandes console RPi->Teensy:\n"
                  "  h / help           : afficher l'aide\n"
                  "  m                  : maintien position\n"
                  "  s                  : stop moteurs\n"
                  "  g X Y              : aller à X,Y (cm plateau)\n"
                  "  o DEG              : rotation relative (degrés)\n"
                  "  p                  : lire la pose odométrique\n"
                  "  z                  : reset odométrie Teensy\n"
                  "  q / quit / exit    : quitter le test\n" +
                  RESET);
}

void runConsoleCommand(const std::string& line, std::atomic<bool>& stop, bool& chassisConnected)
{
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    if (cmd.empty()) {
        return;
    }

    if (cmd == "h" || cmd == "help") {
        printConsoleHelp();
        return;
    }

    if (cmd == "q" || cmd == "quit" || cmd == "exit") {
        stop = true;
        return;
    }

    if (!chassisConnected) {
        chassisConnected = chassisInit("/dev/ttyAMA3", 115200);
    }
    if (!chassisConnected) {
        safePrintLine(std::string(RED) + "[CMD] Teensy non connecté" + RESET);
        return;
    }

    if (cmd == "m") {
        safePrintLine(std::string(chassisHoldPosition() ? GREEN "[CMD] OK m" : RED "[CMD] ERR m") + RESET);
        return;
    }

    if (cmd == "s") {
        safePrintLine(std::string(chassisStopMotion() ? GREEN "[CMD] OK s" : RED "[CMD] ERR s") + RESET);
        return;
    }

    if (cmd == "z") {
        safePrintLine(std::string(chassisResetOdometry() ? GREEN "[CMD] OK z" : RED "[CMD] ERR z") + RESET);
        return;
    }

    if (cmd == "p") {
        ChassisPose pose;
        if (chassisRequestPose(pose) && pose.valid) {
            std::ostringstream oss;
            oss << GREEN << "[CMD] POS board = (" << pose.positionCm.x << ", "
                << pose.positionCm.y << ") h=" << pose.headingDeg << RESET;
            safePrintLine(oss.str());
        } else {
            safePrintLine(std::string(RED) + "[CMD] ERR p" + RESET);
        }
        return;
    }

    if (cmd == "o") {
        double angleDeg = 0.0;
        if (!(iss >> angleDeg)) {
            safePrintLine(std::string(YELLOW) + "[CMD] usage: o DEG" + RESET);
            return;
        }
        safePrintLine(std::string(chassisRotateRelativeDeg(angleDeg) ? GREEN "[CMD] OK o" : RED "[CMD] ERR o") + RESET);
        return;
    }

    if (cmd == "g") {
        double xCm = 0.0;
        double yCm = 0.0;
        if (!(iss >> xCm >> yCm)) {
            safePrintLine(std::string(YELLOW) + "[CMD] usage: g X Y" + RESET);
            return;
        }
        safePrintLine(std::string(chassisGoToPositionCm(xCm, yCm) ? GREEN "[CMD] OK g" : RED "[CMD] ERR g") + RESET);
        return;
    }

    safePrintLine(std::string(YELLOW) + "[CMD] commande inconnue. Tape 'h'." + RESET);
}

bool readViewerGoToCommand(const std::string& path, std::int64_t& lastCmdId, double& xCm, double& yCm)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }

    json cmd;
    try {
        in >> cmd;
    } catch (...) {
        return false;
    }

    if (!cmd.contains("id") || !cmd.contains("x") || !cmd.contains("y")) {
        return false;
    }

    std::int64_t id = 0;
    try {
        id = cmd["id"].get<std::int64_t>();
        xCm = cmd["x"].get<double>();
        yCm = cmd["y"].get<double>();
    } catch (...) {
        return false;
    }

    if (id <= lastCmdId) {
        return false;
    }

    lastCmdId = id;
    return true;
}


// ================================
// Main
// ================================
int main(int argc, char* argv[]) {
    // Position de départ forcée pour les tests
    double robotX = 25.0;
    double robotY = 175.0;
    double robotOrient = -90.0;

    if (argc > 1) {
        if (argc < 4) {
            std::cerr << RED
                      << "Usage: " << argv[0] << " [x_cm y_cm orientation_deg]" << RESET
                      << std::endl;
            return 1;
        }
        try {
            robotX = std::stod(argv[1]);
            robotY = std::stod(argv[2]);
            robotOrient = std::stod(argv[3]);
        } catch (const std::exception&) {
            std::cerr << RED
                      << "[ERREUR] Arguments invalides. Usage: " << argv[0]
                      << " [x_cm y_cm orientation_deg]" << RESET
                      << std::endl;
            return 1;
        }
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

    // Calibration repère châssis:
    // Teensy (0,0,0) -> Plateau (150,25,-90)
    chassisSetBoardReferencePose(robotX, robotY, robotOrient);
    // Correction axe X inversé sur le rendu (inversion de l'axe local Y)
    chassisSetLocalAxesSigns(1.0, -1.0);

    // Connexion optionnelle au Teensy châssis pour récupérer la pose odométrique
    const std::string chassisDevice = "/dev/ttyAMA3";
    bool chassisConnected = chassisInit(chassisDevice, 115200);
    bool poseWarningPrinted = false;
    if (chassisConnected) {
        if (!chassisResetOdometry()) {
            std::cout << YELLOW << "[WARN] Reset odométrie Teensy non confirmé." << RESET << std::endl;
        }

        ChassisPose initialPose;
        if (chassisRequestPose(initialPose) && initialPose.valid) {
            applyTeensyPoseToBoard(board, initialPose);
            std::cout << GREEN << "[OK] Teensy connecté, pose initiale: ("
                      << initialPose.positionCm.x << ", " << initialPose.positionCm.y
                      << ") cm, h=" << initialPose.headingDeg << "°" << RESET << std::endl;
        } else {
            std::cout << YELLOW << "[WARN] Teensy connecté mais pose initiale indisponible." << RESET << std::endl;
        }
    } else {
        std::cout << YELLOW << "[WARN] Teensy non connecté, utilisation de la position fixe (args)." << RESET << std::endl;
    }

    // Connexion au LIDAR (mode dégradé si absent)
    auto lidar = std::make_unique<Slamtec>("/dev/ttyUSB0");
    bool lidarReady = false;
    bool lidarLostWarningPrinted = false;

    if (lidar->connect() && lidar->startScan()) {
        lidarReady = true;
        std::cout << GREEN << "[OK] LIDAR connecté et scan lancé" << RESET << std::endl;
    } else {
        lidar->disconnect();
        std::cout << YELLOW << "[WARN] LIDAR indisponible, mode sans lidar actif." << RESET << std::endl;
    }
    std::cout << MAGENTA << "Console active: tape 'h' pour les commandes, 'q' pour quitter." << RESET << std::endl;
    std::cout << YELLOW << "Ouvre lidar_viewer.html dans un navigateur pour la visualisation" << RESET << std::endl;
    std::cout << std::endl;

    // Thread d'arrêt sur Entrée
    std::atomic<bool> stop = false;
    std::atomic<bool> consoleInputActive = false;

    // Enregistrer le pointeur global pour les handlers de signaux
    g_stop = &stop;

    // Intercepter Ctrl+C (SIGINT) et kill (SIGTERM) → arrêt propre
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    printConsoleHelp();

    // Thread console: simulation d'ordres RPi -> Teensy
    std::thread inputThread([&stop, &chassisConnected, &consoleInputActive]() {
        std::string line;
        while (!stop) {
            {
                std::lock_guard<std::mutex> lock(g_consoleMtx);
                std::cout << "\ncmd> " << std::flush;
            }
            consoleInputActive = true;
            if (!std::getline(std::cin, line)) {
                consoleInputActive = false;
                stop = true;
                break;
            }
            consoleInputActive = false;
            runConsoleCommand(line, stop, chassisConnected);
        }
    });
    // On détache ce thread : si c'est le signal qui arrête, getline() resterait
    // bloqué indéfiniment sinon. L'OS s'occupera du thread en fin de processus.
    inputThread.detach();

    int scanCount = 0;
    auto lastNoLidarExport = std::chrono::steady_clock::now();
    auto lastReconnectTry = std::chrono::steady_clock::now() - std::chrono::seconds(3);
    bool poseClampWarningPrinted = false;
    const std::string viewerCommandFile = "lidar_command.json";
    std::int64_t lastViewerCmdId = 0;

    // Boucle principale
    while (!stop) {
        double clickTargetX = 0.0;
        double clickTargetY = 0.0;
        if (readViewerGoToCommand(viewerCommandFile, lastViewerCmdId, clickTargetX, clickTargetY)) {
            if (!chassisConnected) {
                chassisConnected = chassisInit(chassisDevice, 115200);
            }

            if (chassisConnected) {
                bool goOk = chassisGoToPositionCm(clickTargetX, clickTargetY);
                std::ostringstream oss;
                oss << (goOk ? GREEN : RED)
                    << "[MAP] GoTo (" << clickTargetX << ", " << clickTargetY << ") -> "
                    << (goOk ? "OK" : "ERR") << RESET;
                safePrintLine(oss.str());
            } else {
                safePrintLine(std::string(RED) + "[MAP] Teensy non connecté, commande ignorée" + RESET);
            }
        }

        if (chassisConnected) {
            ChassisPose pose;
            if (chassisRequestPose(pose) && pose.valid) {
                bool moved = applyTeensyPoseToBoard(board, pose);
                if (!moved && !poseClampWarningPrinted) {
                    std::cout << "\n" << YELLOW
                              << "[WARN] Position Teensy hors plateau, clamp appliqué."
                              << RESET << std::endl;
                    poseClampWarningPrinted = true;
                }
            } else if (!poseWarningPrinted) {
                std::cout << "\n" << YELLOW
                          << "[WARN] Lecture pose Teensy impossible, dernière pose conservée."
                          << RESET << std::endl;
                poseWarningPrinted = true;
            }
        }

        Position robotPos = board.getMyRobot().position;
        double orientation = board.getMyRobotOrientation();

        if (lidarReady) {
            std::vector<ScanPoint> points;
            if (lidar->grabData(points)) {
                scanCount++;

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
                if (!consoleInputActive) {
                    std::ostringstream oss;
                    oss << YELLOW
                        << "[Scan #" << scanCount << "] "
                        << "Bruts: " << points.size()
                        << " | Dans plateau: " << detailed.result.pointsInMap
                        << " | Filtrés: " << detailed.result.pointsFiltered;

                    if (detailed.result.enemyDetected) {
                        oss << " | " << GREEN << "ENNEMI ("
                            << detailed.result.enemyPosition.x_cm << ", "
                            << detailed.result.enemyPosition.y_cm << ") "
                            << detailed.result.clusterSize << "pts";
                    } else {
                        oss << " | Pas d'ennemi";
                    }

                    oss << RESET;
                    safePrintStatus(oss.str());
                }
            } else {
                lidarReady = false;
                lidar->disconnect();
                if (!lidarLostWarningPrinted) {
                    std::cout << "\n" << YELLOW
                              << "[WARN] Flux LIDAR perdu, passage en mode sans lidar."
                              << RESET << std::endl;
                    lidarLostWarningPrinted = true;
                }
            }
        } else {
            auto now = std::chrono::steady_clock::now();

            // if (now - lastReconnectTry >= std::chrono::seconds(2)) {
            //     lastReconnectTry = now;
            //     if (lidar->connect() && lidar->startScan()) {
            //         lidarReady = true;
            //         lidarLostWarningPrinted = false;
            //         std::cout << "\n" << GREEN << "[OK] LIDAR reconnecté." << RESET << std::endl;
            //     }
            // }

            if (now - lastNoLidarExport >= std::chrono::milliseconds(150)) {
                lastNoLidarExport = now;
                scanCount++;

                std::vector<ScanPoint> emptyRaw;
                std::vector<AbsolutePoint> emptyFiltered;
                std::vector<AbsolutePoint> emptyOutside;
                LidarProcessingResult emptyResult;

                writeDebugJSON(
                    "lidar_debug.json",
                    robotPos,
                    orientation,
                    emptyRaw,
                    emptyFiltered,
                    emptyOutside,
                    emptyResult,
                    board,
                    scanCount
                );

                if (!consoleInputActive) {
                    std::ostringstream oss;
                    oss << YELLOW
                        << "[Scan #" << scanCount << "] Mode sans lidar | Position: ("
                        << robotPos.x << ", " << robotPos.y << ") h=" << orientation
                        << RESET;
                    safePrintStatus(oss.str());
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::cout << std::endl;
    std::cout << BOLDBLUE << "Arrêt..." << RESET << std::endl;

    lidar->disconnect();
    chassisShutdown();
    // inputThread est détaché, pas besoin de join

    std::cout << GREEN << "Terminé. " << scanCount << " scans effectués." << RESET << std::endl;
    return 0;
}
