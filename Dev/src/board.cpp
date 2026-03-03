#include <iostream>
#include <fstream>

#include "../includes/Board.hpp"
#include "../includes/Constant.h"
#include "../includes/color.h"
#include "../lib/include/json.hpp"

using json = nlohmann::json;

// ================================
// Singleton instance
// ================================
Board& Board::instance() {
    static Board singletonInstance; // créé la première fois à l'accès, thread-safe depuis C++11
    return singletonInstance;
}

// ================================
// Fonctions
// ================================
void Board::initialiseData(const std::string& src) {
    std::lock_guard<std::mutex> lock(mtx);

    // Chemin fourni par l'appelant (relatif au cwd ou absolu)
    std::ifstream file(src);
    if (!file.is_open()) {
        std::cerr << RED << "[Board] Erreur : impossible d'ouvrir " << src << RESET << std::endl;
        return;
    }

    json config;
    try {
        file >> config;
    } catch (const json::parse_error& e) {
        std::cerr << RED << "[Board] Erreur de parsing JSON : " << e.what() << RESET << std::endl;
        return;
    }

    // ---- Game ----
    timeLeft = GAME_DURATION_MS / 1000;
    state = GameState::Waiting;
    if (config.contains("game")) {
        auto& game = config["game"];
        if (game.contains("team")) {
            std::string teamStr = game["team"];
            myTeam = (teamStr == "Blue") ? Team::Blue : Team::Orange;
        }
    }

    // ---- Robots ----
    if (config.contains("myRobot")) {
        auto& r = config["myRobot"];
        myRobot.position = { r["position"]["x"], r["position"]["y"] };
        myRobot.size     = { r["size"]["width"],  r["size"]["length"] };
        myRobot.orientation_deg = r.value("orientation_deg", 0.0);
    }
    if (config.contains("enemyRobot")) {
        auto& r = config["enemyRobot"];
        enemyRobot.position = { r["position"]["x"], r["position"]["y"] };
        enemyRobot.size     = { r["size"]["width"],  r["size"]["length"] };
        enemyRobot.orientation_deg = r.value("orientation_deg", 0.0);
    }

    // ---- Zones de ramassage (storageZones) ----
    for (auto& zone : map.storagesZones) {
        zone = { {0, 0}, {0, 0}, ZoneState::Empty };
    }
    if (config.contains("storageZones")) {
        auto& zones = config["storageZones"];
        for (size_t i = 0; i < zones.size() && i < map.storagesZones.size(); ++i) {
            map.storagesZones[i].position = { zones[i]["x"], zones[i]["y"] };
            map.storagesZones[i].size     = { zones[i]["width"], zones[i]["height"] };
            map.storagesZones[i].state    = ZoneState::Filled;
        }
    }

    // ---- Zones de dépôt (depositZones) ----
    for (auto& zone : map.depositsZones) {
        zone = { {0, 0}, {0, 0}, ZoneState::Empty };
    }
    if (config.contains("depositZones")) {
        auto& zones = config["depositZones"];
        for (size_t i = 0; i < zones.size() && i < map.depositsZones.size(); ++i) {
            map.depositsZones[i].position = { zones[i]["x"], zones[i]["y"] };
            map.depositsZones[i].size     = { zones[i]["width"], zones[i]["height"] };
            map.depositsZones[i].state    = ZoneState::Empty;
        }
    }

    // ---- Nid (dépend de l'équipe) ----
    std::string nidKey = (myTeam == Team::Orange) ? "nidOrange" : "nidBlue";
    if (config.contains(nidKey)) {
        auto& n = config[nidKey];
        map.nid.position = { n["x"], n["y"] };
        map.nid.size     = { n["width"], n["height"] };
        map.nid.state    = ZoneState::Empty;
    }

    std::cout << GREEN << "[Board] Initialisation OK depuis " << src << RESET << std::endl;
    std::cout << YELLOW
              << "  - " << config["storageZones"].size() << " zones de ramassage" << std::endl
              << "  - " << config["depositZones"].size() << " zones de dépôt" << std::endl
              << "  - Equipe : " << (myTeam == Team::Orange ? "Orange" : "Blue")
              << RESET << std::endl;
}

// ================================
// Lecture
// ================================
Robot Board::getMyRobot() {
    std::lock_guard<std::mutex> lock(mtx);
    return myRobot;
}

Robot Board::getEnemyRobot() {
    std::lock_guard<std::mutex> lock(mtx);
    return enemyRobot;
}

Map Board::getMap() {
    std::lock_guard<std::mutex> lock(mtx);
    return map;
}

int Board::getTimeLeft() {
    std::lock_guard<std::mutex> lock(mtx);
    return timeLeft;
}

GameState Board::getState() {
    std::lock_guard<std::mutex> lock(mtx);
    return state;
}

Team Board::getTeam() {
    std::lock_guard<std::mutex> lock(mtx);
    return myTeam;
}

// bool Board::getStorageZone(size_t index, Zone& outZone) {
//     std::lock_guard<std::mutex> lock(mtx);
//     if (index >= map.storagesZones.size()) return false;
//     outZone = map.storagesZones[index];
//     return true;
// }

// bool Board::getDepositZone(size_t index, Zone& outZone) {
//     std::lock_guard<std::mutex> lock(mtx);
//     if (index >= map.depositsZones.size()) return false;
//     outZone = map.depositsZones[index];
//     return true;
// }

Zone Board::getNid() {
    std::lock_guard<std::mutex> lock(mtx);
    return map.nid;
}

// ================================
// Écriture
// ================================
void Board::setState(GameState newState) {
    std::lock_guard<std::mutex> lock(mtx);
    state = newState;
}

void Board::setTeam(Team team) {
    std::lock_guard<std::mutex> lock(mtx);
    myTeam = team;
}

void Board::setTimeLeft(int t) {
    std::lock_guard<std::mutex> lock(mtx);
    timeLeft = std::max(0, t);
}

// todo : refaire cette fonction
void Board::updateTime(int delta) {
    std::lock_guard<std::mutex> lock(mtx);
    timeLeft -= delta;
    if (timeLeft < 0) timeLeft = 0;
}

// ================================
// Robot
// ================================
//todo : refaire à partir des constantes
 bool Board::isInsideMap(const Position& pos, const Size& size) {
    return pos.x >= 0 && pos.y >= 0 
        && pos.x + size.width  <= MAP_WIDTH_CM 
        && pos.y + size.length <= MAP_HEIGHT_CM;
}

 bool Board::moveMyRobot(const Position& newPos) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!isInsideMap(newPos, myRobot.size)) return false;
    myRobot.position = newPos;
    return true;
}

 bool Board::moveEnemyRobot(const Position& newPos) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!isInsideMap(newPos, enemyRobot.size)) return false;
    enemyRobot.position = newPos;
    return true;
}

 bool Board::setMyRobotCleat(size_t slot, Cleat cleat) {
    std::lock_guard<std::mutex> lock(mtx);
    if (slot >= myRobot.cleatsHeld.size()) return false;
    myRobot.cleatsHeld[slot] = cleat;
    return true;
}

 bool Board::setEnemyRobotCleat(size_t slot, Cleat cleat) {
    std::lock_guard<std::mutex> lock(mtx);
    if (slot >= enemyRobot.cleatsHeld.size()) return false;
    enemyRobot.cleatsHeld[slot] = cleat;
    return true;
}

// ================================
// Zones
// ================================
 bool Board::setStorageZoneState(size_t index, ZoneState state) {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= map.storagesZones.size()) return false;
    map.storagesZones[index].state = state;
    return true;
}

 bool Board::setDepositZoneState(size_t index, ZoneState state) {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= map.depositsZones.size()) return false;
    map.depositsZones[index].state = state;
    return true;
}

 void Board::setNidState(ZoneState state) {
    std::lock_guard<std::mutex> lock(mtx);
    map.nid.state = state;
}

// ================================
// Orientation
// ================================
double Board::getMyRobotOrientation() {
    std::lock_guard<std::mutex> lock(mtx);
    return myRobot.orientation_deg;
}

void Board::setMyRobotOrientation(double angle_deg) {
    std::lock_guard<std::mutex> lock(mtx);
    myRobot.orientation_deg = angle_deg;
}

// ================================
// Caméra
// ================================
CameraResult Board::getCameraResult() {
    std::lock_guard<std::mutex> lock(mtx);
    return cameraResult;
}

CameraOrder Board::getCameraOrder() {
    std::lock_guard<std::mutex> lock(mtx);
    return cameraResult.order;
}

void Board::setCameraResult(const CameraResult& result) {
    std::lock_guard<std::mutex> lock(mtx);
    cameraResult = result;
}
