#include <iostream>

#include "../includes/Board.hpp"
#include "../includes/Constant.h"
#include "../includes/color.h"

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
    // exemple d'initialisation (tu peux parser un fichier ou config)
    // std::cout << "Initialisation avec : " << src << std::endl;
    // timeLeft = GAME_DURATION_MS / 1000; // convertir en second
    // state = GameState::Waiting;
    // myTeam = Team::Orange;

    // // initialisation map et robots (exemple)
    // map.nid.position = {0.0, 0.0};
    // for (auto& zone : map.storagesZones) {
    //     zone.state = ZoneState::Empty;
    // }
    // for (auto& zone : map.depositsZones) {
    //     zone.state = ZoneState::Empty;
    // }

    // myRobot.position = {1.0, 1.0};
    // enemyRobot.position = {2.0, 2.0};

    //todo : faire l'initialisation
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
    // simple logique: map de 10x10
    return pos.x >= 0 && pos.y >= 0 && pos.x + size.width <= 10.0 && pos.y + size.length <= 10.0;
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