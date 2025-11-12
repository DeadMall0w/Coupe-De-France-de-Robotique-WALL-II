#pragma once

#include <array>
#include <string>
#include <mutex>

#include "iostream"

// ================================
// Définition des types de base
// ================================

// Position dans le plan 2D
struct Position {
    double x; // coordonnée X (horizontal)
    double y; // coordonnée Y (vertical)
};

// Taille d'un objet (robot, zone, etc.)
struct Size {
    double width;  // dimension sur l'axe X
    double length; // dimension sur l'axe Y
};

// ================================
// Enums
// ================================

// Équipe du robot
enum class Team { 
    Orange, 
    Blue 
};

// État général de la partie
enum class GameState { 
    Waiting,   // en attente du début du match
    Started,   // match en cours
    Finished   // match terminé
};

// État d'une zone
enum class ZoneState { 
    Empty,       // zone vide
    Filled,      // zone contenant des éléments à ramasser/déposer
    UsedByMe,    // zone utilisée par mon robot
    UsedByEnemy  // zone utilisée par l'adversaire
};

// ================================
// Objet de jeu
// ================================

// Un élément que le robot peut tenir (ex: palets)
struct Cleat { 
    Team color; // couleur correspondant à l'équipe
};

// Une zone du plateau
struct Zone {
    Position position; // position de la zone sur le plateau
    Size size;         // taille de la zone
    ZoneState state;   // état actuel de la zone
};

// Le plateau de jeu
struct Map {
    std::array<Zone, 8> storagesZones;  // zones de ramassage
    std::array<Zone, 10> depositsZones; // zones de dépôt
    Zone nid;                            // zone centrale (nid)
};

// Robot
struct Robot {
    Position position;           // position actuelle du robot
    Size size;                   // taille du robot
    std::array<Cleat, 4> cleatsHeld; // cleats/palets tenus par le robot (max 4)
};

// ================================
// Données globales du match
// ================================

// Structure qui contient **tout l'état de la partie**
class Board {
public:
    // Accès au singleton
    static Board& instance();

    // Initialisation
    void initialiseData(const std::string& src);
    void updateTime(int delta);

    // ================================
    // Accès lecture
    // ================================
    Robot getMyRobot();
    Robot getEnemyRobot();
    Map getMap();
    int getTimeLeft();
    GameState getState();
    Team getTeam();

    Zone getStorageZone(size_t index);
    Zone getDepositZone(size_t index);
    Zone getNid();

    // ================================
    // Accès écriture / actions
    // ================================
    void setState(GameState newState);
    void setTeam(Team team);
    void setTimeLeft(int t);

    // Robot
    bool moveMyRobot(const Position& newPos);
    bool moveEnemyRobot(const Position& newPos);
    bool setMyRobotCleat(size_t slot, Cleat cleat);
    bool setEnemyRobotCleat(size_t slot, Cleat cleat);

    // Zones
    bool setStorageZoneState(size_t index, ZoneState state);
    bool setDepositZoneState(size_t index, ZoneState state);
    void setNidState(ZoneState state);

private:
    Board() = default; // constructeur privé

    // garantir le fait qu'il n'y ait qu'une seule instance
    Board(const Board&) = delete;
    Board& operator=(const Board&) = delete;

    // ================================
    // Données internes
    // ================================
    std::mutex mtx;
    int timeLeft = 100;
    Robot myRobot, enemyRobot;
    Team myTeam;
    Map map;
    GameState state = GameState::Waiting;

    // Logique interne
    bool isInsideMap(const Position& pos, const Size& size);
};