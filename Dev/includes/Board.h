#pragma once
#include <array>
#include <string>
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
    static Board& instance();

    void initialiseData(const std::string& src);
    void updateTime(int delta);

    // Accès lecture
    const Robot& getMyRobot() const { return myRobot; }
    const Robot& getEnemyRobot() const { return enemyRobot; }
    const Map& getMap() const { return map; }
    int getTimeLeft() const { return timeLeft; }
    GameState getState() const { return state; }

    // Accès écriture contrôlée
    void setState(GameState newState) { state = newState; }
    void setTeam(Team team) { myTeam = team; }

    // Fonctions d’action
    Robot& myRobotRef() { return myRobot; } // si tu veux un accès modifiable

private:
    Board() = default;

    int timeLeft = 100;
    Robot myRobot, enemyRobot;
    Team myTeam;
    Map map;
    GameState state = GameState::Waiting;
};

