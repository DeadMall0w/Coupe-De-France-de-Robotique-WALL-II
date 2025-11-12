#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

#include "../includes/vision.h"
#include "../includes/Board.hpp"
#include "../includes/Constant.h"
#include "../includes/color.h"

using namespace std::chrono;



// =====================
// Main
// =====================

int main() {
    //juste Hello, world !
    std::cout << BOLDBLUE << "Hello, world !" << RESET << std::endl;

    //* <------ Initialisation ------->
    // bouton d'arrêt d'urgence 
    std::atomic<bool> emergencyStop = false;

    // Singleton du plateau
    Board& board = Board::instance();

    // Initialisation
    board.initialiseData("config.json");

    //* <----- tests --------->
    // std::cout << "=== Test Singleton Board ===\n";
    // std::cout << "Time left: " << board.getTimeLeft() << "\n";
    // std::cout << "State: " << static_cast<int>(board.getState()) << "\n";

    // // -------------------
    // // Déplacement robot
    // // -------------------
    // Position newPos = {3.0, 4.0};
    // if (board.moveMyRobot(newPos)) {
    //     std::cout << "My robot moved to (" 
    //               << board.getMyRobot().position.x << ", " 
    //               << board.getMyRobot().position.y << ")\n";
    // } else {
    //     std::cout << "Move failed, out of map bounds\n";
    // }

    // // Tentative de sortie du plateau
    // Position badPos = {20.0, 5.0};
    // if (!board.moveMyRobot(badPos)) {
    //     std::cout << "Move outside bounds correctly blocked\n";
    // }

    // // -------------------
    // // Modification cleats
    // // -------------------
    // Cleat c{Team::Blue};
    // if (board.setMyRobotCleat(0, c)) {
    //     std::cout << "Set cleat 0 for my robot to Blue\n";
    // }

    // // -------------------
    // // Modification zones
    // // -------------------
    // if (board.setStorageZoneState(0, ZoneState::UsedByMe)) {
    //     std::cout << "Storage zone 0 is now used by me\n";
    // }

    // if (board.setDepositZoneState(1, ZoneState::Filled)) {
    //     std::cout << "Deposit zone 1 is now filled\n";
    // }

    // board.setNidState(ZoneState::UsedByEnemy);
    // std::cout << "Nid state set to UsedByEnemy\n";

    // // -------------------
    // // Time & state
    // // -------------------
    // board.updateTime(10);
    // std::cout << "Time after 10s: " << board.getTimeLeft() << "\n";

    // board.setState(GameState::Started);
    // std::cout << "Game state now: " << static_cast<int>(board.getState()) << "\n";


    // Thread d'écoute du "bouton d'urgence" (touche Entrée)
    std::thread inputThread([&emergencyStop]() {
        std::cout << MAGENTA << "Appuis sur Entrée pour arreter le programme..." << RESET << std::endl;
        std::cin.get();  // attend l'appui sur Entrée
        emergencyStop = true;
    });

    // sauvegarde du temps
    std::atomic<uint64_t> currentTimeMs = 0;
    auto start = steady_clock::now();
    std::atomic<bool> stopVision = false;
    
    //* <----- Lancements des différents threads ---> 
    // Lancement du thread vision
    std::thread t_vision (vision, &stopVision);
    

    //* <----- Attente appuis capteur 'start' ----->
    //todo: attendre cette action
    //todo: mettre l'état en 'en cours'
    // -> nécessaire pour que tous les threads se mettent à fonctionner

    //* <----- Boucle principal ----->
    std::cout << BOLDBLUE << "Lancement de la partie...." << RESET << std::endl;
²
    // boucle qui dure le temps de la partie et qui met à jour le temps
    while (currentTimeMs <= GAME_DURATION_MS)  {

        //Todo : bouton d'urgence
        if (emergencyStop){
            break;
        }

        currentTimeMs = duration_cast<milliseconds>(steady_clock::now() - start).count();
        std::this_thread::sleep_for(1ms); // pour éviter de trop surcharger le processeur
    }

    //* <--- Fin de la partie ---->
    //todo: mettre l'état en 'fini' 
    std::cout << BOLDBLUE << "Temps terminé !" << RESET << std::endl;


    //todo: dire à tous les threads de finir ce qu'ils font et de se mettre en état d'arrêt
    stopVision = true;
    
    // attendre les threads
    if (t_vision.joinable()) t_vision.join();
    if (inputThread.joinable()) inputThread.join();
    
    //todo: se mettre en standby pour une nouvelle partie / changement de configuration

    return 0;
}
