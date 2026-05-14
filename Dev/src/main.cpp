#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

#include "../includes/vision.h"
#include "../includes/camera.h"
#include "../includes/chassis.h"
// #include "../includes/strategy.hpp"
#include "../includes/Board.hpp"
#include "../includes/Constant.h"
#include "../includes/color.h"



//* <==== Importations des différentes stratégies ====>
#include "strategies/stratV1.hpp"
#include "strategies/stratV2.hpp"


using namespace std::chrono;



int main(int argc, char *argv[]) {
    //juste Hello, world !
    std::cout << BOLDBLUE << "Hello, world !" << RESET << std::endl;
    

    int strategyCode = 1; // valeur par défaut
    if (argc == 2){
        std::cout << BOLDBLUE << "Vous avez choisi la stratégie " << argv[1] << RESET << std::endl;
        try {
            int val = std::stoi(argv[1]);
            if (val > 0) {
            strategyCode = val;
            } else {
            std::cout << BOLDRED << "Code stratégie invalide, utilisation de la stratégie 1." << RESET << std::endl;
            }
        } catch (...) {
            std::cout << BOLDRED << "Entrée non numérique, utilisation de la stratégie 1." << RESET << std::endl;
        }   
    }else {
        std::cout << BOLDBLUE << "Utilisation de la stratégie 1" << RESET << std::endl;
    }

    //* <------ Initialisation ------->
    // bouton d'arrêt d'urgence 
    std::atomic<bool> emergencyStop = false;

    // Bouton départ
    std::atomic<bool> depart = false;


    // Singleton du plateau
    Board& board = Board::instance();

    // Initialisation
    board.initialiseData("data/config.json");

    // Calibration repère châssis:
    // Teensy (0,0,0) -> Plateau (150,25,-90)
    chassisSetBoardReferencePose(150.0, 25.0, -90.0);
    // Correction orientation axes locaux OTOS/Teensy vers plateau
    chassisSetLocalAxesSigns(1.0, -1.0);

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
    std::thread inputThread([&emergencyStop, &board, &depart]() {
        // TODO : Changer ca part l'appui sur le bouton départ
        std::cin.get();  // attend l'appui sur Entrée
        depart = true;        

        // TODO : Changer ca part l'appui sur le bouton arrêt d'urgence
        std::cin.get();  // attend l'appui sur Entrée
        emergencyStop = true;
    });

    //* <----- Lancements des différents threads --->
    std::atomic<bool> stopVision = false;
    std::atomic<bool> stopCamera = false;
    std::atomic<bool> stopChassis = false;
    std::atomic<bool> stopStrategy = false;

    

    // Lancement du thread vision
    std::thread t_vision (vision, &stopVision);
    
    // Lancement du thread caméra
    std::thread t_camera (camera, &stopCamera);
    
        // Lancement du thread communication châssis (UART Teensy)
        std::thread t_chassis (chassis, &stopChassis);

    std::thread t_strategy;

    switch (strategyCode) {
        case 1:
            t_strategy = std::thread(strategyV1, &stopStrategy);
            break;
        case 2:
            t_strategy = std::thread(strategyV2, &stopStrategy);
            break;
        default:
            std::cout << RED << "Code invalide -> Utilisation stratégie 1" << RESET << std::endl;
            t_strategy = std::thread(strategyV1, &stopStrategy);
            break;
    }    
    std::cout << "En attente de lancement..." << std::endl;
    
    //* <----- Boucle principal ----->
    while (depart == false){
        ; // ne rien faire
    }
    board.setState(GameState::Started);

    // Lancement des timer
    std::atomic<uint64_t> currentTimeMs = 0;
    auto start = steady_clock::now();

    std::cout << BOLDBLUE << "Lancement de la partie !" << RESET << std::endl;

    // boucle qui dure le temps de la partie et qui met à jour le temps
    while (currentTimeMs <= GAME_DURATION_MS)  {

        //Todo : bouton d'urgence
        if (emergencyStop){
            std::cout << RED << "ARRÊT D'URGENCE ENCLENCHE !" << RESET << std::endl; 
            break;
        }

        // TODO : Mettre le robot en erreur si un des threads critique est arrêté ou qu'il y a un défaut
        //* ex : On ne peut pas faire une partie si le lidar est off 

        currentTimeMs = duration_cast<milliseconds>(steady_clock::now() - start).count();
        std::this_thread::sleep_for(1ms); // pour éviter de trop surcharger le processeur

        // Affichage d'état toutes les 10 secondes
        static uint64_t lastStatusPrint = 0;
        if (currentTimeMs - lastStatusPrint >= 10000 || lastStatusPrint == 0) {
            lastStatusPrint = currentTimeMs;

            std::cout << BOLDYELLOW << "\n===== [DEBUG STATUS] =====\n" << RESET;

            // Temps restant
            uint64_t timeLeft = (GAME_DURATION_MS > currentTimeMs) ? (GAME_DURATION_MS - currentTimeMs) : 0;
            std::cout << CYAN << "Temps écoulé: " << currentTimeMs / 1000 << "s / " << GAME_DURATION_MS / 1000 << "s\n";
            std::cout << CYAN << "Temps restant: " << timeLeft / 1000 << "s\n" << RESET;

            // Etat du bouton d'urgence
            std::cout << (emergencyStop ? RED : GREEN)
                      << "Bouton d'urgence: " << (emergencyStop ? "ACTIVÉ" : "OK") << RESET << std::endl;

            // Etat des threads
            std::cout << MAGENTA << "Threads:\n" << RESET;
            std::cout << "  Vision: " << (stopVision ? RED "ARRÊTÉ" RESET : GREEN "EN COURS" RESET) << std::endl;
            std::cout << "  Camera: " << (stopCamera ? RED "ARRÊTÉ" RESET : GREEN "EN COURS" RESET) << std::endl;
            std::cout << "  Input:  " << (inputThread.joinable() ? GREEN "EN COURS" RESET : RED "ARRÊTÉ" RESET) << std::endl;

            // Etat du plateau
            std::cout << YELLOW << "Etat du plateau:\n" << RESET;
            std::cout << "  GameState: " << static_cast<int>(board.getState()) << std::endl;
            std::cout << "  MyRobot: (" << board.getMyRobot().position.x << ", " << board.getMyRobot().position.y << ")\n";
            std::cout << "  TimeLeft (Board): " << board.getTimeLeft() << std::endl;

            // Ajoute ici d'autres infos utiles si besoin

            std::cout << BOLDYELLOW << "=========================\n" << RESET;
        }
    }

    //* <--- Fin de la partie ---->
    board.setState(GameState::Finished); // Partie finie
    std::cout << BOLDBLUE << "FIN !" << RESET << std::endl;


    //todo: dire à tous les threads de finir ce qu'ils font et de se mettre en état d'arrêt
    stopVision = true;
    stopCamera = true;
    stopChassis = true;
    
    // attendre les threads
    if (t_vision.joinable()) t_vision.join();
    if (t_camera.joinable()) t_camera.join();
    if (t_chassis.joinable()) t_chassis.join();
    if (inputThread.joinable()) inputThread.join();


    std::cout << GREEN << "Arrêt réussi" << RESET << std::endl;
    
    //todo: se mettre en standby pour une nouvelle partie / changement de configuration

    return 0;
}
