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
    std::cout << BOLDBLUE << "Hello, world !" << RESET << std::endl;
    
    // bouton d'arrêt d'urgence 
    std::atomic<bool> emergencyStop = false;


    // Thread d'écoute du "bouton d'urgence" (touche Entrée)
    std::thread inputThread([&emergencyStop]() {
        std::cout << "Appuis sur Entrée pour arreter le programme...\n";
        std::cin.get();  // attend l'appui sur Entrée
        emergencyStop = true;
    });

    // sauvegarde du temps
    std::atomic<uint64_t> currentTimeMs = 0;
    auto start = steady_clock::now();
    std::atomic<bool> stopVision = false;
    
    // <----- Lancements des différents threads ---> 
    // Lancement du thread vision
    std::thread t_vision (vision, &stopVision);
    

    // <----- Attente appuis capteur 'start' ----->
    //todo: attendre cette action
    //todo: mettre l'état en 'en cours'
    // -> nécessaire pour que tous les threads se mettent à fonctionner

    // <----- Boucle principal ----->
    std::cout << BOLDBLUE << "Lancement de la partie...." << RESET << std::endl;

    // boucle qui dure le temps de la partie et qui met à jour le temps
    while (currentTimeMs <= GAME_DURATION_MS)  {

        //Todo : bouton d'urgence
        if (emergencyStop){
            break;
        }

        currentTimeMs = duration_cast<milliseconds>(steady_clock::now() - start).count();
        std::this_thread::sleep_for(1ms); // pour éviter de trop surcharger le processeur
    }

    // <--- Fin de la partie ---->
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
