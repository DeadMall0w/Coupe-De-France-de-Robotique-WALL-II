#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

#include "stratV2.hpp"
#include "../../includes/Board.hpp"
#include "../../includes/color.h"


/*
Liste des états possibles : 
- Attente que le match commence
- Déplacement vers une zone de tasseau
- récupération des tasseaux en cours
- déplacement vers une zone de dépôt
- depot des tasseaux
- recalibrage
- retournement des tasseaux en cours
- retour au départ
- fin de partie
- Arrêt urgence
- Erreur 
*/
enum class State { 
    Waiting,
    MovingToStorageZone,
    MovingToDepositZone,
    PickingUpCleats,
    DropingCleats,
    Calibrating,
    RotatingCleats,
    BackToStartArea,
    EndGame,
    AU,
    Error
};



//* CONSTANTE de pondération (à déplacer vers un fichier) 




//* <======= STRATEGIES ========>
// fonction qui gère la strategy complète
// Premiere version de la stratégie

// description de la stratégie : 
/*
Le robot regarde pour la zone de tasseau libre la plus proche
Il y va, il récupère les tasseaux et va vers la zone de dépôt la plus proche
Ce cycle continu tant qu'il y a des zones libres et qu'il reste plus de 15secondes
*/
void strategyV2(std::atomic<bool>* stop){
    std::cout << "HELLO ! (v2)" << std::endl;
    // while ()
}



