// includes
#include "../includes/vision.h"
#include <iostream>
#include <chrono>
#include <thread>


// namespaces
using namespace std::chrono;

// headers
void runLoop(double frequency);

// script responsable de toute la gestion de la vision (lancé dans un thread séparé)


void vision(){
    // lancement du module de vision
    std::cout << "Module de vision lancé" << std::endl;



    std::cout << "Lidar 1 - OK" << std::endl;

    runLoop(10);
}

void runLoop(double frequency) {
    const auto frameDuration = duration<double>(1.0 / frequency);
    auto lastTime = steady_clock::now();

    while (true) {
        // lancement de l'horloge pour compter le temps
        auto start = steady_clock::now();
        lastTime = start;

        //task(delta); // exécute la tâche

        auto elapsed = steady_clock::now() - start;
        if (elapsed < frameDuration)
            std::this_thread::sleep_for(frameDuration - elapsed);
        else 
            std::cout << "[⚠️  TEMPS DÉPASSÉ] "
                      << "Durée = " << duration_cast<milliseconds>(elapsed).count()
                      << " ms  (Budget = "
                      << duration_cast<milliseconds>(frameDuration).count()
                      << " ms)"
                      << std::endl;
    }
}