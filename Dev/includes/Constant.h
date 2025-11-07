// ce fichier à pour but de contenir toutes les constantes pour éviter les 'magic number' 
// Il contient notamment
/*
Le temps d'une partie

*/
#pragma once
// permet d'être plus libre sur les types de variable utilisés (uint64_t, ...)
#include <cstdint>

// constexpr permet d'indiquer au compilateur de précalculer cette valeur lors de la compilation (donc le minimum d'impact sur les performances possibles)
constexpr uint64_t GAME_DURATION_MS = 100 * 1000; // 100 secondes 


