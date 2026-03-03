// ce fichier à pour but de contenir toutes les constantes pour éviter les 'magic number' 
// Il contient notamment
/*
Le temps d'une partie
Les dimensions du plateau
La configuration de la détection du robot adverse
Les potentiels angles morts (TODO : à vérifier si besoin)
*/
#pragma once
// permet d'être plus libre sur les types de variable utilisés (uint64_t, ...)
#include <cstdint>

// constexpr permet d'indiquer au compilateur de précalculer cette valeur lors de la compilation (donc le minimum d'impact sur les performances possibles)
constexpr uint64_t GAME_DURATION_MS = 100 * 1000; // 100 secondes 

// ================================
// Dimensions du plateau (en cm)
// ================================
constexpr double MAP_WIDTH_CM  = 300.0;  // largeur du plateau (axe X)
constexpr double MAP_HEIGHT_CM = 200.0;  // hauteur du plateau (axe Y)
constexpr double BORDER_HEIGHT_CM = 7.0; // hauteur des bordures du plateau
constexpr double LIDAR_HEIGHT_CM  = 17.0; // hauteur approximative du LIDAR sur le robot

// ================================
// Paramètres de traitement LIDAR
// ================================
constexpr double ENEMY_ROBOT_RADIUS_CM = 25.0;  // rayon approximatif du robot adverse (pour clustering)
constexpr double ZONE_PROXIMITY_CM     = 20.0;  // distance pour considérer le robot adverse "dans" une zone
constexpr double CLUSTER_MAX_GAP_CM    = 30.0;  // distance max entre 2 points pour appartenir au même cluster
constexpr int    MIN_CLUSTER_POINTS    = 3;      // nb minimum de points pour qu'un cluster soit considéré valide

// TODO : vérifier si besoin et configurer
// ================================
// Angles morts du LIDAR (en degrés)
// ================================
// Le LIDAR est entouré d'une structure : certains angles sont bloqués.
// Chaque angle mort est défini par [début, fin] en degrés (0-360, sens trigonométrique).
// Ces valeurs sont à calibrer sur le robot réel.
// Pour l'instant, 4 angles morts de 10° chacun, répartis à 90° d'intervalle.
#include <array>
#include <utility>

constexpr size_t BLIND_SPOT_COUNT = 4;
constexpr std::array<std::pair<double, double>, BLIND_SPOT_COUNT> LIDAR_BLIND_SPOTS = {{
    {40.0,  50.0},   // pilier avant-droit (à calibrer)
    {130.0, 140.0},  // pilier arrière-droit (à calibrer)
    {220.0, 230.0},  // pilier arrière-gauche (à calibrer)
    {310.0, 320.0}   // pilier avant-gauche (à calibrer)
}};
