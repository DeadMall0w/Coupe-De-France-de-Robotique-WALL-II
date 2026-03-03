#pragma once

#include <atomic>

// Point d'entrée du thread caméra
// Similaire à vision() pour le LIDAR
void camera(std::atomic<bool>* stop);
