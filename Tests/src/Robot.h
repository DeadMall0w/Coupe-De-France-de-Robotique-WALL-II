#pragma once   // empêche l’inclusion multiple

#include "PlateauState.h"
#include <iostream>

class Robot {
public:
    float x, y;
    int heldBlue = 0;

    void pickup(int blue);
    void printState() const;
};
