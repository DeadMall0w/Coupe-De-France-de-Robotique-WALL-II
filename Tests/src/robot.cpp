#include "Robot.h"

void Robot::pickup(int blue) {
    heldBlue += blue;
    std::cout << "Picked up " << blue << " blue blocks." << std::endl;
}

void Robot::printState() const {
    std::cout << "Robot at (" << x << "," << y << "), heldBlue=" << heldBlue << std::endl;
}
