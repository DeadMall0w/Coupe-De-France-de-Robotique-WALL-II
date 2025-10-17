#include "Robot.h"
#include "PlateauState.h"

int main() {
    Robot r;
    PlateauState plateau;

    r.x = 5; r.y = 3;
    r.pickup(2);
    r.printState();

    std::cout << "Time remaining: " << plateau.timeRemaining << std::endl;

    return 0;
}
