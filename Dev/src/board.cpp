#include "../includes/Board.h"
#include <string>
#include "iostream"


Board& Board::instance() {
    static Board instance;
    return instance;
}

void Board::initialiseData(const std::string& src) {
    std::cout << "Initialisation du plateau depuis " << src << std::endl;
}




