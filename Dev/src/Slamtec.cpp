#include "../includes/Slamtec.h"
#include <iostream>



Slamtec::Slamtec(std::string _port){
    port = _port;
    //state = disconnect;
}



Slamtec::~Slamtec(){

}

void Slamtec::ChangeState(LidarState _state) {
    state = _state;
}

bool Slamtec::connect() {
    return false;
}

void Slamtec::disconnect(){
    //todo
}

bool Slamtec::startScan(){
    return false;
}

bool Slamtec::grabData(std::vector<ScanPoint>& outPoints) {
    return false;
}