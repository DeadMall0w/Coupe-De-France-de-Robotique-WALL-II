#pragma once

#include "../includes/ILidar.h"
#include <string>
#include <vector>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;


class Slamtec : public ILidar
{
private:

    // <--- constantes ---->
    // constexpr ~ const (mais calculé lors de la compilation -> plus optimisé)
    static constexpr int BAUDRATE = 115200; // baudrate de base
    static constexpr int MAX_NODES = 8192;
    static constexpr double SCAN_TIMEOUT = 0.5; // temps en secondes max sans réponse (sinon on le considère en timeout)

    std::string port; // port de connection du lidar (souvent : /dev/ttyUSB0)
    LidarState state; // etat actuel du lidar
    
    ILidarDriver* driver; 
    IChannel* channel;




    void ChangeState(LidarState _state) override;


public:
    Slamtec(std::string _port);
    ~Slamtec();


    bool connect() override;
    void disconnect() override;
    bool startScan() override;
    bool grabData(std::vector<ScanPoint>& outPoints) override;
};

