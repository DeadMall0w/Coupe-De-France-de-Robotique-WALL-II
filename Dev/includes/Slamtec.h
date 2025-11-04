#pragma once

#include "../includes/ILidar.h"
#include <string>
#include <vector>

class Slamtec : public ILidar
{
private:
    std::string port;
    // constexpr ~ const (mais calculé lors de la compilation -> plus optimisé)
    static constexpr int BAUDRATE = 115200; // baudrate de base
    static constexpr int MAX_NODES = 8192;
    static constexpr double SCAN_TIMEOUT = 0.5; // temps en secondes max sans réponse (sinon on le considère en timeout)
    
    LidarState state;

    void ChangeState(LidarState _state);


public:
    Slamtec(std::string _port);
    ~Slamtec();


    bool connect() override;
    void disconnect() override;
    bool startScan() override;
    bool grabData(std::vector<ScanPoint>& outPoints) override;
};

