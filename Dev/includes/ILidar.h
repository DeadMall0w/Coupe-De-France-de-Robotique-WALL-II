#pragma once
#include <string>
#include <vector>

enum LidarState {
    working, // en cours d'utilisation
    ready, // pret à être lancé
    inInitialisation, // en cours d'initialisation
    disconnected, // déconnecté
    stopped // A l'arrêt
};

struct ScanPoint {
    float angle_deg;
    float dist_mm;
};

class ILidar {
public:
    virtual ~ILidar() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool startScan() = 0;
    virtual bool grabData(std::vector<ScanPoint>& outPoints) = 0;


private:
    virtual void ChangeState(LidarState state);
    
};



