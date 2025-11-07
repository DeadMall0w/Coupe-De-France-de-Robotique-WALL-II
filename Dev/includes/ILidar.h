#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

enum LidarState {
    disconnected, // déconnecté
    disconnecting, // en train de se déocnnecter
    connecting, // en cours d'initialisation
    ready, // pret à être lancé
    working, // en cours d'utilisation
    stopped, // A l'arrêt
    error, // une erreur est survenue
};


struct ScanPoint {
    float angle_deg;
    float dist_mm;
};


struct ScanData {
    std::vector<ScanPoint> points;
    std::chrono::steady_clock::time_point timestamp;
};

class ILidar {
public:
    virtual ~ILidar() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool startScan() = 0;
    virtual bool grabData(std::vector<ScanPoint>& outPoints) = 0;


private:
    virtual void ChangeState(LidarState state) = 0;

    
};


namespace LidarUtils {

    inline void writeScanToCSV(const std::vector<ScanPoint>& points, const std::string& filename = "lidar_data.csv") {
        std::ofstream out(filename, std::ios::trunc);
        if (!out.is_open()) {
            std::cerr << "[ERREUR] Impossible d'ouvrir " << filename << " pour écriture.\n";
            return;
        }

        out << "angle_deg,dist_mm\n";
        out << std::fixed << std::setprecision(2);

        for (const auto& p : points) {
            if (p.dist_mm > 0.1f)
                out << p.angle_deg << "," << p.dist_mm << "\n";
        }

        out.flush();
    }

}



