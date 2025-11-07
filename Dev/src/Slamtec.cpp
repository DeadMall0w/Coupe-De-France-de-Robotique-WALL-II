#include <iostream>
#include <filesystem>

#include "../includes/Slamtec.h"
#include "../includes/color.h"


Slamtec::Slamtec(std::string _port){
    port = _port;
    //state = disconnect;
}



Slamtec::~Slamtec(){
    this->disconnect();
}

bool Slamtec::connect() {
    this->ChangeState(LidarState::connecting);

    if (!std::filesystem::exists(this->port)) {
        this->ChangeState(LidarState::error);
        std::cout << RED << "[ERROR] Le port " << this->port << " n'existe pas." << RESET << std::endl;
        return false;
    }

    auto channelResult = createSerialPortChannel(this->port, this->BAUDRATE);
    if (SL_IS_FAIL(channelResult.err)) {
        this->ChangeState(LidarState::error);
        std::cout << RED << "[ERROR] Impossible d'ouvrir le port "<< this->port << "." << RESET << std::endl;
        return false;
    }
    this->channel = channelResult.value;

    auto drvResult = createLidarDriver();
    if (SL_IS_FAIL(drvResult.err)) {
        this->ChangeState(LidarState::error);
        std::cout << RED << "[ERROR] Impossible de créer le driver LIDAR."  << RESET << std::endl;
        delete this->channel;
        return false;
    }
    this->driver = drvResult.value;

    if (SL_IS_FAIL(this->driver->connect(this->channel))) {
        this->ChangeState(LidarState::error);
        std::cout << RED << "[ERROR] Impossible de se connecter au LIDAR."  << RESET << std::endl;
        delete this->driver;
        delete this->channel;
        return false;
    }

    std::cout << GREEN << "[OK] LIDAR connecté sur " << this->port << RESET << std::endl;
    
    this->ChangeState(LidarState::ready);
    return true;
}

void Slamtec::ChangeState(LidarState _state) {
    this->state = _state;
}

void Slamtec::disconnect(){
    this->ChangeState(LidarState::disconnected);
    if (this->driver) {
        std::cout << YELLOW << "[INFO] Arrêt du LIDAR..." << RESET << std::endl;
        this->driver->stop();
        this->driver->disconnect();
        delete this->driver;
        this->driver = nullptr;
        std::cout << YELLOW << "[INFO] Lidar déconnecté" << RESET << std::endl;
    }
    if (this->channel) {
        delete this->channel;
        this->channel = nullptr;
    }
}

bool Slamtec::startScan() {
    if (this->state != LidarState::ready) {
        this->ChangeState(LidarState::error);
        std::cout << RED << "[ERROR] Impossible de lancer un scan, lidar en état : " << this->state << RESET << std::endl;
        return false;
    }

    auto res = this->driver->startScan(false, true);
    if (SL_IS_FAIL(res)) {
        std::cerr << "[ERROR] Erreur au démarrage du scan, code : " << res << "\n";
        this->ChangeState(LidarState::error);
        this->disconnect();
        return false;
    }

        this->ChangeState(LidarState::working);
    std::cout << GREEN << "[OK] Scan démarré." << RESET << std::endl;
    return true;
}

bool Slamtec::grabData(std::vector<ScanPoint>& outPoints) {
    if (!this->driver || this->state != LidarState::working) return false;

    size_t count = MAX_NODES;
    std::vector<sl_lidar_response_measurement_node_hq_t> nodes(MAX_NODES);

    sl_result res = driver->grabScanDataHq(nodes.data(), count);

    if (!SL_IS_OK(res)) {
        // Timeout ou erreur
        return false;
    }


    // Trier les données par angle
    driver->ascendScanData(nodes.data(), count);

    // Remplir outPoints
    outPoints.clear();
    for (size_t i = 0; i < count; ++i) {
        float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
        float dist  = nodes[i].dist_mm_q2 / 4.f;
        if (dist > 0.1f) {
            outPoints.push_back({angle, dist});
        }
    }

    return true; // nouvelles données
}