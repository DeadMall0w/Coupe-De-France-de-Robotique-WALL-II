#include "../includes/Board.h"
#include <thread>
#include <chrono>
#include <iostream>

#include "rplidar.h"         // inclut les définitions de base
#include "sl_lidar_driver.h"  // pour piloter le LIDAR

using namespace rp::standalone::rplidar;


using namespace std::chrono;

void runLoop(double frequency, void (*task)(double)) {
    const auto frameDuration = duration<double>(1.0 / frequency);
    auto lastTime = steady_clock::now();

    while (true) {
        // lancement de l'horloge pour compter le temps
        auto start = steady_clock::now();
        double delta = duration<double>(start - lastTime).count();
        lastTime = start;

        task(delta); // exécute la tâche

        auto elapsed = steady_clock::now() - start;
        if (elapsed < frameDuration)
            std::this_thread::sleep_for(frameDuration - elapsed);
        else 
            std::cout << "[⚠️  TEMPS DÉPASSÉ] "
                      << "Durée = " << duration_cast<milliseconds>(elapsed).count()
                      << " ms  (Budget = "
                      << duration_cast<milliseconds>(frameDuration).count()
                      << " ms)"
                      << std::endl;
    }
}

// =====================
// Tâches exemples
// =====================

void strategie(double dt) {
    // décision 
}

void command(double dt) {
    std::cout << "[Commandes] Δt=" << dt << "s\n";
    // génération d'ordres moteur, envoi UART
}

void lidar(double dt) {
    std::cout << "[Lidar] Δt=" << dt << "s\n";
    // traitement lidar haut/bas
}

void logCom(double dt) {
    std::cout << "[Log/Unity] Δt=" << dt << "s\n";
    // transmission des logs ou état vers Unity
}

// =====================
// Main
// =====================

int main() {
//     std::thread t_strategy (runLoop, 10.0, strategie); // 100ms 
//     // std::thread t2(runLoop, 10.0, command);
//     // std::thread t3(runLoop, 10.0, lidar);
//     std::thread t_com(runLoop, 10.0, logCom);

//     while (1){
//         printf("Hello world ! \n");
// std::this_thread::sleep_for(50ms);

//     }

//     t_strategy.join();
    // t2.join();
    // t3.join();
    // t4.join();
   ///  Create a communication channel instance
    IChannel* _channel;
    Result<ISerialChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);
    ///  Create a LIDAR driver instance
    ILidarDriver * l idar = *createLidarDriver();
    auto res = (*lidar)->connect(*channel);
    if(SL_IS_OK(res)){
        sl_lidar_response_device_info_t deviceInfo;
        res = (*lidar)->getDeviceInfo(deviceInfo);
        if(SL_IS_OK(res)){
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
            deviceInfo.model,
            deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
            deviceInfo.hardware_version);
        }else{
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    }else{
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
    // TODO
	
    /// Delete Lidar Driver and channel Instance
    * delete *lidar;
    * delete *channel;



}
