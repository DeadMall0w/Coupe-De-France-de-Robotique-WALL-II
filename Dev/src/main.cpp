#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <thread>
#include <csignal>
#include <filesystem>
#include <atomic>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;
constexpr const char* PORT = "/dev/ttyUSB0";
constexpr int BAUDRATE = 115200;
constexpr size_t MAX_NODES = 8192;
constexpr double SCAN_TIMEOUT = 0.5; // secondes max sans réponse

std::atomic<bool> keepRunning(true);

void signalHandler(int) {
    keepRunning = false;
    std::cout << "\n[INFO] Interruption reçue. Arrêt demandé.\n";
}

bool connectLidar(ILidarDriver*& lidar, IChannel*& channel) {
    if (!std::filesystem::exists(PORT)) {
        std::cerr << "[WARN] Le port " << PORT << " n'existe pas.\n";
        return false;
    }

    auto channelResult = createSerialPortChannel(PORT, BAUDRATE);
    if (SL_IS_FAIL(channelResult.err)) {
        std::cerr << "[ERROR] Impossible d'ouvrir le port série.\n";
        return false;
    }
    channel = channelResult.value;

    auto drvResult = createLidarDriver();
    if (SL_IS_FAIL(drvResult.err)) {
        std::cerr << "[ERROR] Impossible de créer le driver LIDAR.\n";
        delete channel;
        return false;
    }
    lidar = drvResult.value;

    if (SL_IS_FAIL(lidar->connect(channel))) {
        std::cerr << "[ERROR] Impossible de se connecter au LIDAR.\n";
        delete lidar;
        delete channel;
        return false;
    }

    std::cout << "[OK] LIDAR connecté sur " << PORT << "\n";
    auto res = lidar->startScan(false, true);
    if (SL_IS_FAIL(res)) {
        std::cerr << "[ERROR] Erreur au démarrage du scan.\n";
        lidar->disconnect();
        delete lidar;
        delete channel;
        return false;
    }

    std::cout << "[INFO] Scan démarré.\n";
    return true;
}

void disconnectLidar(ILidarDriver*& lidar, IChannel*& channel) {
    if (lidar) {
        std::cout << "[INFO] Arrêt du LIDAR...\n";
        lidar->stop();
        lidar->disconnect();
        delete lidar;
        lidar = nullptr;
    }
    if (channel) {
        delete channel;
        channel = nullptr;
    }
}

int main() {
    signal(SIGINT, signalHandler);
    std::cout << "hello, world!\n";
    std::cout << "Appuyez sur Entrée pour arrêter.\n";

    ILidarDriver* lidar = nullptr;
    IChannel* channel = nullptr;
    std::vector<sl_lidar_response_measurement_node_hq_t> nodes(MAX_NODES);

    // Thread d’écoute de l’entrée utilisateur (permet arrêt via Entrée)
    std::thread inputThread([]() {
        std::cin.get();
        keepRunning = false;
    });

    while (keepRunning) {
        if (!connectLidar(lidar, channel)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue; // tente de se reconnecter
        }

        size_t count = MAX_NODES;
        auto lastScanTime = std::chrono::steady_clock::now();
        bool lidar_ok = true;

        while (keepRunning && lidar_ok) {
            count = MAX_NODES;
            auto start = std::chrono::steady_clock::now();
            sl_result res = lidar->grabScanDataHq(nodes.data(), count);

            if (SL_IS_OK(res)) {
                auto now = std::chrono::steady_clock::now();
                double dt_ms = std::chrono::duration<double, std::milli>(now - lastScanTime).count();
                std::cout << "Δt = " << dt_ms << " ms\n";
                lastScanTime = now;

                // écrire dans le fichier csv les différentes points
                std::ofstream out("lidar_data.csv", std::ios::trunc);
                if (!out.is_open()) {
                    std::cerr << "[ERREUR] Impossible d'ouvrir lidar_data.csv pour écriture.\n";
                } else {
                    // Écrire l’en-tête (optionnel)
                    out << "angle_deg,dist_mm\n";
                    out << std::fixed << std::setprecision(2);

                    // Convertir et trier les points
                    lidar->ascendScanData(nodes.data(), count);

                    for (size_t i = 0; i < count; ++i) {
                        float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
                        float dist  = nodes[i].dist_mm_q2 / 4.f;
                        // ignorer les points invalides
                        if (dist > 0.1f) 
                            out << angle << "," << dist << "\n";
                    }

                    out.flush(); // force l’écriture immédiate sur disque
                }
                
            } else {
                double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                if (elapsed > SCAN_TIMEOUT) {
                    std::cerr << "[WARN] Timeout (" << elapsed << "s) – tentative de reconnexion.\n";
                    lidar_ok = false;
                }
            }

            // Si le port disparaît physiquement
            if (!std::filesystem::exists(PORT)) {
                std::cerr << "[ERROR] Déconnexion physique détectée.\n";
                lidar_ok = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        disconnectLidar(lidar, channel);
        if (keepRunning) {
            std::cerr << "[INFO] Reconnexion dans 1 seconde...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    disconnectLidar(lidar, channel);
    if (inputThread.joinable()) inputThread.join();
    std::cout << "[INFO] Programme terminé proprement.\n";
    return 0;
}
