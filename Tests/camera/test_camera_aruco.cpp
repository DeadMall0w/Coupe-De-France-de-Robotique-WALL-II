// Test standalone — détection ArUco via CameraProcessing
//
// Usage :
//   ./test_camera_aruco                    → logs console SSH (Ctrl+C pour quitter)
//   ./test_camera_aruco --serve 8080       → serveur MJPEG HTTP sur :8080
//       Client : ffplay http://IP:8080   ou   vlc http://IP:8080
//   ./test_camera_aruco --stream           → MJPEG brut sur stdout
//       Client : ssh pi@robot "./test_camera_aruco --stream" | ffplay -f mjpeg -i -
//   ./test_camera_aruco v4l2               → forcer V4L2 (/dev/video1)
//   ./test_camera_aruco --display          → fenêtre OpenCV locale (si DISPLAY dispo)

#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>
#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <cstring>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "../../Dev/includes/CameraProcessing.h"
#include "../../Dev/includes/color.h"

using namespace std;
using namespace std::chrono;

// -----------------------------------------------
// Gestion propre de Ctrl+C
// -----------------------------------------------
static volatile bool g_running = true;
static void sigHandler(int) { g_running = false; }

// -----------------------------------------------
// Helpers texte
// -----------------------------------------------
static const char* orderToString(CameraOrder o) {
    switch (o) {
        case CameraOrder::Recherche:         return "RECHERCHE      ";
        case CameraOrder::Avancer:           return "AVANCER        ";
        case CameraOrder::Reculer:           return "RECULER        ";
        case CameraOrder::Gauche:            return "GAUCHE         ";
        case CameraOrder::Droite:            return "DROITE         ";
        case CameraOrder::TourneHoraire:     return "TOURNE ->      ";
        case CameraOrder::TourneAntiHoraire: return "TOURNE <-      ";
        case CameraOrder::Parfait:           return "*** PARFAIT ***";
        default:                             return "INCONNU        ";
    }
}
static const char* colorToString(DetectedColor c) {
    switch (c) {
        case DetectedColor::Yellow: return "JAUNE";
        case DetectedColor::Blue:   return "BLEU ";
        default:                    return "-----";
    }
}

// -----------------------------------------------
// Serveur MJPEG HTTP minimaliste (un seul client)
// -----------------------------------------------
static int mjpeg_server_fd = -1;
static int mjpeg_client_fd = -1;

static bool mjpeg_server_init(int port) {
    mjpeg_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (mjpeg_server_fd < 0) return false;
    int opt = 1;
    setsockopt(mjpeg_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    fcntl(mjpeg_server_fd, F_SETFL, O_NONBLOCK);
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port);
    if (bind(mjpeg_server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) return false;
    listen(mjpeg_server_fd, 1);
    return true;
}

static void mjpeg_accept_client() {
    sockaddr_in cli{};
    socklen_t len = sizeof(cli);
    int fd = accept(mjpeg_server_fd, (sockaddr*)&cli, &len);
    if (fd < 0) return;
    if (mjpeg_client_fd >= 0) close(mjpeg_client_fd);
    mjpeg_client_fd = fd;
    const char* hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    send(mjpeg_client_fd, hdr, strlen(hdr), MSG_NOSIGNAL);
    char ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &cli.sin_addr, ip, sizeof(ip));
    cerr << GREEN << "[Stream] Client connecte : " << ip << RESET << endl;
}

static bool mjpeg_send_frame(const vector<uchar>& jpg) {
    if (mjpeg_client_fd < 0) return false;
    char hdr[128];
    int hlen = snprintf(hdr, sizeof(hdr),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", jpg.size());
    if (send(mjpeg_client_fd, hdr,        hlen,       MSG_NOSIGNAL) < 0) goto dc;
    if (send(mjpeg_client_fd, jpg.data(), jpg.size(), MSG_NOSIGNAL) < 0) goto dc;
    if (send(mjpeg_client_fd, "\r\n",     2,          MSG_NOSIGNAL) < 0) goto dc;
    return true;
dc:
    cerr << YELLOW << "[Stream] Client deconnecte." << RESET << endl;
    close(mjpeg_client_fd); mjpeg_client_fd = -1;
    return false;
}

// -----------------------------------------------
// main
// -----------------------------------------------
int main(int argc, char* argv[]) {
    signal(SIGINT,  sigHandler);
    signal(SIGTERM, sigHandler);
    signal(SIGPIPE, SIG_IGN);

    bool useGstreamer = true;
    bool showDisplay  = false;
    bool streamStdout = false;
    int  servePort    = -1;

    for (int i = 1; i < argc; i++) {
        string arg(argv[i]);
        if (arg == "v4l2")      useGstreamer = false;
        if (arg == "--display") showDisplay  = true;
        if (arg == "--stream")  streamStdout = true;
        if (arg == "--serve" && i + 1 < argc) servePort = atoi(argv[++i]);
    }

    if (showDisplay) {
        bool hasDisplay = getenv("DISPLAY") || getenv("WAYLAND_DISPLAY");
        if (!hasDisplay) {
            cerr << YELLOW << "[Display] Pas de DISPLAY/WAYLAND, --display ignore." << RESET << endl;
            showDisplay = false;
        }
    }

    bool logToStderr = streamStdout;
    auto log = [&](const string& msg) {
        if (logToStderr) cerr << msg << endl;
        else             cout << msg << endl;
    };

    log(string(BOLDBLUE) + "=== WALL-II Camera ArUco ===" + RESET);
    log(string(YELLOW) + "Mode : " + (useGstreamer ? "GStreamer" : "V4L2") + RESET);

    if (servePort > 0) {
        if (!mjpeg_server_init(servePort)) {
            cerr << RED << "Impossible d'ouvrir le port " << servePort << RESET << endl;
            return 1;
        }
        log(string(GREEN)  + "Serveur MJPEG : http://<IP>:" + to_string(servePort) + RESET);
        log(string(YELLOW) + "  -> ffplay http://<IP>:" + to_string(servePort) + RESET);
    } else if (streamStdout) {
        log(string(GREEN)  + "Stream MJPEG sur stdout (pipe SSH)" + RESET);
        log(string(YELLOW) + "  -> ssh pi@robot './test_camera_aruco --stream' | ffplay -f mjpeg -i -" + RESET);
    }

    CameraProcessing cam;
    if (!(useGstreamer ? cam.init(-1) : cam.init(1))) {
        cerr << RED << "Echec ouverture camera." << RESET << endl;
        return 1;
    }

    bool needDebug = showDisplay || streamStdout || (servePort > 0);
    cam.setDebug(needDebug);

    log(string(GREEN) + "Camera OK. Ctrl+C pour quitter." + RESET);

    int  frameCount = 0, detCount = 0;
    auto startTime  = steady_clock::now();
    constexpr int PRINT_EVERY = 5, NEWLINE_EVERY = 30;

    while (g_running) {
        auto t1 = steady_clock::now();
        CameraResult result = cam.processNextFrame();
        auto t2 = steady_clock::now();

        double ms = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
        frameCount++;
        if (result.objectDetected) detCount++;

        // Encode la frame annotee en JPEG
        vector<uchar> jpegBuf;
        if (needDebug) {
            cv::Mat dbg = cam.getDebugFrame();
            if (!dbg.empty()) {
                vector<int> p = { cv::IMWRITE_JPEG_QUALITY, 80 };
                cv::imencode(".jpg", dbg, jpegBuf, p);
            }
        }

        // Fenetre locale
        if (showDisplay && !jpegBuf.empty()) {
            cv::imshow("WALL-II Camera ArUco", cam.getDebugFrame());
            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) break;
        }

        // Stream stdout
        if (streamStdout && !jpegBuf.empty()) {
            fwrite(jpegBuf.data(), 1, jpegBuf.size(), stdout);
            fflush(stdout);
        }

        // Serveur HTTP
        if (servePort > 0) {
            mjpeg_accept_client();
            if (!jpegBuf.empty()) mjpeg_send_frame(jpegBuf);
        }

        // Logs console
        if (frameCount % PRINT_EVERY == 0) {
            double elapsed = duration_cast<milliseconds>(t2 - startTime).count() / 1000.0;
            double fps     = elapsed > 0 ? frameCount / elapsed : 0.0;
            double detRate = frameCount > 0 ? 100.0 * detCount / frameCount : 0.0;

            const DetectedCrate* best = result.closest();

            // Ligne de résumé : ordre global groupe + détail par tasseau
            char buf[512];
            if (result.objectDetected && !result.crates.empty()) {
                int off = snprintf(buf, sizeof(buf),
                    "[%5d] %s %dt | GrpZ:%4.0f X:%+4.0f cap:%+3.0f |",
                    frameCount, orderToString(result.groupOrder),
                    (int)result.crates.size(),
                    result.groupCenterZ, result.groupCenterX, result.groupAngle);

                // Détail de chaque tasseau (triés par X, gauche→droite)
                vector<const DetectedCrate*> sorted;
                for (const auto& c : result.crates) sorted.push_back(&c);
                sort(sorted.begin(), sorted.end(),
                     [](const DetectedCrate* a, const DetectedCrate* b){ return a->smoothX < b->smoothX; });

                for (const auto* c : sorted) {
                    const char* col_c = (c->color == DetectedColor::Blue) ? "B" : "J";
                    off += snprintf(buf + off, sizeof(buf) - off,
                        " %s Z:%4.0f X:%+4.0f |",
                        col_c, c->smoothZ, c->smoothX);
                }
                snprintf(buf + off, sizeof(buf) - off,
                    " %5.1fms %4.1ffps det:%4.1f%%",
                    ms, fps, detRate);
            } else {
                snprintf(buf, sizeof(buf),
                    "[%5d] RECHERCHE...                                         |  %5.1fms  %4.1ffps  det:%4.1f%%",
                    frameCount, ms, fps, detRate);
            }

            const char* col  = result.objectDetected ? GREEN : YELLOW;
            const char* bold = (result.groupOrder == CameraOrder::Parfait) ? BOLDGREEN : col;

            if (frameCount % NEWLINE_EVERY == 0) {
                double ts = duration_cast<milliseconds>(t2 - startTime).count() / 1000.0;
                char ts_buf[280];
                snprintf(ts_buf, sizeof(ts_buf), "[+%6.1fs] %s", ts, buf);
                cerr << bold << ts_buf << RESET << "\n";
            } else {
                cerr << "\r" << col << buf << RESET << "   ";
            }
            cerr.flush();
        }
    }

    cerr << "\n" << YELLOW << "Arret." << RESET << "\n";

    if (showDisplay) cv::destroyAllWindows();
    if (mjpeg_client_fd >= 0) close(mjpeg_client_fd);
    if (mjpeg_server_fd >= 0) close(mjpeg_server_fd);
    cam.release();

    double elapsed = duration_cast<milliseconds>(steady_clock::now() - startTime).count() / 1000.0;
    cerr << BOLDBLUE << "\n=== Resume ===" << RESET << "\n"
         << "  Frames  : " << frameCount << "\n"
         << "  Detect. : " << detCount
         << " (" << fixed << setprecision(1)
         << (frameCount > 0 ? 100.0 * detCount / frameCount : 0.0) << "%)\n"
         << "  Duree   : " << elapsed << "s\n"
         << "  FPS moy.: " << (elapsed > 0 ? frameCount / elapsed : 0.0) << "\n";

    return 0;
}
