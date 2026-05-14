#include "../includes/chassis.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <algorithm>

#include "../includes/color.h"

namespace {
constexpr double CM_PER_INCH = 2.54;
constexpr double PI_LOCAL = 3.14159265358979323846;
constexpr int DEFAULT_TIMEOUT_MS = 400;
constexpr int POSE_POLL_PERIOD_MS = 80;

std::mutex gChassisMutex;
int gFd = -1;
std::string gDevice = "/dev/ttyAMA3";
int gBaudRate = 115200;
Position gBoardRefPosCm{0.0, 0.0};
double gBoardRefHeadingDeg = 0.0;
double gLocalXSign = 1.0;
double gLocalYSign = 1.0;

double normalizeAngleDeg(double angleDeg) {
    while (angleDeg > 180.0) {
        angleDeg -= 360.0;
    }
    while (angleDeg <= -180.0) {
        angleDeg += 360.0;
    }
    return angleDeg;
}

speed_t toTermiosBaud(int baudRate) {
    switch (baudRate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return B115200;
    }
}

bool configureUart(int fd, int baudRate) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << RED << "[Chassis] tcgetattr: " << std::strerror(errno) << RESET << std::endl;
        return false;
    }

    speed_t baud = toTermiosBaud(baudRate);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << RED << "[Chassis] tcsetattr: " << std::strerror(errno) << RESET << std::endl;
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

bool isOpenUnsafe() {
    return gFd >= 0;
}

void closeUnsafe() {
    if (gFd >= 0) {
        close(gFd);
        gFd = -1;
    }
}

bool writeAllUnsafe(const std::string& payload) {
    const char* data = payload.c_str();
    size_t total = 0;
    while (total < payload.size()) {
        ssize_t written = write(gFd, data + total, payload.size() - total);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << RED << "[Chassis] write: " << std::strerror(errno) << RESET << std::endl;
            return false;
        }
        total += static_cast<size_t>(written);
    }
    return true;
}

bool readLineUnsafe(std::string& outLine, int timeoutMs) {
    outLine.clear();
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while (std::chrono::steady_clock::now() < deadline) {
        char c = 0;
        ssize_t received = read(gFd, &c, 1);
        if (received > 0) {
            if (c == '\n' || c == '\r') {
                if (!outLine.empty()) {
                    return true;
                }
                continue;
            }
            outLine.push_back(c);
            if (outLine.size() > 255) {
                return true;
            }
            continue;
        }

        if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            std::cerr << RED << "[Chassis] read: " << std::strerror(errno) << RESET << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return !outLine.empty();
}

bool sendCommandLocked(const std::string& command, std::string* response, int timeoutMs) {
    if (!isOpenUnsafe()) {
        return false;
    }

    std::string payload = command;
    if (payload.empty() || payload.back() != '\n') {
        payload.push_back('\n');
    }

    if (!writeAllUnsafe(payload)) {
        return false;
    }

    if (response == nullptr) {
        return true;
    }

    std::string line;
    if (!readLineUnsafe(line, timeoutMs)) {
        return false;
    }

    *response = line;
    return true;
}

bool parsePoseLine(const std::string& line, ChassisPose& outPose) {
    if (line.rfind("POS ", 0) != 0) {
        return false;
    }

    std::string values = line.substr(4);
    std::replace(values.begin(), values.end(), ',', ' ');

    std::istringstream iss(values);
    double xInches = 0.0;
    double yInches = 0.0;
    double headingDeg = 0.0;
    if (!(iss >> xInches >> yInches >> headingDeg)) {
        return false;
    }

    const double localXcm = xInches * CM_PER_INCH * gLocalXSign;
    const double localYcm = yInches * CM_PER_INCH * gLocalYSign;

    const double theta = gBoardRefHeadingDeg * PI_LOCAL / 180.0;
    const double cosT = std::cos(theta);
    const double sinT = std::sin(theta);

    outPose.positionCm.x = gBoardRefPosCm.x + (localXcm * cosT) - (localYcm * sinT);
    outPose.positionCm.y = gBoardRefPosCm.y + (localXcm * sinT) + (localYcm * cosT);
    outPose.headingDeg = normalizeAngleDeg(headingDeg + gBoardRefHeadingDeg);
    outPose.valid = true;
    return true;
}

std::string formatGoCommand(double xCm, double yCm) {
    const double theta = gBoardRefHeadingDeg * PI_LOCAL / 180.0;
    const double cosT = std::cos(theta);
    const double sinT = std::sin(theta);

    const double dx = xCm - gBoardRefPosCm.x;
    const double dy = yCm - gBoardRefPosCm.y;

    const double localXcm = (cosT * dx) + (sinT * dy);
    const double localYcm = (-sinT * dx) + (cosT * dy);

    const double rawXcm = localXcm * gLocalXSign;
    const double rawYcm = localYcm * gLocalYSign;

    const double xInches = rawXcm / CM_PER_INCH;
    const double yInches = rawYcm / CM_PER_INCH;

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "g " << xInches << "," << yInches;
    return oss.str();
}

std::string formatRotateCommand(double deltaDeg) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "o " << deltaDeg;
    return oss.str();
}
} // namespace

bool chassisInit(const std::string& device, int baudRate) {
    std::lock_guard<std::mutex> lock(gChassisMutex);

    gDevice = device;
    gBaudRate = baudRate;

    if (isOpenUnsafe()) {
        return true;
    }

    gFd = open(gDevice.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (gFd < 0) {
        std::cerr << RED << "[Chassis] Impossible d'ouvrir " << gDevice << ": " << std::strerror(errno) << RESET << std::endl;
        return false;
    }

    if (!configureUart(gFd, gBaudRate)) {
        closeUnsafe();
        return false;
    }

    std::cout << GREEN << "[Chassis] UART connecte sur " << gDevice << RESET << std::endl;
    return true;
}

void chassisShutdown() {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    closeUnsafe();
}

bool chassisIsConnected() {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    return isOpenUnsafe();
}

void chassisSetBoardReferencePose(double xCm, double yCm, double headingDeg) {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    gBoardRefPosCm = {xCm, yCm};
    gBoardRefHeadingDeg = normalizeAngleDeg(headingDeg);
}

void chassisSetLocalAxesSigns(double xSign, double ySign) {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    gLocalXSign = (xSign >= 0.0) ? 1.0 : -1.0;
    gLocalYSign = (ySign >= 0.0) ? 1.0 : -1.0;
}

bool chassisResetOdometry() {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    std::string response;
    if (!sendCommandLocked("z", &response, DEFAULT_TIMEOUT_MS)) {
        return false;
    }
    return response.rfind("OK", 0) == 0;
}

bool chassisHoldPosition() {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    std::string response;
    if (!sendCommandLocked("m", &response, DEFAULT_TIMEOUT_MS)) {
        return false;
    }
    return response.rfind("OK", 0) == 0;
}

bool chassisStopMotion() {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    std::string response;
    if (!sendCommandLocked("s", &response, DEFAULT_TIMEOUT_MS)) {
        return false;
    }
    return response.rfind("OK", 0) == 0;
}

bool chassisGoToPositionCm(double xCm, double yCm) {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    std::string response;
    if (!sendCommandLocked(formatGoCommand(xCm, yCm), &response, DEFAULT_TIMEOUT_MS)) {
        return false;
    }
    return response.rfind("OK", 0) == 0;
}

bool chassisRotateRelativeDeg(double deltaDeg) {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    std::string response;
    if (!sendCommandLocked(formatRotateCommand(deltaDeg), &response, DEFAULT_TIMEOUT_MS)) {
        return false;
    }
    return response.rfind("OK", 0) == 0;
}

bool chassisRequestPose(ChassisPose& outPose) {
    std::lock_guard<std::mutex> lock(gChassisMutex);
    std::string response;
    if (!sendCommandLocked("p", &response, DEFAULT_TIMEOUT_MS)) {
        return false;
    }
    return parsePoseLine(response, outPose);
}

void chassis(std::atomic<bool>* stop) {
    Board& board = Board::instance();
    bool lastConnectedState = false;
    bool odometryResetDone = false;

    while (!*stop) {
        if (!chassisIsConnected()) {
            chassisInit(gDevice, gBaudRate);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        if (!odometryResetDone) {
            odometryResetDone = chassisResetOdometry();
            if (odometryResetDone) {
                std::cout << GREEN << "[Chassis] Odometrie reset (0,0,0) OK." << RESET << std::endl;
            }
        }

        ChassisPose pose;
        bool ok = chassisRequestPose(pose);

        if (ok && pose.valid) {
            board.moveMyRobot(pose.positionCm);
            board.setMyRobotOrientation(pose.headingDeg);
            if (!lastConnectedState) {
                std::cout << GREEN << "[Chassis] Position odometrique recue." << RESET << std::endl;
            }
            lastConnectedState = true;
        } else {
            if (lastConnectedState) {
                std::cerr << YELLOW << "[Chassis] Perte lecture position, tentative de reconnexion." << RESET << std::endl;
            }
            lastConnectedState = false;
            chassisShutdown();
            odometryResetDone = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(POSE_POLL_PERIOD_MS));
    }

    chassisShutdown();
}
