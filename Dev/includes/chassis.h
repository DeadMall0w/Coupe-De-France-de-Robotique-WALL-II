#pragma once

#include <atomic>
#include <string>

#include "Board.hpp"

struct ChassisPose {
    Position positionCm;
    double headingDeg = 0.0;
    bool valid = false;
};

bool chassisInit(const std::string& device = "/dev/ttyAMA3", int baudRate = 115200);
void chassisShutdown();
bool chassisIsConnected();

void chassisSetBoardReferencePose(double xCm, double yCm, double headingDeg);
void chassisSetLocalAxesSigns(double xSign, double ySign);
bool chassisResetOdometry();

bool chassisHoldPosition();
bool chassisStopMotion();
bool chassisGoToPositionCm(double xCm, double yCm);
bool chassisRotateRelativeDeg(double deltaDeg);
bool chassisRequestPose(ChassisPose& outPose);

void chassis(std::atomic<bool>* stop);
