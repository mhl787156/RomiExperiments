#ifndef _Planner_h
#define _Planner_h

#include "mapping.h"
#include "kinematics.h"
#include "utils.h"


class Planner {
    public:
        Planner(Mapper& map):_map(map) {};
        void cancelCurrentMove();
        void calculateNextMove(Kinematics& pose);
        bool calculateDemand(Kinematics& pose);
        bool isPreviousMoveComplete(Kinematics& pose);
        bool isNextMoveValid();
        float nextMoveX();
        float nextMoveY();
        float nextMoveTargetDist();
        float nextMoveTargetAngle();

    private:
        Mapper _map;
        bool _cancel = true;
        bool _validMove = false;
        float _nextMoveX = 0;
        float _nextMoveY = 0;
        float _targetDist = 0;
        float _targetAngle = 0;
};

void Planner::calculateNextMove(Kinematics& pose) {
    // Default Random Motion
    float randHeading = randGaussian(pose.getThetaRadians(), PI/8);
    float randDist = randGaussian(150, 40);
    float distx = randDist * cos(randHeading);
    float disty = randDist * sin(randHeading);
    _nextMoveX = pose.getX() + distx;
    _nextMoveY = pose.getY() + disty;

    Serial1.print("NextMove: ");
    Serial1.print(pose.getX());
    Serial1.print(", ");
    Serial1.print(pose.getY());
    Serial1.print(" -> ");
    Serial1.print(_nextMoveX);
    Serial1.print(", ");
    Serial1.println(_nextMoveY);
    // Demands calculated in Romi.ino each loop cycle
}

bool Planner::calculateDemand(Kinematics& pose) {
    // Given current pose, calculate straight line demand to get to wanted pose
    // Work out distance from origin
    float dx = _nextMoveX - pose.getX(); // targetx - x
    float dy = _nextMoveY - pose.getY(); // targety - y

    // Calculate trajectory parameters
    _targetDist = sqrt( (dx*dx) + (dy*dy) );
    _targetAngle = atan2(dy, dx);
    _targetAngle = wrapAngle(_targetAngle);
}

bool Planner::isPreviousMoveComplete(Kinematics& pose) {
    if(_cancel) {
        // If cancel was called, return true as previous move is complete
        _cancel = false;
        return true;
    } else {
        // Checks if previous move was executed, currently arbitrary small threshold
        // Within 2cm and 15 deg
        calculateDemand(pose);
        return abs(_targetDist) < 20 && abs(_targetAngle - pose.getThetaRadians()) < PI/12;
    }
}

bool Planner::isNextMoveValid() {
    return _validMove;
}

float Planner::nextMoveX() {
    return _nextMoveX;
}

float Planner::nextMoveY() {
    return _nextMoveY;
}

float Planner::nextMoveTargetDist() {
    return _targetDist;
}

float Planner::nextMoveTargetAngle() {
    return _targetAngle;
}

void Planner::cancelCurrentMove() {
    _cancel = true;
}

#endif