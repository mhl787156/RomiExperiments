#ifndef _Planner_h
#define _Plamnner_h

#include "mapping.h"
#include "kinematics.h"
#include "utils.h"


class Planner {
    public:
        void init(Mapper& map);
        void calculateNextMove(Pose& pose);
        bool calculateDemand(Pose& pose);
        bool isPreviousMoveComplete(Pose& pose);
        bool isNextMoveValid();
        float nextMoveX();
        float nextMoveY();
        float nextMoveTargetAngle();

    private:
        Mapper _map;
        bool _validMove = false;
        float _nextMoveX = 0;
        float _nextMoveY = 0;
        float _targetDist = 0;
        float _targetAngle = 0;
};

void Planner::init(Mapper& map){
    _map = map;
}

void Planner::calculateNextMove(Pose& pose) {

    if(!isPreviousMoveComplete(pose)) {
        return;
    }

    // Default Random Motion
    _nextMoveX = randGaussian(pose.getX(), 5);
    _nextMoveY = randGaussian(pose.getY(), 5);
}

bool Planner::calculateDemand(Pose& pose) {
    // Given current pose, calculate straight line demand to get to wanted pose
    // Work out distance from origin
    float dx = _nextMoveX - pose.getX(); // targetx - x
    float dy = _nextMoveY - pose.getY(); // targety - y

    // Calculate trajectory parameters
    _targetDist = sqrt( (dx*dx) + (dy*dy) );
    _targetAngle = atan2(dy, dx) - pose.getThetaRadians();
    _targetAngle = wrapAngle(_targetAngle);
}

bool Planner::isPreviousMoveComplete(Pose& pose) {
    // Checks if previous move was executed, currently arbitrary small threshold
    calculateDemand(pose);
    return abs(_targetDist) < 1 && abs(_targetAngle) < PI/12;
}

bool Planner::isNextMoveValid() {
    return _validMove
}

float Planner::nextMoveX() {
    return _nextMoveX;
}

float Planner::nextMoveY() {
    return _nextMoveY;
}

float nextMoveTargetAngle() {
    return _targetAngle;
}

#endif