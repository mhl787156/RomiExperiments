#ifndef _Planner_h
#define _Planner_h

#include "beliefmapping.h"
#include "kinematics.h"
#include "utils.h"


class Planner {
    public:
        Planner(BeliefMapper& map):_map(map) {};
        void cancelCurrentMove();
        void calculateNextMove(Kinematics& pose);
        bool calculateDemand(Kinematics& pose);
        int isPreviousMoveComplete(Kinematics& pose);
        bool isNextMoveValid();
        float nextMoveX();
        float nextMoveY();
        float nextMoveTargetDist();
        float nextMoveTargetAngle();

    private:
        BeliefMapper _map;
        bool _cancel = true;
        bool _validMove = false;
        float _nextMoveX = 0;
        float _nextMoveY = 0;
        float _targetDist = 0;
        float _targetAngle = 0;
};

void Planner::calculateNextMove(Kinematics& pose) {
    // Maximum closest gradient based method

    float best_x = pose.getX();
    float best_y = pose.getY();
    int max_entropy = 0;

    float rand_threshold = 100;

    bool found = false;
    while(!found) {
        int i = random(0, 25);
        int j = random(0, 25);
        int eeprom_address = (i*MAP_RESOLUTION)+j;
        
        if (eeprom_address > 1023)
        {
            Serial1.println(F("Error: EEPROM Address greater than 1023"));
        }
        else
        {
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            short conf = value & CONFIDENCE_MASK;
            float conf_p = ((float) conf) / 64;
            
            float entropy_p = -(conf_p * log(conf_p)) -((1 - conf_p) * log(1-conf_p));

            // Find cell of maximum entropy
            if (entropy_p >= max_entropy) {
                max_entropy = entropy_p;
                float x_loc = _map.indexToPose(j, MAP_X, MAP_RESOLUTION);
                float y_loc = _map.indexToPose(i, MAP_Y, MAP_RESOLUTION);    
                best_x = x_loc;
                best_y = y_loc;

                if (random(0, 1000) < rand_threshold) {                        
                    found = true;
                    break;
                }
            }
        }
    }
    // Set next Move
    _nextMoveX = best_x;
    _nextMoveY = best_y;

    Serial1.print("NextMove: ");
    Serial1.print(pose.getX());
    Serial1.print(", ");
    Serial1.print(pose.getY());
    Serial1.print(" -> ");
    Serial1.print(_nextMoveX);
    Serial1.print(", ");
    Serial1.println(_nextMoveY);
    Serial1.print("  ");

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

int Planner::isPreviousMoveComplete(Kinematics& pose) {
    if(_cancel) {
        // If cancel was called, return true as previous move is complete
        _cancel = false;
        return 2;
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