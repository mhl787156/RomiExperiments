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

    short best_i = 0;
    short best_j = 0;

    float best_x = pose.getX();
    float best_y = pose.getY();
    short conf_threshold = 0;
    short dist_min_threshold = 100; // 10cm
    float best_dist = 10000;

    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for (int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            
            if (eeprom_address > 1023)
            {
                Serial1.println(F("Error: EEPROM Address greater than 1023"));
            }
            else
            {
                int eeprom_address = (i*MAP_RESOLUTION)+j;
                byte value;
                value = EEPROM.read(eeprom_address);//, value);
                short conf = value & CONFIDENCE_MASK;
                short conf_diff = abs(conf - DEFAULT_PROB);
                
                // Check Probability value, want within threshold of default
                if(conf_diff <= conf_threshold) {
                    float x_loc = _map.indexToPose(i, MAP_X, MAP_RESOLUTION);
                    float y_loc = _map.indexToPose(j, MAP_Y, MAP_RESOLUTION);
                    float dist = pose.getDistanceFromLoc(x_loc, y_loc);

                    if (dist < best_dist && dist > dist_min_threshold) {
                        // Serial1.print(conf_diff);
                        // Serial1.print(" ");
                        // Serial1.println(dist);
                        best_dist = dist;
                        best_x = x_loc;
                        best_y = y_loc;
                        best_i = i;
                        best_j = j;
                    }
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
    Serial1.print(best_i);
    Serial1.print(", ");
    Serial1.println(best_j);

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