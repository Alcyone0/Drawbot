#ifndef ROBOT_STRUCTURES_H
#define ROBOT_STRUCTURES_H

#include <Arduino.h>

/* ===== STRUCTURES ===== */
struct DeltaXY {
  float x;
  float y;
  
  // Constructeur par défaut
  DeltaXY() : x(0), y(0) {}
  
  // Constructeur avec paramètres
  DeltaXY(float _x, float _y) : x(_x), y(_y) {}
};

// Structure pour représenter la position et l'orientation du robot
struct RobotState {
  float x;
  float y;
  float theta;        // Orientation du robot en radians
  
  // Constructor
  RobotState(float _x, float _y, float _theta) 
    : x(_x), y(_y), theta(_theta) {}
    
  // Constructeur par défaut
  RobotState() : x(0), y(0), theta(0) {}
};

struct WheelDistances {
  float left;   // Distance pour la roue gauche en cm
  float right;  // Distance pour la roue droite en cm
  
  // Constructeur par défaut
  WheelDistances() : left(0), right(0) {}
  
  // Constructeur avec paramètres
  WheelDistances(float _left, float _right) : left(_left), right(_right) {};
};

#endif // ROBOT_STRUCTURES_H
