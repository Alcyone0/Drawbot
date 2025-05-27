#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "RobotStructures.h"
#include "calculs.h"

// Variables externes nécessaires pour le contrôle des moteurs
extern volatile long countLeft;
extern volatile long countRight;

// Variables définies dans PID.cpp
extern long seuilImpulsionsRoueGauche;
extern long seuilImpulsionsRoueDroite;
extern float distance_en_cm_roue_gauche;
extern float distance_en_cm_roue_droite;
extern bool directionAvantGauche;
extern bool directionAvantDroite;

// Broches des encodeurs
extern const int encoderLeftA;
extern const int encoderRightA;

// Constantes nécessaires au module PID
extern const int PWM_MIN;
extern const int PWM_MAX;
extern const float IMPULSIONS_PAR_CM;

// Variables de contrôle d'état
extern bool correctionActive;
extern bool deplacementFait;

// Variables de séquence
extern bool sequenceEnCours;
extern bool sequenceCercleEnCours;
extern int etapeSequence;
extern int etapeCercle;
extern bool executerProchainMouvement;
extern bool executerProchainPointCercle;

// Position du robot
extern RobotState robotState;

// Fonctions de contrôle des moteurs
void initMotorPins();
void resetRobot();
void demarer(float deltaX, float deltaY);
bool avancerCorrige();
void arreter();

#endif // PID_H
