#ifndef PID_H
#define PID_H

#include <Arduino.h>

// Variables externes nécessaires pour le contrôle des moteurs
extern volatile long countLeft;
extern volatile long countRight;
extern long seuilImpulsionsRoueGauche;
extern long seuilImpulsionsRoueDroite;
extern bool directionAvantGauche;
extern bool directionAvantDroite;
extern const int PWM_MIN;
extern const int PWM_MAX;

// Fonctions de contrôle des moteurs
void arreter();
bool avancerCorrige();

#endif // PID_H
