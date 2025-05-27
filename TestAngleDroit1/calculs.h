#ifndef CALCULS_H
#define CALCULS_H

#include <Arduino.h>
#include "RobotStructures.h"

// Calcule les distances des deux roues en fonction du déplacement demandé
WheelDistances calculerDistancesRoues(DeltaXY robotRelativePoint);

// Calcule la nouvelle position du robot après un déplacement
RobotState calculerNouvellePosition(WheelDistances distances);

// Convertit les coordonnées absolues en coordonnées relatives au robot
DeltaXY convertAbsoluteToRobotCoordinates(DeltaXY targetPoint, RobotState robotState);

// Variables externes utilisées par les fonctions de calcul
extern RobotState robotState;
extern const float LARGEUR_ROBOT;
extern const float LONGUEUR_ROBOT;

#endif // CALCULS_H
