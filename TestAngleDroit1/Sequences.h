#ifndef SEQUENCES_H
#define SEQUENCES_H

#include <Arduino.h>
#include "RobotStructures.h"

// Déclarations externes pour utiliser les variables et fonctions du fichier principal
extern RobotState robotState;
extern bool sequenceEnCours;
extern int etapeSequence;
extern const int ETAPES_SEQUENCE_MAX;
extern bool executerProchainMouvement;
extern bool sequenceCercleEnCours;
extern int etapeCercle;
extern const int ETAPES_CERCLE_MAX;
extern const float RAYON_CERCLE;
extern bool executerProchainPointCercle;

// Prototypes de fonctions
void addLog(String message);
void demarer(float deltaX, float deltaY);
DeltaXY convertAbsoluteToRobotCoordinates(DeltaXY targetPoint, RobotState robotState);

// Déclarations des fonctions de séquence
void executerSequenceAutomatique();
void executerSequenceCercle();

#endif // SEQUENCES_H
