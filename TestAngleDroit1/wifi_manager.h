#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include "RobotStructures.h"
#include "logger.h"

void setupWiFi();
void handleWiFiClient();
extern const char* COMPILE_DATE;
extern const char* COMPILE_TIME;

// Extern globales utilisées dans wifi_manager.cpp
extern bool deplacementFait;
extern bool sequenceEnCours;
extern bool sequenceCercleEnCours;
extern int etapeSequence;
extern const int ETAPES_SEQUENCE_MAX;
extern int etapeCercle;
extern const int ETAPES_CERCLE_MAX;
extern bool executerProchainMouvement;
extern bool executerProchainPointCercle;
extern RobotState robotState;
void resetRobot();
void demarer(float, float);
void executerSequenceAutomatique();
void executerSequenceCercle();
String getAllLogs();

#endif // WIFI_MANAGER_H
