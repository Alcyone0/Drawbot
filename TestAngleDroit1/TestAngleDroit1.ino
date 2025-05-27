#include <Wire.h>
#include "wifi_manager.h"
#include <SparkFunLSM6DS3.h>
#include <math.h>
#include "RobotStructures.h"
#include "Sequences.h"
#include "logger.h"
#include "PID.h"
#include "calculs.h"

/* ===== IMU ===== */
LSM6DS3 imu(I2C_MODE, 0x6B);
float biaisGyroZ = 0.0;


/* ===== ÉTATS DU ROBOT ===== */
bool demarre = false;
bool atteint = false;
bool correctionActive = false;
bool deplacementFait = true; // Set to true initially to prevent movement until WiFi instructions

/* ===== PARAMÈTRES ===== */
const float IMPULSIONS_PAR_CM = 34.0;
const int   PWM_MIN = 80;
const int   PWM_MAX = 110;
const float LARGEUR_ROBOT = 8.5/2; // Distance entre l'axe des roue et une roues
const float LONGUEUR_ROBOT = 13.0; // Distance entre l'axe des roues et le stylo

/* ===== VARIABLES INTERNES ===== */
// Variables de contrôle moteur déplacées dans le module PID
extern long seuilImpulsionsRoueGauche;
extern long seuilImpulsionsRoueDroite;
extern float distance_en_cm_roue_gauche;
extern float distance_en_cm_roue_droite;
extern bool directionAvantGauche;
extern bool directionAvantDroite;

/* ===== COMPTEURS ENCODEURS ===== */
volatile long countLeft = 0;   // Pour la roue gauche
volatile long countRight = 0;  // Pour la roue droite

/* ===== VARIABLES POUR SÉQUENCE ===== */
// Déplacées dans Sequences.cpp
extern bool sequenceEnCours;
extern int etapeSequence;
extern const int ETAPES_SEQUENCE_MAX;
extern bool executerProchainMouvement;
extern bool sequenceCercleEnCours;
extern int etapeCercle;
extern const int ETAPES_CERCLE_MAX;
extern const float RAYON_CERCLE;
extern bool executerProchainPointCercle;

/* ===== POSITION DU ROBOT ===== */
RobotState robotState;  // Position et orientation du robot

/* === UTILS === */
void countLeftEncoder()  { countLeft++; }
void countRightEncoder() { countRight++; }
 
void calibrerGyro()
{
  const int N = 500;
  float somme = 0.0;
  for (int i = 0; i < N; i++) {
    somme += imu.readFloatGyroZ();
    delay(5);
  }
  biaisGyroZ = somme / N;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  setupLogger();
  addLog("[setup] Drawbot démarré");
  addLog("[setup] Position initiale: X=0.0, Y=0.0, Theta=0.0°");

  Wire.begin();
  if (imu.begin() != 0) {
    Serial.println("IMU non detectee !");
    while (1);
  }

  calibrerGyro();

  // Initialisation des broches des moteurs et encodeurs dans le module PID
  initMotorPins();

  resetRobot(); 
  setupWiFi();
  arreter();
}

void loop() {
  // S'assurer que le robot ne bouge pas au démarrage ou après un déplacement
  if (deplacementFait && !correctionActive) {
    arreter(); // S'assurer que le robot est bien arrêté
  }
  
  // Ne pas démarrer automatiquement de séquence lors du chargement de la page
  static bool firstRun = true;
  if (firstRun) {
    sequenceEnCours = false;
    sequenceCercleEnCours = false;
    deplacementFait = true;
    firstRun = false;
    addLog("[loop] Premier démarrage, séquences désactivées");
  }
  
  // Gestion WiFi séparée
  handleWiFiClient();
}