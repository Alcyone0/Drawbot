#include <Wire.h>
#include <WiFi.h>
#include <SparkFunLSM6DS3.h>
#include <math.h>
#include "RobotStructures.h"
#include "Sequences.h"
#include "PID.h"

/* ===== IMU ===== */
LSM6DS3 imu(I2C_MODE, 0x6B);
float biaisGyroZ = 0.0;

/* ===== WIFI ===== */
const char* ssid = "Drawbot_WIFI";
const char* password = "12345678";
WiFiServer server(80);
const char* COMPILE_DATE = __DATE__;
const char* COMPILE_TIME = __TIME__;

/* ===== MOTEURS ===== */
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16
#define EN_D   23
#define EN_G    4

/* ===== ENCODEURS ===== */
const int encoderLeftA  = 27;
const int encoderRightA = 33;
volatile long countLeft  = 0;
volatile long countRight = 0;
bool demarre = false;
bool atteint = false;
bool correctionActive = false;
bool deplacementFait = true; // Set to true initially to prevent movement until WiFi instructions

/* ===== PARAMÈTRES ===== */
const float IMPULSIONS_PAR_CM = 54.4;  // Valeur recalibrée (ancienne valeur = 34.0)
const int   PWM_MIN = 70;       // Valeur d'origine
const int   PWM_MAX = 110;      // Valeur d'origine
const float DIST_STYLO_CM = 13.0;
const float LARGEUR_ROBOT = 8.5/2;
const float LONGUEUR_ROBOT = 13.0; // Distance entre l'axe des roues et le stylo
const int   TEMPS_RAMPE_MS = 300; // Rampe d'accélération courte

/* ===== VARIABLES INTERNES ===== */
long seuilImpulsionsRoueGauche = 0;
long seuilImpulsionsRoueDroite = 0;
float distance_en_cm_roue_gauche = 0;
float distance_en_cm_roue_droite = 0;
bool directionAvantGauche = true; // true = avant, false = arrière
bool directionAvantDroite = true; // true = avant, false = arrière

/* ===== VARIABLES PID ===== */
// Paramètres de PID améliorés
double Kp_G = 8.0;    // Coefficient proportionnel roue gauche
double Ki_G = 0.1;    // Coefficient intégrateur roue gauche
double Kd_G = 0.5;    // Coefficient dérivé roue gauche

double Kp_D = 8.0;    // Coefficient proportionnel roue droite
double Ki_D = 0.1;    // Coefficient intégrateur roue droite
double Kd_D = 0.5;    // Coefficient dérivé roue droite

// Variables pour le nouveau PID
double inputGauche = 0.0;   // Valeur actuelle - impulsions roue gauche
double outputGauche = 0.0;  // Sortie calculée - puissance moteur gauche
double setpointGauche = 0.0; // Consigne - impulsions cibles roue gauche

double inputDroite = 0.0;   // Valeur actuelle - impulsions roue droite
double outputDroite = 0.0;  // Sortie calculée - puissance moteur droite
double setpointDroite = 0.0; // Consigne - impulsions cibles roue droite

// Création des objets PID pour chaque roue
PID pidGauche(&inputGauche, &outputGauche, &setpointGauche, Kp_G, Ki_G, Kd_G, 1, 0); // 1=P_ON_E, 0=DIRECT
PID pidDroite(&inputDroite, &outputDroite, &setpointDroite, Kp_D, Ki_D, Kd_D, 1, 0); // 1=P_ON_E, 0=DIRECT

unsigned long tempsPrec = 0;  // Pour calculer le delta temps
unsigned long tempsDebutMouvement = 0; // Pour la rampe d'accélération

/* ===== VARIABLES POUR SÉQUENCE AUTOMATIQUE ===== */
bool sequenceEnCours = false;
int etapeSequence = 0;
const int ETAPES_SEQUENCE_MAX = 30; // 10 pas à droite + 10 pas en haut + 10 pas à droite
bool executerProchainMouvement = true;

//----------------------------------------------------------------------------------------------------------------VARIABLES------------------------------------------------------------------

/* ===== PROTOTYPES DE FONCTIONS ===== */
void demarer(float deltaX, float deltaY); // Déclaration anticipée
void executerSequenceAutomatique();
void executerSequenceEscalier(); // Déclaration pour la séquence automatique

/* ===== POSITION DU ROBOT ===== */
RobotState robotState;  // Position et orientation du robot
RobotState positionTheorique;  // Position théorique calculée
RobotState positionReelle;  // Position réelle calculée

/* ===== LOGS ===== */
const int MAX_LOGS = 100; // Augmentation significative du nombre de logs conservés
String logs[MAX_LOGS];
int logIndex = 0;

/* === UTILS === */
void countLeftEncoder()  { countLeft++; }
void countRightEncoder() { countRight++; }

void arreter()
{
  analogWrite(EN_D, 0);  analogWrite(EN_G, 0);
  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, LOW);
}
 
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

//----------------------------------------------------------------------------------------------------------------CALCULER DISTANCES ROUES----------------------------------------------------------

// Calcule les distances des deux roues en fonction du déplacement demandé et met à jour la position du robot
WheelDistances calculerDistancesRoues(float deltaX, float deltaY) {
  WheelDistances distances;
  distances.left = deltaX - (LARGEUR_ROBOT / LONGUEUR_ROBOT) * deltaY;
  distances.right = deltaX + (LARGEUR_ROBOT / LONGUEUR_ROBOT) * deltaY;  

  addLog("[calculerDistancesRoues] Distances roues: G=" + String(distances.left, 1) + " cm, D=" + String(distances.right, 1) + " cm");

  return distances;
}

//----------------------------------------------------------------------------------------------------------------CALCULER NOUVELLE POSITION THEORIQUE----------------------------------------------------------

RobotState calculerNouvellePositionTheorique(float distanceLeft, float distanceRight) {
  WheelDistances distances;
  distances.left = distanceLeft;
  distances.right = distanceRight;

  // Préparer la structure RobotState à renvoyer
  RobotState newState(robotState.x, robotState.y, robotState.theta);
  
  // Calcul de la distance moyenne parcourue par les deux roues
  float deplacementAxeRobot = (distances.left + distances.right) / 2.0;

  // Calcul de la différence entre les distances des roues
  float deltaRoues = distances.left - distances.right; // Attention au signe
  
  // Calcul de l'angle relatif avec atan2
  float angleRelatif = atan2(deltaRoues, LARGEUR_ROBOT)/2.; // Attention à l'ordre

  // Déplacement latéral
  float deplacementLatéral = LONGUEUR_ROBOT * sin(angleRelatif);
  
  // Calcul de l'angle absolu
  float angle = robotState.theta - angleRelatif;
  
  // Mise à jour de la position du robot en fonction de son orientation
  newState.x = robotState.x + deplacementAxeRobot * cos(robotState.theta) + deplacementLatéral * sin(robotState.theta); // Attention à cos sin et au + -
  newState.y = robotState.y + deplacementAxeRobot * sin(robotState.theta) - deplacementLatéral * cos(robotState.theta); // Attention à cos sin et au + -
  
  // Mise à jour de l'angle du robot
  newState.theta = angle;
  
  // Normaliser l'angle entre -PI et PI
  while (newState.theta > PI) newState.theta -= 2*PI;
  while (newState.theta < -PI) newState.theta += 2*PI;
    
  return newState;
}

//----------------------------------------------------------------------------------------------------------------CALCULER NOUVELLE POSITION REELLE----------------------------------------------------------

RobotState calculerNouvellePositionReelle(int tickLeft, int tickRight) {
  WheelDistances distances;
  distances.left = tickLeft / IMPULSIONS_PAR_CM;
  distances.right = tickRight / IMPULSIONS_PAR_CM;

  // Préparer la structure RobotState à renvoyer
  RobotState newState(robotState.x, robotState.y, robotState.theta);
  
  // Calcul de la distance moyenne parcourue par les deux roues
  float deplacementAxeRobot = (distances.left + distances.right) / 2.0;

  // Calcul de la différence entre les distances des roues
  float deltaRoues = distances.left - distances.right; // Attention au signe
  
  // Calcul de l'angle relatif avec atan2
  float angleRelatif = atan2(deltaRoues, LARGEUR_ROBOT)/2.; // Attention à l'ordre

  // Déplacement latéral
  float deplacementLatéral = LONGUEUR_ROBOT * sin(angleRelatif);
  
  // Calcul de l'angle absolu
  float angle = robotState.theta - angleRelatif;
  
  // Mise à jour de la position du robot en fonction de son orientation
  newState.x = robotState.x + deplacementAxeRobot * cos(robotState.theta) + deplacementLatéral * sin(robotState.theta); // Attention à cos sin et au + -
  newState.y = robotState.y + deplacementAxeRobot * sin(robotState.theta) - deplacementLatéral * cos(robotState.theta); // Attention à cos sin et au + -
  
  // Mise à jour de l'angle du robot
  newState.theta = angle;
  
  // Normaliser l'angle entre -PI et PI
  while (newState.theta > PI) newState.theta -= 2*PI;
  while (newState.theta < -PI) newState.theta += 2*PI;
    
  return newState;
}

//----------------------------------------------------------------------------------------------------------------CONVERT ABSOLU RELATIF----------------------------------------------------------

// Fonction pour convertir les coordonnées absolues en coordonnées relatives au robot
DeltaXY convertAbsoluteToRobotCoordinates(DeltaXY targetPoint, RobotState robotState) {
  float deltaX = targetPoint.x - robotState.x;
  float deltaY = targetPoint.y - robotState.y;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  addLog("[convertAbsoluteToRobotCoordinates] Delta X = " + String(deltaX, 2));
  addLog("[convertAbsoluteToRobotCoordinates] Delta Y = " + String(deltaY, 2));
  addLog("[convertAbsoluteToRobotCoordinates] Distance = " + String(distance, 2));

  float angle = atan2(deltaY, deltaX); // Angle par rapport à l'axe X
  addLog("[convertAbsoluteToRobotCoordinates] AngleAbsolu = " + String(angle*180/PI, 2));
  float angleRelatif =  angle - robotState.theta;
  addLog("[convertAbsoluteToRobotCoordinates] Angle relatif = " + String(angleRelatif*180/PI, 2));
  addLog("[convertAbsoluteToRobotCoordinates] Angle Robot = " + String(robotState.theta*180/PI, 2));
  
  // 3. Reconvertir en coordonnées cartésiennes relatives au robot
  float deltaRobotX = distance * cos(angleRelatif);
  float deltaRobotY = distance * sin(angleRelatif);
  addLog("[convertAbsoluteToRobotCoordinates] Delta robot X = " + String(deltaRobotX, 2));
  addLog("[convertAbsoluteToRobotCoordinates] Delta robot Y = " + String(deltaRobotY, 2));
  
  // Créer un point pour les coordonnées relatives au robot
  DeltaXY robotRelativePoint(deltaRobotX, deltaRobotY);
  
  // Retourner les coordonnées relatives au robot
  return robotRelativePoint;
}

//----------------------------------------------------------------------------------------------------------------ADD LOGS----------------------------------------------------------

void addLog(String message) {
  // Ajouter l'horodatage
  unsigned long ms = millis();
  String logMessage = String(ms) + "ms: " + message;
  
  // Enregistrer dans le tableau circulaire
  logs[logIndex] = logMessage;
  logIndex = (logIndex + 1) % MAX_LOGS;
  
  // Afficher aussi sur le moniteur série
  Serial.println(logMessage);
}

String getAllLogs() {
  String allLogs = "";
  int count = 0;
  
  // Trouver le plus ancien log (non vide)
  int startIdx = logIndex;
  for (int i = 0; i < MAX_LOGS; i++) {
    int idx = (logIndex + i) % MAX_LOGS;
    if (logs[idx].length() == 0) {
      startIdx = (idx + 1) % MAX_LOGS;
      break;
    }
  }
  
  // Parcourir les logs à partir du plus ancien vers le plus récent
  for (int i = 0; i < MAX_LOGS; i++) {
    int idx = (startIdx + i) % MAX_LOGS;
    if (logs[idx].length() > 0 && idx != logIndex) {
      allLogs += logs[idx] + "<br>";
      count++;
    }
  }
  
  return allLogs.length() > 0 ? allLogs : "Aucun log disponible";
}

//----------------------------------------------------------------------------------------------------------------DEMARRER MOUVEMENT----------------------------------------------------------

// Fonction pour démarrer le mouvement du robot
void demarer(float deltaX, float deltaY) {
  addLog("[demarer] DX=" + String(deltaX) + ", DY=" + String(deltaY));
  // Vérifier si les valeurs sont zéro
  if (abs(deltaX) < 0.05 && abs(deltaY) < 0.05) {
    addLog("[demarer] Valeurs trop petites, pas de mouvement");
    return;
  }
  
  // Calculer les distances pour chaque roue avec la nouvelle fonction
  WheelDistances distances = calculerDistancesRoues(deltaX, deltaY);
  distance_en_cm_roue_gauche = distances.left;
  distance_en_cm_roue_droite = distances.right;
  
  // Vérifier que les distances ne sont pas aberrantes
  if (abs(distance_en_cm_roue_gauche) > 50 || abs(distance_en_cm_roue_droite) > 50) {
    addLog("[demarer] ATTENTION: Distances calculées anormalement grandes, limitation à 20cm");
    if (distance_en_cm_roue_gauche > 50) distance_en_cm_roue_gauche = 20;
    if (distance_en_cm_roue_gauche < -50) distance_en_cm_roue_gauche = -20;
    if (distance_en_cm_roue_droite > 50) distance_en_cm_roue_droite = 20;
    if (distance_en_cm_roue_droite < -50) distance_en_cm_roue_droite = -20;
  }
  
  // Déterminer la direction de chaque roue
  directionAvantGauche = (distance_en_cm_roue_gauche >= 0);
  directionAvantDroite = (distance_en_cm_roue_droite >= 0);
  
  // Calculer les seuils d'impulsions (toujours en valeur absolue)
  seuilImpulsionsRoueGauche = abs(distance_en_cm_roue_gauche * IMPULSIONS_PAR_CM);
  seuilImpulsionsRoueDroite = abs(distance_en_cm_roue_droite * IMPULSIONS_PAR_CM);
  
  // Enregistrer les valeurs dans les logs avec plus de détails
  String logMsg = "[demarer] Commande reçue: DX=" + String(deltaX, 2) + 
                  ", DY=" + String(deltaY, 2) + 
                  ", Distance Gauche=" + String(distance_en_cm_roue_gauche, 2) + " cm" +
                  ", Distance Droite=" + String(distance_en_cm_roue_droite, 2) + " cm" + 
                  ", Seuil G=" + String(seuilImpulsionsRoueGauche) + 
                  ", Seuil D=" + String(seuilImpulsionsRoueDroite);
  addLog("[demarer] " + logMsg);
  

  // Calculer et mettre à jour la position du robot
  positionReelle = calculerNouvellePositionReelle(countLeft, countRight);
  positionTheorique = calculerNouvellePositionTheorique(distance_en_cm_roue_gauche, distance_en_cm_roue_droite);
  robotState = positionTheorique; // On utilise la position théorique comme position actuelle

  // Réinitialiser les compteurs
  countLeft = 0;
  countRight = 0;
  
  // Réinitialiser les variables pour le nouveau PID
  inputGauche = countLeft;
  inputDroite = countRight;
  setpointGauche = seuilImpulsionsRoueGauche;
  setpointDroite = seuilImpulsionsRoueDroite;
  
  // Réinitialiser les PID en réactivant le mode automatique
  pidGauche.SetMode(0);  // MANUAL
  pidGauche.SetMode(1);  // AUTOMATIC
  pidDroite.SetMode(0);  // MANUAL
  pidDroite.SetMode(1);  // AUTOMATIC
  
  // Début du mouvement avec le nouveau PID
  tempsDebutMouvement = millis();
  tempsPrec = millis();
  addLog("[demarer] Début du mouvement avec nouveau PID: Gauche=" + String(setpointGauche) + ", Droite=" + String(setpointDroite));
  
  // Ne pas désactiver complètement le WiFi, juste marquer qu'on est en mouvement
  // WiFi.disconnect();
  // server.end();
  addLog("[demarer] Début du déplacement - WiFi peut être moins réactif");
  
  // Désactiver temporairement le WiFi pendant le déplacement
  WiFi.disconnect();
  server.end();
  addLog("[demarer] WiFi temporairement désactivé pour le déplacement");
  
  // Activer le mouvement
  correctionActive = true;
  deplacementFait = false;
  addLog("[demarer] Début du mouvement");
} 

//----------------------------------------------------------------------------------------------------------------AVANCER CORRIGE----------------------------------------------------------

bool avancerCorrige() {
  // Configurer la direction des moteurs en fonction des valeurs calculées
  if (directionAvantDroite) {
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
  } else {
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
  }
  
  if (directionAvantGauche) {
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
  } else {
    digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, HIGH);
  }
  
  // Mise à jour des entrées du PID avec les valeurs actuelles des encodeurs
  inputGauche = countLeft;
  inputDroite = countRight;
  
  // Calculer les nouvelles sorties PID
  pidGauche.Compute();
  pidDroite.Compute();
  
  // Utiliser le facteur de rampe pour un démarrage en douceur
  float facteurRampe = 1.0;
  unsigned long tempsActuel = millis();
  unsigned long tempsEcoule = tempsActuel - tempsDebutMouvement;
  
  if (tempsEcoule < TEMPS_RAMPE_MS) {
    facteurRampe = (float)tempsEcoule / TEMPS_RAMPE_MS;
    // Rampe plus douce au démarrage (fonction quadratique)
    facteurRampe = facteurRampe * facteurRampe;
  }
  
  // Calculer les erreurs actuelles pour les logs
  double erreurGauche = setpointGauche - inputGauche;
  double erreurDroite = setpointDroite - inputDroite;
  
  // Appliquer les corrections PID calculées avec la rampe d'accélération
  int pwmG = constrain(PWM_MIN + outputGauche * facteurRampe, PWM_MIN, PWM_MAX);
  int pwmD = constrain(PWM_MIN + outputDroite * facteurRampe, PWM_MIN, PWM_MAX);
  
  // Logs détaillés et mise à jour de la position réelle périodiquement
  static unsigned long dernierLog = 0;
  if (millis() - dernierLog > 1000) { // Une fois par seconde
    // Mettre à jour la position réelle pendant le mouvement
    positionReelle = calculerNouvellePositionReelle(countLeft, countRight);
    
    // Log des informations PID détaillées
    addLog("[PID G] Cible: " + String(setpointGauche) + ", Actuel: " + String(inputGauche) + 
           ", Erreur: " + String(erreurGauche) + ", Puissance: " + String(pwmG));
    addLog("[PID D] Cible: " + String(setpointDroite) + ", Actuel: " + String(inputDroite) + 
           ", Erreur: " + String(erreurDroite) + ", Puissance: " + String(pwmD));
    
    // Log de la position réelle
    addLog("[position] Réelle: X=" + String(positionReelle.x, 2) + ", Y=" + String(positionReelle.y, 2) + 
           ", Écart: X=" + String(positionReelle.x - robotState.x, 2) + ", Y=" + String(positionReelle.y - robotState.y, 2));
    
    dernierLog = millis();
  }
  
  /* Logs PID commentés
  static unsigned long dernierLogRampe = 0;
  if (tempsEcoule < TEMPS_RAMPE_MS && millis() - dernierLogRampe > 100) {
    addLog("[rampe] Facteur: " + String(facteurRampe, 2) + 
           " PWM G: " + String(pwmG) + 
           " PWM D: " + String(pwmD));
    dernierLogRampe = millis();
  }
  
  // Loguer les valeurs du PID occasionnellement pour le débogage
  if (millis() - dernierLog > 500) { // Log toutes les 500ms
    addLog("[PID] G: err=" + String(erreurGauche, 1) + 
          " P=" + String(Kp_G * erreurGauche, 1) + 
          " I=" + String(Ki_G * erreurSommeGauche, 1) + 
          " D=" + String(Kd_G * erreurDeriveGauche, 1) + 
          " PWM=" + String(pwmG));
    addLog("[PID] D: err=" + String(erreurDroite, 1) + 
          " P=" + String(Kp_D * erreurDroite, 1) + 
          " I=" + String(Ki_D * erreurSommeDroite, 1) + 
          " D=" + String(Kd_D * erreurDeriveDroite, 1) + 
          " PWM=" + String(pwmD));
    dernierLog = millis();
  }
  */

  analogWrite(EN_D, pwmD);
  analogWrite(EN_G, pwmG);

  bool fini = (countLeft >= seuilImpulsionsRoueGauche) && (countRight >= seuilImpulsionsRoueDroite);
  if (fini) {
    addLog("[avancerCorrige] Déplacement terminé");
    
    // Mettre à jour les positions calculées à la fin du mouvement
    positionReelle = calculerNouvellePositionReelle(countLeft, countRight);
    positionTheorique = calculerNouvellePositionTheorique(distance_en_cm_roue_gauche, distance_en_cm_roue_droite);
    robotState = positionTheorique; // Utiliser la position théorique comme position actuelle
    
    addLog("[avancerCorrige] Position mise à jour - X=" + String(robotState.x, 2) + ", Y=" + String(robotState.y, 2));
    
    // Réactiver le WiFi à la fin du mouvement
    WiFi.softAP(ssid, password);
    server.begin();
    String ipAddress = WiFi.softAPIP().toString();
    addLog("[avancerCorrige] WiFi réactivé - IP: " + ipAddress);
    addLog("[avancerCorrige] Déplacement terminé - WiFi pleinement disponible");
  }
  return fini;
}

//----------------------------------------------------------------------------------------------------------------RESET ROBOT----------------------------------------------------------

// Fonction pour réinitialiser la position et l'orientation du robot
void resetRobot() {
  // Arrêter le robot et toute séquence en cours
  arreter();
  sequenceEnCours = false;
  deplacementFait = true;
  correctionActive = false;
  etapeSequence = 0;
  executerProchainMouvement = true;
  
  // Réinitialiser les seuils et les distances
  seuilImpulsionsRoueGauche = 0;
  seuilImpulsionsRoueDroite = 0;
  distance_en_cm_roue_gauche = 0;
  distance_en_cm_roue_droite = 0;
  
  // Réinitialiser les compteurs d'encodeurs
  countLeft = 0;
  countRight = 0;
  
  // Réinitialiser la position et l'orientation
  robotState = RobotState(); // Réinitialiser à 0,0,0 avec le constructeur par défaut
  positionTheorique = RobotState(); // Réinitialiser la position théorique à 0,0,0
  positionReelle = RobotState(); // Réinitialiser la position réelle à 0,0,0
  
  addLog("[reset] Positions (actuelle, théorique, réelle) et orientation réinitialisées à zéro");
  addLog("[reset] X=0.0, Y=0.0, Theta=0.0°");
}

//----------------------------------------------------------------------------------------------------------------SETUP----------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(1000);
  addLog("[setup] Drawbot démarré");
  
  // Initialiser toutes les positions à 0,0,0
  robotState = RobotState();
  positionTheorique = RobotState();
  positionReelle = RobotState();
  
  addLog("[setup] Position initiale: X=0.0, Y=0.0, Theta=0.0°");

  Wire.begin();
  if (imu.begin() != 0) {
    Serial.println("IMU non detectee !");
    while (1);
  }

  calibrerGyro();

  pinMode(IN_1_D, OUTPUT); pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT); pinMode(IN_2_G, OUTPUT);
  pinMode(EN_D, OUTPUT);   pinMode(EN_G, OUTPUT);
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderRightA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLeftA),  countLeftEncoder,  RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), countRightEncoder, RISING);

  // Initialiser les contrôleurs PID
  pidGauche.SetOutputLimits(0, PWM_MAX - PWM_MIN);
  pidDroite.SetOutputLimits(0, PWM_MAX - PWM_MIN);
  pidGauche.SetSampleTime(50);  // 50ms (20Hz)
  pidDroite.SetSampleTime(50);  // 50ms (20Hz)
  pidGauche.SetMode(1);  // AUTOMATIC
  pidDroite.SetMode(1);  // AUTOMATIC
  
  // S'assurer que les séquences sont désactivées au démarrage
  sequenceEnCours = false;
  deplacementFait = true; // Pour éviter que le robot ne bouge au démarrage
  
  WiFi.softAP(ssid, password);
  server.begin();
  String ipAddress = WiFi.softAPIP().toString();
  addLog("[setup] WiFi AP démarré - IP: " + ipAddress);
  
  // S'assurer que le robot est arrêté au démarrage
  arreter();
}

//----------------------------------------------------------------------------------------------------------------LOOP----------------------------------------------------------

void loop() {
  // S'assurer que le robot ne bouge pas au démarrage ou après un déplacement
  if (deplacementFait && !correctionActive) {
    arreter(); // S'assurer que le robot est bien arrêté
  }
  
  // Ne pas démarrer automatiquement de séquence lors du chargement de la page
  static bool firstRun = true;
  if (firstRun) {
    sequenceEnCours = false;

    deplacementFait = true;
    firstRun = false;
    addLog("[loop] Premier démarrage, séquences désactivées");
  }
  
  // Ne traiter les requêtes WiFi que si le robot n'est pas en train de se déplacer
  //if (deplacementFait) {
    WiFiClient client = server.available();
    if (client) {
      while (!client.available()) delay(1);
      String request = client.readStringUntil('\r');
      client.flush();

      int posX = request.indexOf("dx=");
      int posY = request.indexOf("dy=");
      int posType = request.indexOf("type=");
      int posIncrement = request.indexOf("increment=");
      
      // Vérifier si c'est une vraie requête de formulaire avec des paramètres
      bool isFormSubmit = false;
      
      // Vérifier le paramètre de soumission explicite
      int posSubmit = request.indexOf("submit=1");
      
      // Vérifier si la requête contient le paramètre de soumission
      if (posSubmit != -1) {
        isFormSubmit = true;
        if (request.indexOf("GET /?reset=") != -1) {
          addLog("[wifi] Demande de réinitialisation reçue");
        } else if (request.indexOf("GET /?dx=") != -1) {
          addLog("[wifi] Demande de déplacement reçue");
        }
      } else if (request.indexOf("GET /?dx=") != -1 || request.indexOf("GET /?reset=") != -1) {
        // C'est un rechargement de page avec les paramètres dans l'URL
        addLog("[wifi] Recharge de page détectée, mouvement ignoré");
      }
      
      // Traiter les boutons de déplacement
      if (posX != -1 && posY != -1 && isFormSubmit) {
        // Déterminer le type de coordonnées (absolu, robot, ou direct)
        String coordType = "absolu"; // Par défaut, on considère les coordonnées comme absolues
        
        if (posType != -1) {
          String typeValue = request.substring(posType + 5, request.indexOf('&', posType + 5));
          if (typeValue == "robot") {
            coordType = "robot";
            addLog("[wifi] Type de coordonnées: relatives au robot");
          } else if (typeValue == "direct") {
            coordType = "direct";
            addLog("[wifi] Type de coordonnées: directes (sans conversion)");
          } else {
            addLog("[wifi] Type de coordonnées: absolues");
          }
        } else {
          addLog("[wifi] Type non spécifié, utilisation des coordonnées absolues par défaut");
        }
        
        float dx = request.substring(posX + 3, request.indexOf('&', posX)).toFloat();
        float dy = request.substring(posY + 3).toFloat();
        addLog("[wifi] Valeurs reçues: dx=" + String(dx) + ", dy=" + String(dy));
        
        if (coordType == "absolu") {
          // Convertir les coordonnées absolues en coordonnées relatives au robot
          addLog("[wifi] Conversion de coordonnées absolues vers robot");
          DeltaXY absolutePoint(robotState.x + dx, robotState.y + dy);
          addLog("[wifi] Point robot: " + String(robotState.x, 2) + ", " + String(robotState.y, 2));
          addLog("[wifi] Point absolu: " + String(absolutePoint.x, 2) + ", " + String(absolutePoint.y, 2));

          DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(absolutePoint, robotState);
          // Démarrer le mouvement avec les coordonnées relatives
          demarer(robotCoord.x, robotCoord.y);
        } else {
          // Utiliser directement les coordonnées relatives au robot (type=robot ou type=direct)
          addLog("[wifi] Utilisation directe des coordonnées: " + coordType + " (" + String(dx, 2) + ", " + String(dy, 2) + ")");
          demarer(dx, dy);
        }
      } else {
        // Vérifier si c'est une demande pour lancer la séquence escalier
        int posSequenceEscalier = request.indexOf("sequence_escalier=1");
        
        if (posSequenceEscalier != -1) {
          addLog("[wifi] Paramètre sequence_escalier=1 détecté à la position " + String(posSequenceEscalier));
        
          // Vérifier si c'est un clic de bouton (submit=1) et pas un simple rafraîchissement
          if (isFormSubmit && request.indexOf("GET /?sequence_escalier=1&submit=1") != -1) {
            addLog("[wifi] Demande de séquence escalier confirmée via clic de bouton");
            // Démarrer la séquence escalier uniquement si le robot est disponible
            if (deplacementFait) {
              sequenceEnCours = true;
              etapeSequence = 0;
              executerProchainMouvement = true;
              addLog("[wifi] Séquence escalier initialisée");
            } else {
              addLog("[wifi] Impossible de démarrer la séquence: robot en mouvement");
            }
          } else {
            addLog("[wifi] Paramètre sequence_escalier détecté mais pas via clic de bouton - ignoré");
          }
        }
        
        // Vérifier si c'est une demande pour réinitialiser la position du robot
        int posReset = request.indexOf("reset=1");
        
        if (posReset != -1) {
          addLog("[wifi] Détection paramètre reset=1 à la position " + String(posReset));
          if (isFormSubmit) {
            addLog("[wifi] Demande de réinitialisation confirmée");
            // Exécuter la fonction de réinitialisation
            resetRobot();
            
            // Vérifier que la réinitialisation a bien fonctionné
            addLog("[wifi] Vérification après réinitialisation: X=" + String(robotState.x) + ", Y=" + String(robotState.y) + ", Theta=" + String(robotState.theta));
          } else {
            addLog("[wifi] Paramètre reset=1 détecté mais formulaire non soumis");
          }
        } else {
          // Si c'est juste un chargement de page sans soumission
          addLog("[wifi] Page chargée sans commande");
        }
      }

      String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Drawbot WiFi</title>";
      html += "<style>";
      html += "body{font-family:Arial;text-align:center;background:#f2f2f2;padding:20px;}";
      html += "form{background:#fff;padding:20px;border-radius:12px;box-shadow:0 0 10px #888;display:inline-block;margin-bottom:20px;width:80%;max-width:400px;}";
      html += "input{padding:10px;margin:8px;border-radius:5px;width:80%;}";
      html += "input[type=submit]{background:#4CAF50;color:white;border:none;cursor:pointer;}";
      html += "input[type=submit]:hover{background:#45a049;}";
      html += ".log-container{background:#fff;border-radius:12px;box-shadow:0 0 10px #888;padding:15px;margin:0 auto;width:90%;max-width:800px;text-align:left;max-height:400px;overflow-y:auto;}";
      html += ".log-entry{font-family:monospace;font-size:0.9em;margin:3px 0;border-bottom:1px solid #eee;padding-bottom:3px;}";
      html += "h2{color:#333;margin-top:25px;}";
      html += "</style>";

      html += "</head><body>";
      html += "<h1>Drawbot WiFi</h1>";
      html += "<div style='background:#f8f8f8;padding:5px;border-radius:5px;margin-bottom:10px;font-size:0.8em;text-align:right;'>";
      html += "Compilé le " + String(COMPILE_DATE) + " à " + String(COMPILE_TIME);
      html += "</div>";
      
      // Récupérer la valeur d'incrément précédente ou utiliser la valeur par défaut (0.1)
      float increment = 1; // Valeur par défaut
      if (posIncrement != -1) {
        String incrementStr = request.substring(posIncrement + 10, request.indexOf('&', posIncrement + 10));
        float tmpIncrement = incrementStr.toFloat();
        if (tmpIncrement > 0) {
          increment = tmpIncrement;
          addLog("[wifi] Valeur d'incrément: " + String(increment, 2) + " cm");
        }
      }
      
      // Ajouter le champ pour personnaliser la valeur d'incrément
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Valeur d'incrément</h2>";
      html += "<form action='/' method='get'>";
      html += "<input type='number' step='0.01' min='0.01' max='10' name='increment' value='" + String(increment, 2) + "' style='width:100px;'>";
      html += "<input type='submit' value='Appliquer' style='width:auto;'>";
      html += "</form>";
      html += "</div>";
      
      // Ajouter les boutons directionnels pour déplacement rapide
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Déplacement rapide (Absolu)</h2>";
      // Formatter l'affichage de l'incrément avec une virgule au lieu d'un point
      String incrementDisplay = String(increment, 1);
      incrementDisplay.replace('.', ',');
      html += "<p>Déplacement absolu de " + incrementDisplay + " cm</p>";
      html += "<div style='display:grid; grid-template-columns:1fr 1fr 1fr; max-width:180px; margin:0 auto; gap:5px;'>";
      html += "<div></div>";
      html += "<a href='/?dx=0&dy=" + String(increment) + "&type=absolu&increment=" + String(increment) + "&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>+y</a>"; // Flèche vers le haut (Y+)
      html += "<div></div>";
      html += "<a href='/?dx=-" + String(increment) + "&dy=0&type=absolu&increment=" + String(increment) + "&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>-x</a>"; // Flèche vers la gauche (X-)
      html += "<div style='background:#ddd; color:#666; padding:5px; border-radius:5px;'>+" + incrementDisplay + "</div>"; // Centre
      html += "<a href='/?dx=" + String(increment) + "&dy=0&type=absolu&increment=" + String(increment) + "&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>+x</a>"; // Flèche vers la droite (X+)
      html += "<div></div>";
      html += "<a href='/?dx=0&dy=-" + String(increment) + "&type=absolu&increment=" + String(increment) + "&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>-y</a>"; // Flèche vers le bas (Y-)
      html += "<div></div>";
      html += "</div>"; // Fin de la grille
      html += "</div>"; // Fin du conteneur des boutons directionnels absolus
      
      // Ajouter les boutons directionnels pour déplacement relatif avec valeur d'incrément variable
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Déplacement rapide (Relatif)</h2>";
      html += "<p>Déplacement relatif au robot de " + incrementDisplay + " cm</p>";
      html += "<div style='display:grid; grid-template-columns:1fr 1fr 1fr; max-width:200px; margin:0 auto; gap:5px;'>";
      html += "<div></div>";
      html += "<a href='/?dx=" + String(increment) + "&dy=0&type=direct&increment=" + String(increment) + "&submit=1' style='background:#2196F3; color:white; padding:10px; border-radius:5px; text-decoration:none; text-align:center;'>AV</a>";
      html += "<div></div>";
      html += "<a href='/?dx=0&dy=" + String(increment) + "&type=direct&increment=" + String(increment) + "&submit=1' style='background:#2196F3; color:white; padding:10px; border-radius:5px; text-decoration:none; text-align:center;'>G</a>";
      html += "<div style='background:#ddd; color:#666; padding:5px; border-radius:5px; text-align:center;'>+" + incrementDisplay + "</div>";
      html += "<a href='/?dx=0&dy=-" + String(increment) + "&type=direct&increment=" + String(increment) + "&submit=1' style='background:#2196F3; color:white; padding:10px; border-radius:5px; text-decoration:none; text-align:center;'>D</a>";
      html += "<div></div>";
      html += "<a href='/?dx=-" + String(increment) + "&dy=0&type=direct&increment=" + String(increment) + "&submit=1' style='background:#2196F3; color:white; padding:10px; border-radius:5px; text-decoration:none; text-align:center;'>AR</a>";
      html += "<div></div>";
      html += "</div>"; // Fin de la grille
      
      // Affichage des distances parcourues par les roues
      html += "<div style='margin-top:10px; background:#e8f5ff; padding:10px; border-radius:5px; width:100%; max-width:400px; margin-left:auto; margin-right:auto;'>";
      html += "<h3>Distances parcourues par les roues</h3>";
      html += "<table style='width:100%; border-collapse:collapse;'>";
      html += "<tr><th style='text-align:left; padding:5px; border-bottom:1px solid #ccc;'>Roue gauche:</th><td style='text-align:right; padding:5px; border-bottom:1px solid #ccc;'>" + String(distance_en_cm_roue_gauche, 2) + " cm</td></tr>";
      html += "<tr><th style='text-align:left; padding:5px;'>Roue droite:</th><td style='text-align:right; padding:5px;'>" + String(distance_en_cm_roue_droite, 2) + " cm</td></tr>";
      html += "</table>";
      html += "</div>";
      
      html += "</div>"; // Fin du conteneur des boutons directionnels relatifs
      
      // Ajouter le bouton pour la séquence en escalier
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Séquence Escalier</h2>";
      html += "<p>Dessiner un escalier : 20cm en ligne droite, puis 10 pas de 1cm vers le haut, puis 20 pas de 1cm vers la droite</p>";
      html += "<a href='/?sequence_escalier=1&submit=1' style='background:#FF5722; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Lancer la séquence</a>";
      html += sequenceEnCours ? "<p><strong>Séquence en cours : Étape " + String(etapeSequence) + "/" + String(ETAPES_SEQUENCE_MAX) + "</strong></p>" : "";
      html += "</div>"; // Fin du conteneur pour la séquence escalier
      
      // Ajouter une section de test de coordonnées absolues
      // html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      // html += "<h2>Test de Coordonnées</h2>";
      // html += "<p>Vérifier si le robot atteint les coordonnées indiquées</p>";
      // html += "<form action='/' method='get' style='background:#e3f2fd;'>";
      // html += "<div class='input-group'>";
      // html += "<label for='target_x'>Coordonnée X (cm):</label>";
      // html += "<input type='number' step='0.1' name='target_x' id='target_x' value='0' required>";
      // html += "</div>";
      // html += "<div class='input-group'>";
      // html += "<label for='target_y'>Coordonnée Y (cm):</label>";
      // html += "<input type='number' step='0.1' name='target_y' id='target_y' value='0' required>";
      // html += "</div>";
      // html += "<input type='hidden' name='submit' value='1'>";
      // html += "<input type='submit' value='Aller à cette position' style='background:#2196F3;'>";
      // html += "</form>";
      // html += "<p><strong>Position actuelle: X=" + String(robotState.x, 2) + " cm, Y=" + String(robotState.y, 2) + " cm</strong></p>";
      // html += "</div>"; // Fin du conteneur pour le test de coordonnées
      
      // Ajouter le bouton de réinitialisation (reset)
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Réinitialisation</h2>";
      html += "<p>Réinitialiser la position du robot à (0,0) et l'angle à 0°</p>";
      html += "<a href='/?reset=1&submit=1' style='background:#F44336; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Réinitialiser position</a>";
      html += "<div style='margin-bottom:15px; background:#f0f8ff; padding:10px; border-radius:5px; width:100%; max-width:500px; margin-left:auto; margin-right:auto;'>";
      html += "<h3>Positions</h3>";
      html += "<table style='width:100%; border-collapse:collapse;'>";
      html += "<tr style='background:#e0e0e0;'><th style='text-align:left; padding:5px; border-bottom:1px solid #ccc;'>Type</th><th style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>X (cm)</th><th style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>Y (cm)</th><th style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>Angle (°)</th></tr>";
      html += "<tr><td style='text-align:left; padding:5px; border-bottom:1px solid #ccc;'><strong>Actuelle</strong></td><td style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>" + String(robotState.x, 2) + "</td><td style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>" + String(robotState.y, 2) + "</td><td style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>" + String(robotState.theta * 180.0 / PI, 1) + "</td></tr>";
      html += "<tr><td style='text-align:left; padding:5px; border-bottom:1px solid #ccc;'><strong>Théorique</strong></td><td style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>" + String(positionTheorique.x, 2) + "</td><td style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>" + String(positionTheorique.y, 2) + "</td><td style='text-align:center; padding:5px; border-bottom:1px solid #ccc;'>" + String(positionTheorique.theta * 180.0 / PI, 1) + "</td></tr>";
      html += "<tr><td style='text-align:left; padding:5px;'><strong>Réelle</strong></td><td style='text-align:center; padding:5px;'>" + String(positionReelle.x, 2) + "</td><td style='text-align:center; padding:5px;'>" + String(positionReelle.y, 2) + "</td><td style='text-align:center; padding:5px;'>" + String(positionReelle.theta * 180.0 / PI, 1) + "</td></tr>";
      html += "</table>";
      html += "</div>";
      html += "</div>"; // Fin du conteneur pour la réinitialisation
      
      // Affichage des logs
      html += "<h2>Logs</h2>";
      html += "<div class='log-container'>" + getAllLogs() + "</div>";
      html += "</body></html>";
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println("Connection: close");
      client.println();
      client.println(html);
      delay(1);
      client.stop();
    }
  //}

  // Vérifier si un mouvement est en cours
  if (correctionActive && !deplacementFait) {
    bool fini = avancerCorrige();
    if (fini) {
      arreter();
      correctionActive = false;
      deplacementFait = true;
      
      addLog("[position] Mouvement terminé - Robot arrêté");
      addLog("[position] abasolue X: " + String(robotState.x, 2) + " cm | Y: " + String(robotState.y, 2) + " cm | Orientation: " + String(robotState.theta * 180.0 / PI, 1) + "°");
      
      // Si nous sommes dans une séquence automatique, préparer l'étape suivante
      if (sequenceEnCours) {
        etapeSequence++;
        executerProchainMouvement = true;
        // Ajouter un court délai entre les mouvements
        delay(200);
      }

    }
  }
  
  // Si une séquence est en cours, continuer son exécution
  if (sequenceEnCours && deplacementFait) {
    executerSequenceEscalier();
  }
  

}