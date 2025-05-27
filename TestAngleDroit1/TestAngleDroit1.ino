#include <Wire.h>
#include "wifi_manager.h"
#include <SparkFunLSM6DS3.h>
#include <math.h>
#include "RobotStructures.h"
#include "Sequences.h"

/* ===== IMU ===== */
LSM6DS3 imu(I2C_MODE, 0x6B);
float biaisGyroZ = 0.0;

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
const float IMPULSIONS_PAR_CM = 34.0;
const int   PWM_MIN = 80;
const int   PWM_MAX = 110;
const float DIST_STYLO_CM = 13.0;
const float LARGEUR_ROBOT = 8.5/2;
const float LONGUEUR_ROBOT = 13.0; // Distance entre l'axe des roues et le stylo

/* ===== VARIABLES INTERNES ===== */
long seuilImpulsionsRoueGauche = 0;
long seuilImpulsionsRoueDroite = 0;
float distance_en_cm_roue_gauche = 0;
float distance_en_cm_roue_droite = 0;
bool directionAvantGauche = true; // true = avant, false = arrière
bool directionAvantDroite = true; // true = avant, false = arrière

/* ===== VARIABLES POUR SÉQUENCE AUTOMATIQUE ===== */
bool sequenceEnCours = false;
int etapeSequence = 0;
const int ETAPES_SEQUENCE_MAX = 30; // 10 pas à droite + 10 pas en haut + 10 pas à droite
bool executerProchainMouvement = true;

/* ===== VARIABLES POUR SÉQUENCE CERCLE ===== */
bool sequenceCercleEnCours = false;
int etapeCercle = 0;
const int ETAPES_CERCLE_MAX = 100; // 100 points pour le cercle
const float RAYON_CERCLE = 0.5; // Rayon du cercle en cm (diamètre 1cm)
bool executerProchainPointCercle = true;

/* ===== PROTOTYPES DE FONCTIONS ===== */
void demarer(float deltaX, float deltaY); // Déclaration anticipée
void executerSequenceAutomatique(); // Déclaration pour la séquence automatique
void executerSequenceCercle(); // Déclaration pour la séquence de cercle

/* ===== STRUCTURES ===== */
// Les structures ont été déplacées dans le fichier RobotStructures.h

/* ===== POSITION DU ROBOT ===== */
RobotState robotState;  // Position et orientation du robot

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

// Calcule les distances des deux roues en fonction du déplacement demandé et met à jour la position du robot
WheelDistances calculerDistancesRoues(DeltaXY robotRelativePoint) {
  WheelDistances distances;
  distances.left = robotRelativePoint.y + (LARGEUR_ROBOT / LONGUEUR_ROBOT) * robotRelativePoint.x;
  distances.right = robotRelativePoint.y - (LARGEUR_ROBOT / LONGUEUR_ROBOT) * robotRelativePoint.x;  

  addLog("[calculerDistancesRoues] Distances roues: " + String(distances.left, 1) + " cm, " + String(distances.right, 1) + " cm");

  return distances;
}

RobotState calculerNouvellePosition(WheelDistances distances) {
  // Préparer la structure RobotState à renvoyer
  RobotState newState(robotState.x, robotState.y, robotState.theta);
  
  // Calcul de l'angle relatif basé sur la différence des distances des roues
  float deltaRoues = distances.left - distances.right;
  float angleRelatif = atan(deltaRoues / (LARGEUR_ROBOT));
  
  addLog("[calculerNouvellePosition] Robot theta avant: " + String(robotState.theta * 180.0 / PI, 1) + "°");
  
  // Calcul du déplacement moyen pour la position
  float deltaMoyen = (distances.left + distances.right) / 2.0;
  
  // Mise à jour de la position du robot en fonction de son orientation
  newState.x = robotState.x + deltaMoyen * cos(robotState.theta + angleRelatif/2.0);
  newState.y = robotState.y + deltaMoyen * sin(robotState.theta + angleRelatif/2.0);
  
  // Mise à jour de l'angle du robot
  newState.theta = robotState.theta + angleRelatif;
  
  // Normaliser l'angle entre -PI et PI
  while (newState.theta > PI) newState.theta -= 2*PI;
  while (newState.theta < -PI) newState.theta += 2*PI;
  
  
  addLog("[calculerNouvellePosition] Nouvelle position robot: X=" + String(newState.x, 1) + " cm, Y=" + String(newState.y, 1) + " cm"); 
  
  return newState;
}

// Fonction pour convertir les coordonnées absolues en coordonnées relatives au robot
// Utilise une transformation en coordonnées cylindriques/polaires
DeltaXY convertAbsoluteToRobotCoordinates(DeltaXY targetPoint, RobotState robotState) {
  addLog("[convertCoords] Début de la conversion de coordonnées absolues en relatives");
  
  // Vérifier si le déplacement demandé est trop petit
  if (abs(targetPoint.x) < 0.001 && abs(targetPoint.y) < 0.001) {
    addLog("[convertCoords] Déplacement trop petit, évitement de division par zéro");
    return DeltaXY(0, 0);
  }
  
  // 1. Convertir les coordonnées absolues en coordonnées polaires
  float distance = sqrt(targetPoint.x * targetPoint.x + targetPoint.y * targetPoint.y);
  float angle = atan2(targetPoint.y, targetPoint.x); // Angle par rapport à l'axe X
  
  // 2. Ajuster l'angle en fonction de l'orientation du robot
  float angleRelatif = angle - robotState.theta;
  
  // 3. Reconvertir en coordonnées cartésiennes relatives au robot
  float deltaRobotX = distance * cos(angleRelatif); // X dans le repère du robot correspond à un déplacement latéral
  float deltaRobotY = distance * sin(angleRelatif); // Y dans le repère du robot correspond à un déplacement avant
  
  // Créer un point pour les coordonnées relatives au robot
  DeltaXY robotRelativePoint(deltaRobotX, deltaRobotY);
  
  // Afficher les valeurs pour débogage
  addLog("[convertCoords] Absolue (" + String(targetPoint.x, 2) + "," + String(targetPoint.y, 2) + ") -> " +
         "Distance=" + String(distance, 2) + "cm, Angle=" + String(angle * 180.0 / PI, 1) + "°");
  addLog("[convertCoords] Angle relatif: " + String(angleRelatif * 180.0 / PI, 1) + "° -> Robot (" + 
         String(robotRelativePoint.x, 2) + "," + String(robotRelativePoint.y, 2) + ")");
         
  // Retourner les coordonnées relatives au robot
  return robotRelativePoint;
}

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

// Fonction pour démarrer le mouvement du robot
void demarer(float deltaX, float deltaY) {
  addLog("[demarer] DX=" + String(deltaX) + ", DY=" + String(deltaY));
  // Vérifier si les valeurs sont zéro
  if (abs(deltaX) < 0.01 && abs(deltaY) < 0.01) {
    addLog("[demarer] Valeurs trop petites, pas de mouvement");
    return;
  }

  // Créer une structure DeltaXY pour les coordonnées relatives
  DeltaXY robotRelativePoint(deltaX, deltaY);
  
  // Calculer les distances pour chaque roue avec la nouvelle fonction
  WheelDistances distances = calculerDistancesRoues(robotRelativePoint);
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
  
  // Réinitialiser les compteurs
  countLeft = 0;
  countRight = 0;
  
  // Activer le mouvement
  correctionActive = true;
  deplacementFait = false;
  addLog("[demarer] Début du mouvement");
  
  // Calculer et mettre à jour la position du robot
  robotState = calculerNouvellePosition(distances);
  addLog("[demarer] Nouvelle position robot: X=" + String(robotState.x, 1) + " cm, Y=" + String(robotState.y, 1) + " cm");
} 

// Les fonctions executerSequenceAutomatique et executerSequenceCercle ont été déplacées dans le fichier Sequences.h

// Fonction pour réinitialiser la position et l'orientation du robot
void resetRobot() {
  // Arrêter le robot et toute séquence en cours
  arreter();
  sequenceEnCours = false;
  sequenceCercleEnCours = false;
  deplacementFait = true;
  correctionActive = false;
  etapeSequence = 0;
  etapeCercle = 0;
  executerProchainMouvement = true;
  executerProchainPointCercle = true;
  
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
  
  addLog("[reset] Position et orientation réinitialisées à zéro");
  addLog("[reset] X=0.0, Y=0.0, Theta=0.0°");
}

bool avancerCorrige() {
  // Configurer la direction des moteurs en fonction des valeurs calculées
  if (directionAvantDroite) {
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
  } else {
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
  }
  
  if (directionAvantGauche) {
    digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, HIGH);
  } else {
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
  }
  
  int erreurGauche = seuilImpulsionsRoueGauche - countLeft;
  int erreurDroite = seuilImpulsionsRoueDroite - countRight;

  float Kp = 10;
  int correctionGauche = Kp * erreurGauche;
  int correctionDroite = Kp * erreurDroite;

  int pwmD = constrain(PWM_MIN + correctionDroite,PWM_MIN , PWM_MAX);
  int pwmG = constrain(PWM_MIN + correctionGauche, PWM_MIN, PWM_MAX);

  analogWrite(EN_D, pwmD);
  analogWrite(EN_G, pwmG);

  bool fini = (countLeft >= seuilImpulsionsRoueGauche) && (countRight >= seuilImpulsionsRoueDroite);
  if (fini) {
    addLog("[avancerCorrige] Déplacement terminé");
  }
  return fini;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  addLog("[setup] Drawbot démarré");
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

  // S'assurer que les séquences sont désactivées au démarrage
  sequenceEnCours = false;
  sequenceCercleEnCours = false;
  deplacementFait = true; // Pour éviter que le robot ne bouge au démarrage
  
  setupWiFi();
  
  // S'assurer que le robot est arrêté au démarrage
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