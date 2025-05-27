#include <Arduino.h>
#include "PID.h"
#include "logger.h"
#include "calculs.h"
#include "RobotStructures.h"

// Définition directe des broches pour éviter les problèmes de références
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16
#define EN_D   23
#define EN_G    4

// Définition des broches d'encodeur
const int encoderLeftA  = 27;
const int encoderRightA = 33;

/* ===== RÉFÉRENCES AUX CONSTANTES ===== */
// Ces constantes sont définies dans le fichier principal
extern const float IMPULSIONS_PAR_CM;
extern const int PWM_MIN;
extern const int PWM_MAX;

/* ===== VARIABLES DE CONTRÔLE D'ÉTAT ===== */
extern bool correctionActive;
extern bool deplacementFait;

/* ===== VARIABLES DE SÉQUENCE ===== */
extern bool sequenceEnCours;
extern bool sequenceCercleEnCours;
extern int etapeSequence;
extern int etapeCercle;
extern bool executerProchainMouvement;
extern bool executerProchainPointCercle;

/* ===== POSITION DU ROBOT ===== */
extern RobotState robotState;

/* ===== VARIABLES INTERNES DE CONTRÔLE MOTEUR ===== */
long seuilImpulsionsRoueGauche = 0;
long seuilImpulsionsRoueDroite = 0;
float distance_en_cm_roue_gauche = 0;
float distance_en_cm_roue_droite = 0;
bool directionAvantGauche = true; // true = avant, false = arrière
bool directionAvantDroite = true; // true = avant, false = arrière

// Fonctions pour les encodeurs - déclarées ici car elles sont nécessaires pour initMotorPins
extern void countLeftEncoder();
extern void countRightEncoder();

// Initialisation des broches des moteurs et encodeurs
void initMotorPins() {
  pinMode(IN_1_D, OUTPUT); pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT); pinMode(IN_2_G, OUTPUT);
  pinMode(EN_D, OUTPUT);   pinMode(EN_G, OUTPUT);
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderRightA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLeftA), countLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), countRightEncoder, RISING);
}

void arreter()
{
  analogWrite(EN_D, 0);  analogWrite(EN_G, 0);
  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, LOW);
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

  int pwmD = constrain(PWM_MIN + correctionDroite, PWM_MIN, PWM_MAX);
  int pwmG = constrain(PWM_MIN + correctionGauche, PWM_MIN, PWM_MAX);

  analogWrite(EN_D, pwmD);
  analogWrite(EN_G, pwmG);

  bool fini = (countLeft >= seuilImpulsionsRoueGauche) && (countRight >= seuilImpulsionsRoueDroite);
  if (fini) {
    addLog("[avancerCorrige] Déplacement terminé");
  }
  return fini;
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