#include "PID.h"
#include "logger.h"

// Définition directe des broches pour éviter les problèmes de références
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16
#define EN_D   23
#define EN_G    4

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
