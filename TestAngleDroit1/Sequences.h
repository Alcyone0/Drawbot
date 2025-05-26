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

// Fonction pour exécuter une séquence automatique de mouvements formant un carré
void executerSequenceAutomatique() {
  // Si la séquence n'est pas déjà en cours, l'initialiser
  if (!sequenceEnCours) {
    addLog("[sequence] Début de la séquence automatique");
    sequenceEnCours = true;
    etapeSequence = 0;
    executerProchainMouvement = true;
  }
  
  // Si nous sommes dans une séquence et qu'il faut exécuter le prochain mouvement
  if (sequenceEnCours && executerProchainMouvement && etapeSequence < ETAPES_SEQUENCE_MAX) {
    float dx = 0.0;
    float dy = 0.0;
    
    // Déterminer la direction du mouvement en fonction de l'étape
    if (etapeSequence < 10) {
      // 10 premiers pas : vers la droite (X+)
      dx = 0.1;
      dy = 0.0;
      addLog("[sequence] Étape " + String(etapeSequence+1) + "/30 : Déplacement à droite");
    } else if (etapeSequence < 20) {
      // 10 pas suivants : vers le haut (Y+)
      dx = 0.0;
      dy = 0.1;
      addLog("[sequence] Étape " + String(etapeSequence+1) + "/30 : Déplacement en haut");
    } else {
      // 10 derniers pas : vers la droite (X+)
      dx = 0.1;
      dy = 0.0;
      addLog("[sequence] Étape " + String(etapeSequence+1) + "/30 : Déplacement à droite");
    }
    
    // Convertir les coordonnées absolues en coordonnées relatives au robot
    DeltaXY absolutePoint(dx, dy);
    DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(absolutePoint, robotState);
    
    // Lancer le mouvement
    demarer(robotCoord.x, robotCoord.y);
    
    // Indiquer qu'il faut attendre la fin du mouvement avant le prochain
    executerProchainMouvement = false;
    
    // Si nous avons atteint la fin de la séquence
    if (etapeSequence >= ETAPES_SEQUENCE_MAX - 1) {
      sequenceEnCours = false;
      addLog("[sequence] Fin de la séquence automatique");
    }
  }
}

// Fonction pour dessiner un cercle de rayon 4cm en 100 points
void executerSequenceCercle() {
  // Si la séquence n'est pas déjà en cours, l'initialiser
  if (!sequenceCercleEnCours) {
    addLog("[cercle] Début de la séquence cercle");
    sequenceCercleEnCours = true;
    etapeCercle = 0;
    executerProchainPointCercle = true;
  }
  
  // Si nous sommes dans une séquence cercle et qu'il faut exécuter le prochain point
  if (sequenceCercleEnCours && executerProchainPointCercle && etapeCercle < ETAPES_CERCLE_MAX) {
    // Calculer les coordonnées du point sur le cercle
    float angle = 2.0 * PI * etapeCercle / ETAPES_CERCLE_MAX;
    float dx = RAYON_CERCLE * cos(angle);
    float dy = RAYON_CERCLE * sin(angle);
    
    addLog("[cercle] Point " + String(etapeCercle+1) + "/" + String(ETAPES_CERCLE_MAX) + 
           " : Angle=" + String(angle * 180.0 / PI, 1) + "°, X=" + String(dx, 2) + ", Y=" + String(dy, 2));
    
    // Convertir les coordonnées absolues en coordonnées relatives au robot
    DeltaXY absolutePoint(dx, dy);
    DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(absolutePoint, robotState);
    
    // Lancer le mouvement
    demarer(robotCoord.x, robotCoord.y);
    
    // Indiquer qu'il faut attendre la fin du mouvement avant le prochain
    executerProchainPointCercle = false;
    
    // Si nous avons atteint la fin de la séquence
    if (etapeCercle >= ETAPES_CERCLE_MAX - 1) {
      sequenceCercleEnCours = false;
      addLog("[cercle] Fin de la séquence cercle");
    }
  }
}

#endif // SEQUENCES_H
