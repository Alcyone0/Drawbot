#include "calculs.h"
#include "logger.h"

WheelDistances calculerDistancesRoues(DeltaXY robotRelativePoint) {
  WheelDistances distances;
  distances.left = robotRelativePoint.y + (LARGEUR_ROBOT / LONGUEUR_ROBOT) * robotRelativePoint.x;
  distances.right = robotRelativePoint.y - (LARGEUR_ROBOT / LONGUEUR_ROBOT) * robotRelativePoint.x;  
  return distances;
}

RobotState calculerNouvellePosition(WheelDistances distances) {
  // Préparer la structure RobotState à renvoyer
  RobotState newState(robotState.x, robotState.y, robotState.theta);
  
  // Calcul de l'angle relatif basé sur la différence des distances des roues
  float deltaRoues = distances.left - distances.right;
  float angleRelatif = atan(deltaRoues / (LARGEUR_ROBOT));
  
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
  
  return newState;
}

// Fonction pour convertir les coordonnées absolues en coordonnées relatives au robot
// Utilise une transformation en coordonnées cylindriques/polaires
DeltaXY convertAbsoluteToRobotCoordinates(DeltaXY targetPoint, RobotState robotState) {
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
  
  // Retourner les coordonnées relatives au robot
  return robotRelativePoint;
}
