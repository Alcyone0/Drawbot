#include "calculs.h"
#include "logger.h"

WheelDistances calculerDistancesRoues(DeltaXY robotRelativePoint) {
  WheelDistances distances;
  distances.left = robotRelativePoint.y + (LARGEUR_ROBOT / LONGUEUR_ROBOT) * robotRelativePoint.x;
  distances.right = robotRelativePoint.y - (LARGEUR_ROBOT / LONGUEUR_ROBOT) * robotRelativePoint.x;  
  return distances;
}

RobotState calculerNouvellePosition(WheelDistances distances) {
  RobotState newState(robotState.x, robotState.y, robotState.theta);
  
  float distance = (distances.left + distances.right) / 2.0;

  float deltaRoues = distances.left - distances.right;
  float angleRelatif = atan2(deltaRoues, LARGEUR_ROBOT);
  float angle = robotState.theta + angleRelatif;

  // Mise à jour de la position du robot en fonction de son orientation
  newState.x = robotState.x + distance * cos(angle);
  newState.y = robotState.y + distance * sin(angle);
  newState.theta = angle;
  
  // Normaliser l'angle entre -PI et PI
  while (newState.theta > PI) newState.theta -= 2*PI;
  while (newState.theta < -PI) newState.theta += 2*PI;
  
  return newState;
}

// Fonction pour convertir les coordonnées absolues en coordonnées relatives au robot
// Utilise une transformation en coordonnées cylindriques/polaires
DeltaXY convertAbsoluteToRobotCoordinates(DeltaXY targetPoint, RobotState robotState) {
  
  // 1. Convertir les coordonnées absolues en coordonnées polaires
  float deltaX = targetPoint.x - robotState.x;
  float deltaY = targetPoint.y - robotState.y;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  float angle = atan2(deltaY, deltaX); // Angle par rapport à l'axe X
  float angleRelatif = angle - robotState.theta;
  
  // 3. Reconvertir en coordonnées cartésiennes relatives au robot
  float deltaRobotX = distance * cos(angleRelatif); // X dans le repère du robot correspond à un déplacement latéral
  float deltaRobotY = distance * sin(angleRelatif); // Y dans le repère du robot correspond à un déplacement avant
  
  // Créer un point pour les coordonnées relatives au robot
  DeltaXY robotRelativePoint(deltaRobotX, deltaRobotY);
  
  // Retourner les coordonnées relatives au robot
  return robotRelativePoint;
}
