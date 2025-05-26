#include <Wire.h>
#include <WiFi.h>
#include <SparkFunLSM6DS3.h>
#include <math.h>

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
const float IMPULSIONS_PAR_CM = 34.0;
const int   PWM_MIN = 80;
const int   PWM_MAX = 110;
const float DIST_STYLO_CM = 13.0;
const float LARGEUR_ROBOT = 8.5;
const float LONGUEUR_ROBOT = 13.0; // Distance entre l'axe des roues et le stylo

/* ===== VARIABLES INTERNES ===== */
long seuilImpulsionsRoueGauche = 0;
long seuilImpulsionsRoueDroite = 0;
float distance_en_cm_roue_gauche = 0;
float distance_en_cm_roue_droite = 0;
float deltaX_wifi = 0;
float deltaY_wifi = 0;
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
void naviguerVersPointAbsolu(float targetX, float targetY); // Déclaration pour la navigation vers un point absolu

/* ===== STRUCTURES ===== */
struct DeltaXY {
  float x;
  float y;
  
  // Constructeur par défaut
  DeltaXY() : x(0), y(0) {}
  
  // Constructeur avec paramètres
  DeltaXY(float _x, float _y) : x(_x), y(_y) {}
};

// Structure pour représenter la position et l'orientation du robot
struct RobotState {
  float x;
  float y;
  float theta;        // Orientation du robot en radians
  
  // Constructor
  RobotState(float _x, float _y, float _theta) 
    : x(_x), y(_y), theta(_theta) {}
    
  // Constructeur par défaut
  RobotState() : x(0), y(0), theta(0) {}
};

struct WheelDistances {
  float left;   // Distance pour la roue gauche en cm
  float right;  // Distance pour la roue droite en cm
  
  // Constructeur par défaut
  WheelDistances() : left(0), right(0) {}
  
  // Constructeur avec paramètres
  WheelDistances(float _left, float _right) : left(_left), right(_right) {};
};

/* ===== POSITION DU ROBOT ===== */
float RobotX = 0.0;      // Position X du robot en cm
float RobotY = 0.0;      // Position Y du robot en cm
float RobotTheta = 0.0;  // Orientation du robot en radians

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

// Calcule les distances des deux roues en fonction du déplacement demandé sans mettre à jour la position du robot
WheelDistances calculerDistancesRoues(float deltaRobotX, float deltaRobotY) {
  WheelDistances distances;
  distances.left = deltaRobotY + (LARGEUR_ROBOT / LONGUEUR_ROBOT) * deltaRobotX;
  distances.right = deltaRobotY - (LARGEUR_ROBOT / LONGUEUR_ROBOT) * deltaRobotX;  

  // Calcul de l'angle relatif basé sur la différence des distances des roues
  float deltaRoues = distances.left - distances.right;
  float angleRelatif = atan(deltaRoues / LARGEUR_ROBOT);
  
  addLog("[calculerDistancesRoues] Robot theta actuel: " + String(RobotTheta * 180.0 / PI, 1) + "°");
  addLog("[calculerDistancesRoues] DeltaX=" + String(deltaRobotX, 2) + ", DeltaY=" + String(deltaRobotY, 2));
  addLog("[calculerDistancesRoues] Distances calculées - Gauche: " + String(distances.left, 2) + " cm, Droite: " + String(distances.right, 2) + " cm");
  addLog("[calculerDistancesRoues] Angle relatif calculé: " + String(angleRelatif * 180.0 / PI, 1) + "°");
  
  // Ne pas mettre à jour la position du robot ici
  // Les positions seront mises à jour uniquement après que le mouvement est terminé
  
  return distances;
}

// Fonction pour convertir les coordonnées absolues en coordonnées relatives au robot
// Utilise une transformation en coordonnées cylindriques/polaires
DeltaXY convertAbsoluteToRobotCoordinates(float deltaAbsoluteX, float deltaAbsoluteY) {
  addLog("[convertCoords] Début de la conversion de coordonnées absolues en relatives");
  
  // Créer les structures à partir des variables globales
  DeltaXY targetPoint(deltaAbsoluteX, deltaAbsoluteY);
  RobotState robotState(RobotX, RobotY, RobotTheta);
  
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

// Fonction pour calculer l'angle theta entre l'orientation du robot et la direction vers le point cible
// Basée sur le code Python fourni
float calculTheta(float x, float y, float omega, float xt, float yt) {
  // Vecteur directeur du robot (unitaire)
  float u_x = cos(omega);
  float u_y = sin(omega);
  
  // Vecteur directeur de la trajectoire
  float v_x = xt - x;
  float v_y = yt - y;
  
  // Norme de v
  float norm_v = sqrt(v_x*v_x + v_y*v_y);
  
  if (norm_v < 0.001) {
    return 0.0; // Éviter division par zéro
  }
  
  // Normaliser v
  v_x /= norm_v;
  v_y /= norm_v;
  
  // Produit scalaire
  float dot_product = u_x * v_x + u_y * v_y;
  
  // Limiter le produit scalaire entre -1 et 1 pour éviter les erreurs numériques
  if (dot_product > 1.0) dot_product = 1.0;
  if (dot_product < -1.0) dot_product = -1.0;
  
  // Calcul de theta (en radians)
  float theta_rad = acos(dot_product);
  
  // Déterminer le signe de theta (pour savoir si tourner à gauche ou droite)
  // en utilisant le produit vectoriel (déterminant)
  float cross = u_x * v_y - u_y * v_x;
  if (cross < 0) {
    theta_rad = -theta_rad;  // tourner à droite
  }
  
  return theta_rad;
}

// Structure pour stocker les résultats du calcul de theta et des distances
typedef struct {
  float theta_deg;
  float distance;
  float distance_gauche;
  float distance_droite;
} ThetaEtDistances;

// Déclaration anticipée de la fonction
ThetaEtDistances calculerThetaEtDistance(float x, float y, float omega_deg, float xt, float yt);

// Fonction pour calculer l'angle theta et les distances des roues
// Version simplifiée pour une meilleure précision
ThetaEtDistances calculerThetaEtDistance(float x, float y, float omega_deg, float xt, float yt) {
  ThetaEtDistances result;
  
  // Vecteur vers la cible dans le repère absolu
  float dx = xt - x;
  float dy = yt - y;
  
  // Distance euclidienne jusqu'à la cible
  result.distance = sqrt(dx*dx + dy*dy);
  
  if (result.distance < 0.01) {
    // Si on est déjà sur la cible
    result.theta_deg = 0.0;
    result.distance_gauche = 0.0;
    result.distance_droite = 0.0;
    return result;
  }
  
  // Angle cible dans le repère absolu
  float target_angle = atan2(dy, dx);
  
  // Différence d'angle entre l'orientation actuelle et la cible
  float omega_rad = omega_deg * PI / 180.0;
  float delta_angle = target_angle - omega_rad;
  
  // Normaliser l'angle entre -PI et PI
  while (delta_angle > PI) delta_angle -= 2*PI;
  while (delta_angle < -PI) delta_angle += 2*PI;
  
  result.theta_deg = delta_angle * 180.0 / PI;  // Conversion en degrés
  
  // Calcul des distances des roues en utilisant une approche en deux temps
  // 1. Rotation sur place pour s'orienter vers la cible
  // 2. Déplacement en ligne droite vers la cible
  
  // Distance de rotation (arc de cercle)
  float distance_rotation = abs(delta_angle) * (LARGEUR_ROBOT / 2.0);
  
  // Si l'angle est très petit, on se contente d'avancer tout droit
  if (abs(delta_angle) < 0.01) {
    result.distance_gauche = result.distance;
    result.distance_droite = result.distance;
  } else {
    // Sinon, on calcule les distances pour chaque roue
    if (delta_angle > 0) {
      // Rotation vers la gauche
      result.distance_gauche = -distance_rotation;
      result.distance_droite = distance_rotation;
    } else {
      // Rotation vers la droite
      result.distance_gauche = distance_rotation;
      result.distance_droite = -distance_rotation;
    }
  }
  
  // Journalisation pour le débogage
  addLog("[calculerThetaEtDistance] Theta: " + String(result.theta_deg, 2) + "°");
  addLog("[calculerThetaEtDistance] Distance: " + String(result.distance, 2) + " cm");
  addLog("[calculerThetaEtDistance] Roue G: " + String(result.distance_gauche, 2) + " cm");
  addLog("[calculerThetaEtDistance] Roue D: " + String(result.distance_droite, 2) + " cm");
  
  return result;
}

// Fonction pour naviguer vers un point absolu sur la feuille (x, y)
// Cette fonction calcule le déplacement nécessaire pour atteindre le point cible
// depuis la position actuelle du robot, en tenant compte de son orientation
void naviguerVersPointAbsolu(float targetX, float targetY) {
  addLog("[navPoint] Navigation vers le point absolu (" + String(targetX, 2) + "," + String(targetY, 2) + ")");
  addLog("[navPoint] Position actuelle: (" + String(RobotX, 2) + "," + String(RobotY, 2) + "), Angle: " + String(RobotTheta * 180.0 / PI, 1) + "°");
  
  // Vérifier si on est déjà au point cible (ou très proche)
  float distance = sqrt(pow(targetX - RobotX, 2) + pow(targetY - RobotY, 2));
  if (distance < 0.01) {
    addLog("[navPoint] Déjà au point cible ou très proche, pas de déplacement nécessaire");
    return;
  }
  
  // Calculer l'angle et les distances nécessaires
  ThetaEtDistances resultat = calculerThetaEtDistance(RobotX, RobotY, RobotTheta * 180.0 / PI, targetX, targetY);
  
  // Mettre à jour les distances des roues
  distance_en_cm_roue_gauche = resultat.distance_gauche;
  distance_en_cm_roue_droite = resultat.distance_droite;
  
  // Déterminer les directions des roues
  directionAvantGauche = (distance_en_cm_roue_gauche >= 0);
  directionAvantDroite = (distance_en_cm_roue_droite >= 0);
  
  // Calculer les seuils d'impulsions (toujours en valeur absolue)
  seuilImpulsionsRoueGauche = abs(distance_en_cm_roue_gauche * IMPULSIONS_PAR_CM);
  seuilImpulsionsRoueDroite = abs(distance_en_cm_roue_droite * IMPULSIONS_PAR_CM);
  
  // Journalisation
  addLog("[navPoint] Angle de rotation: " + String(resultat.theta_deg, 2) + "°");
  addLog("[navPoint] Distance à parcourir: " + String(resultat.distance, 2) + " cm");
  addLog("[navPoint] Distance roue G: " + String(distance_en_cm_roue_gauche, 2) + " cm");
  addLog("[navPoint] Distance roue D: " + String(distance_en_cm_roue_droite, 2) + " cm");
  
  // Activer le mouvement
  correctionActive = true;
  deplacementFait = false;
  
  // Réinitialiser les compteurs
  countLeft = 0;
  countRight = 0;
  
  addLog("[navPoint] Début du mouvement vers la cible");
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


// Variables pour stocker les données du dernier mouvement
float lastDeltaX = 0.0;
float lastDeltaY = 0.0;

// Fonction pour démarrer le mouvement du robot
void demarer(float deltaX, float deltaY) {
  addLog("[demarer] DX=" + String(deltaX) + ", DY=" + String(deltaY));
  // Vérifier si les valeurs sont zéro
  if (abs(deltaX) < 0.01 && abs(deltaY) < 0.01) {
    addLog("[demarer] Valeurs trop petites, pas de mouvement");
    return;
  }
  
  // Sauvegarder les deltas pour les utiliser lors de la mise à jour de la position
  lastDeltaX = deltaX;
  lastDeltaY = deltaY;

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
  
  // Réinitialiser les compteurs
  countLeft = 0;
  countRight = 0;
  
  // Activer le mouvement
  correctionActive = true;
  deplacementFait = false;
  addLog("[demarer] Début du mouvement");
} 

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
    DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(dx, dy);
    
    // Lancer le mouvement
    demarer(robotCoord.x, robotCoord.y);
    
    // Indiquer qu'il faut attendre la fin du mouvement avant le prochain
    executerProchainMouvement = false;
  }
  
  // Si nous avons terminé toutes les étapes
  if (etapeSequence >= ETAPES_SEQUENCE_MAX && sequenceEnCours) {
    sequenceEnCours = false;
    addLog("[sequence] Séquence automatique terminée");
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
  
  // Si nous sommes dans une séquence cercle et qu'il faut exécuter le prochain mouvement
  if (sequenceCercleEnCours && executerProchainPointCercle && etapeCercle < ETAPES_CERCLE_MAX) {
    // Calculer l'angle en radians pour cette étape (de 0 à 2π)
    float angle = 2.0 * PI * etapeCercle / ETAPES_CERCLE_MAX;
    
    // Calculer les coordonnées absolues du point du cercle (relatif à la position actuelle)
    float dx = RAYON_CERCLE * cos(angle) - RAYON_CERCLE * cos(2.0 * PI * (etapeCercle - 1) / ETAPES_CERCLE_MAX);
    float dy = RAYON_CERCLE * sin(angle) - RAYON_CERCLE * sin(2.0 * PI * (etapeCercle - 1) / ETAPES_CERCLE_MAX);
    
    // Pour le premier point, on se déplace juste au début du cercle sans calcul différentiel
    if (etapeCercle == 0) {
      dx = RAYON_CERCLE;
      dy = 0.0;
    }
    
    addLog("[cercle] Étape " + String(etapeCercle+1) + "/" + String(ETAPES_CERCLE_MAX) + " : Angle=" + String(angle * 180.0 / PI, 1) + "°, dx=" + String(dx, 3) + ", dy=" + String(dy, 3));
    
    // Convertir les coordonnées absolues en coordonnées relatives au robot
    DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(dx, dy);
    
    // Lancer le mouvement
    demarer(robotCoord.x, robotCoord.y);
    
    // Indiquer qu'il faut attendre la fin du mouvement avant le prochain
    executerProchainPointCercle = false;
  }
  
  // Si nous avons terminé toutes les étapes
  if (etapeCercle >= ETAPES_CERCLE_MAX && sequenceCercleEnCours) {
    sequenceCercleEnCours = false;
    addLog("[cercle] Séquence cercle terminée");
  }
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
  deltaX_wifi = 0;
  deltaY_wifi = 0;
  
  // Réinitialiser les compteurs d'encodeurs
  countLeft = 0;
  countRight = 0;
  
  // Réinitialiser la position et l'orientation
  RobotX = 0.0;
  RobotY = 0.0;
  RobotTheta = 0.0;
  
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
  
  WiFi.softAP(ssid, password);
  server.begin();
  String ipAddress = WiFi.softAPIP().toString();
  addLog("[setup] WiFi AP démarré - IP: " + ipAddress);
  
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
  
  // Ne traiter les requêtes WiFi que si le robot n'est pas en train de se déplacer
  if (deplacementFait) {
    WiFiClient client = server.available();
    if (client) {
      while (!client.available()) delay(1);
      String request = client.readStringUntil('\r');
      client.flush();

      int posX = request.indexOf("dx=");
      int posY = request.indexOf("dy=");
      int posType = request.indexOf("type=");
      
      // Vérifier si c'est une vraie requête de formulaire avec des paramètres
      bool isFormSubmit = false;
      
      // Nouveau: vérifier le paramètre de soumission explicite
      int posSubmit = request.indexOf("submit=1");
      
      // Vérifier si la requête contient "GET /?" et le paramètre de soumission
      if (request.indexOf("GET /?targetX=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de navigation vers point absolu recu");
      } else if (request.indexOf("GET /?reset=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de réinitialisation recu");
      } else if (request.indexOf("GET /?sequence=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de séquence carré recu");
      } else if (request.indexOf("GET /?cercle=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de séquence cercle recu");
      } else if (request.indexOf("GET /?dx=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de mouvement recu");
      } else if (request.indexOf("GET /?dx=") != -1 || request.indexOf("GET /?sequence=") != -1 || request.indexOf("GET /?cercle=") != -1 || request.indexOf("GET /?reset=") != -1 || request.indexOf("GET /?targetX=") != -1) {
        // C'est un rechargement de page avec les paramètres dans l'URL
        addLog("[wifi] Recharge de page détectée, mouvement ignoré");
      }
      
      if (posX != -1 && posY != -1 && isFormSubmit) {
        // Déterminer le type de coordonnées (absolu ou robot)
        bool isAbsoluteCoords = true; // Par défaut, on considère les coordonnées comme absolues
        
        if (posType != -1) {
          String typeValue = request.substring(posType + 5, request.indexOf('&', posType + 5));
          if (typeValue == "robot") {
            isAbsoluteCoords = false;
            addLog("[wifi] Type de coordonnées: relatives au robot");
          } else {
            addLog("[wifi] Type de coordonnées: absolues");
          }
        } else {
          addLog("[wifi] Type non spécifié, utilisation des coordonnées absolues par défaut");
        }
        float dx = request.substring(posX + 3, request.indexOf('&', posX)).toFloat();
        float dy = request.substring(posY + 3).toFloat();
        addLog("[wifi] Valeurs reçues: dx=" + String(dx) + ", dy=" + String(dy));
        
        if (isAbsoluteCoords) {
          // Convertir les coordonnées absolues en coordonnées relatives au robot
          addLog("[wifi] Conversion de coordonnées absolues vers robot");
          DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(dx, dy);
          // Démarrer le mouvement avec les coordonnées relatives
          demarer(robotCoord.x, robotCoord.y);
        } else {
          // Utiliser directement les coordonnées relatives au robot
          addLog("[wifi] Utilisation directe des coordonnées relatives au robot");
          demarer(dx, dy);
        }
      } else {
        // Vérifier si c'est une demande pour lancer la séquence automatique (carré ou cercle), réinitialiser ou naviguer
        int posSequence = request.indexOf("sequence=1");
        int posCercle = request.indexOf("cercle=1");
        int posReset = request.indexOf("reset=1");
        int posTargetX = request.indexOf("targetX=");
        int posTargetY = request.indexOf("targetY=");
        
        // Vérifier si c'est une demande de navigation vers un point absolu
        if (posTargetX != -1 && isFormSubmit) {
          addLog("[wifi] Demande de navigation vers un point absolu détectée");
          // Déterminer la position du paramètre targetY
          posTargetY = request.indexOf("targetY=");
          
          if (posTargetY != -1) {
            // Extraire les valeurs X et Y
            float targetX = request.substring(posTargetX + 8, request.indexOf('&', posTargetX)).toFloat();
            // Vérifier si le paramètre targetY est suivi par un autre paramètre
            int endPosY = request.indexOf('&', posTargetY + 8);
            float targetY;
            if (endPosY != -1) {
              targetY = request.substring(posTargetY + 8, endPosY).toFloat();
            } else {
              // Si pas d'autre paramètre après targetY, prendre jusqu'à la fin
              targetY = request.substring(posTargetY + 8).toFloat();
            }
            
            addLog("[wifi] Coordonnées cibles: X=" + String(targetX) + ", Y=" + String(targetY));
            // Lancer la navigation vers le point
            naviguerVersPointAbsolu(targetX, targetY);
          } else {
            addLog("[wifi] Paramètre targetY manquant dans la requête");
          }
        }
        else if (posReset != -1) {
          addLog("[wifi] Détection paramètre reset=1 à la position " + String(posReset));
          if (isFormSubmit) {
            addLog("[wifi] Demande de réinitialisation confirmée");
            // Exécuter la fonction de réinitialisation
            resetRobot();
            
            // Vérifier que la réinitialisation a bien fonctionné
            addLog("[wifi] Vérification après réinitialisation: X=" + String(RobotX) + ", Y=" + String(RobotY) + ", Theta=" + String(RobotTheta));
          } else {
            addLog("[wifi] Paramètre reset=1 détecté mais formulaire non soumis");
          }
        } else if (posSequence != -1) {
          addLog("[wifi] Détection paramètre sequence=1 à la position " + String(posSequence));
          if (isFormSubmit) {
            addLog("[wifi] Demande de démarrage de la séquence automatique confirmée");
            // Démarrer la séquence si le robot n'est pas déjà en mouvement
            if (deplacementFait && !sequenceEnCours && !sequenceCercleEnCours) {
              sequenceEnCours = true;
              etapeSequence = 0;
              executerProchainMouvement = true;
              addLog("[wifi] Séquence automatique lancée - variables: sequenceEnCours=" + String(sequenceEnCours) + ", etapeSequence=" + String(etapeSequence));
              // Exécuter immédiatement la première étape
              executerSequenceAutomatique();
            } else {
              addLog("[wifi] Impossible de démarrer la séquence, robot occupé - deplacementFait=" + String(deplacementFait) + ", sequenceEnCours=" + String(sequenceEnCours));
            }
          } else {
            addLog("[wifi] Paramètre sequence=1 détecté mais formulaire non soumis");
          }
        } else if (posCercle != -1) {
          addLog("[wifi] Détection paramètre cercle=1 à la position " + String(posCercle));
          if (isFormSubmit) {
            addLog("[wifi] Demande de démarrage de la séquence cercle confirmée");
            // Démarrer la séquence cercle si le robot n'est pas déjà en mouvement
            if (deplacementFait && !sequenceCercleEnCours && !sequenceEnCours) {
              sequenceCercleEnCours = true;
              etapeCercle = 0;
              executerProchainPointCercle = true;
              addLog("[wifi] Séquence cercle lancée - variables: sequenceCercleEnCours=" + String(sequenceCercleEnCours) + ", etapeCercle=" + String(etapeCercle));
              // Exécuter immédiatement la première étape
              executerSequenceCercle();
            } else {
              addLog("[wifi] Impossible de démarrer la séquence cercle, robot occupé - deplacementFait=" + String(deplacementFait) + ", sequenceCercleEnCours=" + String(sequenceCercleEnCours));
            }
          } else {
            addLog("[wifi] Paramètre cercle=1 détecté mais formulaire non soumis");
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
      html += "<div style='background:#fff;padding:10px;border-radius:10px;margin-bottom:15px;'><strong>Position: </strong>";
      html += "X: " + String(RobotX, 1) + " cm, ";
      html += "Y: " + String(RobotY, 1) + " cm, ";
      html += "Angle: " + String(RobotTheta * 180.0 / PI, 1) + "° </div>";
      html += "<form action='/' method='get'>";
      html += "<h2>Commande</h2>";
      html += "<div class='input-group'>";
      html += "<label for='dx'>Delta X (cm):</label>";
      html += "<input type='number' step='0.1' name='dx' id='dx' value='" + String(deltaX_wifi) + "' required>";
      html += "</div>";
      html += "<div class='input-group'>";
      html += "<label for='dy'>Delta Y (cm):</label>";
      html += "<input type='number' step='0.1' name='dy' id='dy' value='" + String(deltaY_wifi) + "' required>";
      html += "</div>";
      html += "<div class='input-group'>";
      html += "<label for='type'>Type de coordonnées:</label>";
      html += "<select name='type' id='type'>";
      html += "<option value='absolu' selected>Absolues</option>";
      html += "<option value='robot'>Relatives au robot</option>";
      html += "</select>";
      html += "</div>";
      // Ajouter un champ caché pour indiquer une soumission explicite du formulaire
      html += "<input type='hidden' name='submit' value='1'>";
      html += "<input type='submit' value='Déplacer'>";
      html += "</form>";
      
      // Ajouter les boutons directionnels pour déplacement rapide de 0,1 cm
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Déplacement rapide</h2>";
      html += "<p>Déplacement absolu de 0,1 cm</p>";
      html += "<div style='display:grid; grid-template-columns:1fr 1fr 1fr; max-width:180px; margin:0 auto; gap:5px;'>";
      html += "<div></div>";
      html += "<a href='/?dx=0&dy=0.1&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>&#8593;</a>"; // Flèche vers le haut (Y+)
      html += "<div></div>";
      html += "<a href='/?dx=-0.1&dy=0&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>&#8592;</a>"; // Flèche vers la gauche (X-)
      html += "<div style='background:#ddd; color:#666; padding:5px; border-radius:5px;'>+0,1</div>"; // Centre
      html += "<a href='/?dx=0.1&dy=0&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>&#8594;</a>"; // Flèche vers la droite (X+)
      html += "<div></div>";
      html += "<a href='/?dx=0&dy=-0.1&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>&#8595;</a>"; // Flèche vers le bas (Y-)
      html += "<div></div>";
      html += "</div>"; // Fin de la grille
      html += "</div>"; // Fin du conteneur des boutons directionnels
      
      // Ajouter le bouton pour la séquence automatique (carré)
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Séquence Automatique</h2>";
      html += "<p>Dessiner un carré : 10 pas à droite, 10 pas en haut, 10 pas à droite</p>";
      html += "<a href='/?sequence=1&submit=1' style='background:#FF5722; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Lancer la séquence</a>";
      html += sequenceEnCours ? "<p><strong>Séquence en cours : Étape " + String(etapeSequence) + "/" + String(ETAPES_SEQUENCE_MAX) + "</strong></p>" : "";
      html += "</div>"; // Fin du conteneur pour la séquence automatique
      
      // Ajouter le bouton pour la séquence de cercle
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Dessiner un Cercle</h2>";
      html += "<p>Cercle de 1 cm de diamètre en 100 points</p>";
      html += "<a href='/?cercle=1&submit=1' style='background:#2196F3; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Dessiner le cercle</a>";
      html += sequenceCercleEnCours ? "<p><strong>Cercle en cours : Point " + String(etapeCercle) + "/" + String(ETAPES_CERCLE_MAX) + "</strong></p>" : "";
      html += "</div>"; // Fin du conteneur pour la séquence cercle
      
      // Ajouter le bouton de réinitialisation (reset)
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Réinitialisation</h2>";
      html += "<p>Réinitialiser la position du robot à (0,0) et l'angle à 0°</p>";
      html += "<a href='/?reset=1&submit=1' style='background:#F44336; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Réinitialiser position</a>";
      html += "<p><strong>Position actuelle: X=" + String(RobotX, 2) + " cm, Y=" + String(RobotY, 2) + " cm, Angle=" + String(RobotTheta * 180.0 / PI, 1) + "°</strong></p>";
      html += "</div>"; // Fin du conteneur pour la réinitialisation
      
      // Ajouter un formulaire pour la navigation vers un point absolu
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Navigation vers un point</h2>";
      html += "<p>Aller vers des coordonnées absolues sur la feuille</p>";
      html += "<form action='/' method='get'>";
      html += "<div style='display:flex; justify-content:center; align-items:center; gap:10px;'>";
      html += "<div>";
      html += "<label for='targetX'>X (cm):</label>";
      html += "<input type='number' step='0.1' name='targetX' id='targetX' value='0' style='width:80px;'>";
      html += "</div>";
      html += "<div>";
      html += "<label for='targetY'>Y (cm):</label>";
      html += "<input type='number' step='0.1' name='targetY' id='targetY' value='0' style='width:80px;'>";
      html += "</div>";
      html += "<input type='hidden' name='submit' value='1'>";
      html += "<input type='submit' value='Naviguer' style='margin-top:20px;'>";
      html += "</div>";
      html += "</form>";
      html += "</div>"; // Fin du conteneur pour la navigation vers un point
      
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
  }

  // Vérifier si un mouvement est en cours
  if (correctionActive && !deplacementFait) {
    bool fini = avancerCorrige();
    if (fini) {
      arreter();
      correctionActive = false;
      deplacementFait = true;
      
      // Mettre à jour la position du robot après que le mouvement est terminé
      RobotX += distance_en_cm_roue_droite * cos(RobotTheta) + distance_en_cm_roue_gauche * cos(RobotTheta);
      RobotY += distance_en_cm_roue_droite * sin(RobotTheta) + distance_en_cm_roue_gauche * sin(RobotTheta);
      
      // Mise à jour de l'angle du robot
      float deltaRoues = distance_en_cm_roue_gauche - distance_en_cm_roue_droite;
      float angleRelatif = atan(deltaRoues / LARGEUR_ROBOT);
      RobotTheta += angleRelatif;
      
      // Normaliser l'angle entre -PI et PI
      while (RobotTheta > PI) RobotTheta -= 2*PI;
      while (RobotTheta < -PI) RobotTheta += 2*PI;
      
      addLog("[position] Mouvement terminé - Robot arrêté");
      addLog("[position] abasolue X: " + String(RobotX, 2) + " cm | Y: " + String(RobotY, 2) + " cm | Orientation: " + String(RobotTheta * 180.0 / PI, 1) + "°");
      
      // Si nous sommes dans une séquence automatique, préparer l'étape suivante
      if (sequenceEnCours) {
        etapeSequence++;
        executerProchainMouvement = true;
        // Ajouter un court délai entre les mouvements
        delay(200);
      }
      // Si nous sommes dans une séquence de cercle, préparer le point suivant
      else if (sequenceCercleEnCours) {
        etapeCercle++;
        executerProchainPointCercle = true;
        // Ajouter un court délai entre les mouvements
        delay(200);
      }
    }
  }
  
  // Si une séquence automatique est en cours, continuer son exécution
  if (sequenceEnCours && deplacementFait) {
    executerSequenceAutomatique();
  }
  
  // Si une séquence de cercle est en cours, continuer son exécution
  if (sequenceCercleEnCours && deplacementFait) {
    executerSequenceCercle();
  }
}