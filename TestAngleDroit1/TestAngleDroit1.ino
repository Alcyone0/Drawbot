#include <Wire.h>
#include <WiFi.h>
#include <SparkFunLSM6DS3.h>
#include <math.h>
#include "RobotStructures.h"
#include "Sequences.h"

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
const int   PWM_MIN = 70;
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



/* ===== PROTOTYPES DE FONCTIONS ===== */
void demarer(float deltaX, float deltaY); // Déclaration anticipée
void executerSequenceAutomatique(); // Déclaration pour la séquence automatique


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
WheelDistances calculerDistancesRoues(float deltaX, float deltaY) {
  WheelDistances distances;
  distances.left = deltaX - (LARGEUR_ROBOT / LONGUEUR_ROBOT) * deltaY;
  distances.right = deltaX + (LARGEUR_ROBOT / LONGUEUR_ROBOT) * deltaY;  

  addLog("[calculerDistancesRoues] Distances roues: G=" + String(distances.left, 1) + " cm, D=" + String(distances.right, 1) + " cm");

  return distances;
}

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
  float angle = robotState.theta + angleRelatif;
  
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

// Fonction pour convertir les coordonnées absolues en coordonnées relatives au robot
// Utilise une transformation en coordonnées cylindriques/polaires
DeltaXY convertAbsoluteToRobotCoordinates(DeltaXY targetPoint, RobotState robotState) {

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
  //robotState = calculerNouvellePositionReelle(countLeft, countRight);
  robotState = calculerNouvellePositionTheorique(distance_en_cm_roue_gauche, distance_en_cm_roue_droite);

  // Réinitialiser les compteurs
  countLeft = 0;
  countRight = 0;
  
  // Activer le mouvement
  correctionActive = true;
  deplacementFait = false;
  addLog("[demarer] Début du mouvement");
} 

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
  
  addLog("[reset] Position et orientation réinitialisées à zéro");
  addLog("[reset] X=0.0, Y=0.0, Theta=0.0°");
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
      int posTargetX = request.indexOf("target_x=");
      int posTargetY = request.indexOf("target_y=");
      
      // Vérifier si c'est une vraie requête de formulaire avec des paramètres
      bool isFormSubmit = false;
      
      // Nouveau: vérifier le paramètre de soumission explicite
      int posSubmit = request.indexOf("submit=1");
      
      // Vérifier si la requête contient "GET /?" et le paramètre de soumission
      if (request.indexOf("GET /?reset=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de réinitialisation recu");
      } else if (request.indexOf("GET /?sequence=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de séquence carré recu");

      } else if (request.indexOf("GET /?dx=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de mouvement recu");
      } else if (request.indexOf("GET /?target_x=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("[wifi] Formulaire de test de coordonnées recu");
      } else if (request.indexOf("GET /?dx=") != -1 || request.indexOf("GET /?sequence=") != -1 ||  request.indexOf("GET /?reset=") != -1 || request.indexOf("GET /?target_x=") != -1) {
        // C'est un rechargement de page avec les paramètres dans l'URL
        addLog("[wifi] Recharge de page détectée, mouvement ignoré");
      }
      
      // Traiter le formulaire de test de coordonnées absolues
      if (posTargetX != -1 && posTargetY != -1 && isFormSubmit) {
        float targetX = request.substring(posTargetX + 9, request.indexOf('&', posTargetX)).toFloat();
        float targetY = request.substring(posTargetY + 9).toFloat();
        addLog("[wifi] Test de coordonnées: X=" + String(targetX) + ", Y=" + String(targetY));
        
        // Calculer les deltas pour aller de la position actuelle à la position cible
        float deltaX = targetX - robotState.x;
        float deltaY = targetY - robotState.y;
        addLog("[wifi] Deltas calculés: dX=" + String(deltaX) + ", dY=" + String(deltaY));
        
        // Convertir en coordonnées relatives au robot
        DeltaXY absolutePoint(deltaX, deltaY);
        DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(absolutePoint, robotState);
        
        // Démarrer le mouvement
        demarer(robotCoord.x, robotCoord.y);
      }
      // Traiter le formulaire de mouvement standard
      else if (posX != -1 && posY != -1 && isFormSubmit) {
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
          DeltaXY absolutePoint(dx, dy);
          DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(absolutePoint, robotState);
          // Démarrer le mouvement avec les coordonnées relatives
          demarer(robotCoord.x, robotCoord.y);
        } else {
          // Utiliser directement les coordonnées relatives au robot (type=robot ou type=direct)
          addLog("[wifi] Utilisation directe des coordonnées: " + coordType + " (" + String(dx, 2) + ", " + String(dy, 2) + ")");
          demarer(dx, dy);
        }
      } else {
        // Vérifier si c'est une demande pour lancer la séquence automatique (carré) ou réinitialiser
        int posSequence = request.indexOf("sequence=1");
  
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
        } else if (posSequence != -1) {
          addLog("[wifi] Détection paramètre sequence=1 à la position " + String(posSequence));
          if (isFormSubmit) {
            addLog("[wifi] Demande de démarrage de la séquence automatique confirmée");
            // Démarrer la séquence si le robot n'est pas déjà en mouvement
            if (deplacementFait && !sequenceEnCours) {
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
      html += "X: " + String(robotState.x, 1) + " cm, ";
      html += "Y: " + String(robotState.y, 1) + " cm, ";
      html += "Angle: " + String(robotState.theta * 180.0 / PI, 1) + "° </div>";
      html += "<form action='/' method='get'>";
      html += "<h2>Commande</h2>";
      html += "<div class='input-group'>";
      html += "<label for='dx'>Delta X (cm):</label>";
      html += "<input type='number' step='0.1' name='dx' id='dx' value='0' required>";
      html += "</div>";
      html += "<div class='input-group'>";
      html += "<label for='dy'>Delta Y (cm):</label>";
      html += "<input type='number' step='0.1' name='dy' id='dy' value='0' required>";
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
      html += "<h2>Déplacement rapide (Absolu)</h2>";
      html += "<p>Déplacement absolu de 0,1 cm</p>";
      html += "<div style='display:grid; grid-template-columns:1fr 1fr 1fr; max-width:180px; margin:0 auto; gap:5px;'>";
      html += "<div></div>";
      html += "<a href='/?dx=0&dy=0.1&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>+y</a>"; // Flèche vers le haut (Y+)
      html += "<div></div>";
      html += "<a href='/?dx=-0.1&dy=0&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>-x</a>"; // Flèche vers la gauche (X-)
      html += "<div style='background:#ddd; color:#666; padding:5px; border-radius:5px;'>+0,1</div>"; // Centre
      html += "<a href='/?dx=0.1&dy=0&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>+x</a>"; // Flèche vers la droite (X+)
      html += "<div></div>";
      html += "<a href='/?dx=0&dy=-0.1&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>-y</a>"; // Flèche vers le bas (Y-)
      html += "<div></div>";
      html += "</div>"; // Fin de la grille
      html += "</div>"; // Fin du conteneur des boutons directionnels absolus
      
      // Ajouter les boutons directionnels pour déplacement relatif de 0,1 cm
      html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      html += "<h2>Déplacement rapide (Relatif)</h2>";
      html += "<p>Déplacement relatif au robot de 0,1 cm</p>";
      html += "<div style='display:grid; grid-template-columns:1fr 1fr 1fr; max-width:200px; margin:0 auto; gap:5px;'>";
      html += "<div></div>";
      html += "<a href='/?dx=0.1&dy=0&type=direct&submit=1' style='background:#2196F3; color:white; padding:10px; border-radius:5px; text-decoration:none; text-align:center;'>AV</a>";
      html += "<div></div>";
      html += "<a href='/?dx=0&dy=0.1&type=direct&submit=1' style='background:#2196F3; color:white; padding:10px; border-radius:5px; text-decoration:none; text-align:center;'>G</a>";
      html += "<div style='background:#ddd; color:#666; padding:5px; border-radius:5px; text-align:center;'>+0,1</div>";
      html += "<a href='/?dx=0&dy=-0.1&type=direct&submit=1' style='background:#2196F3; color:white; padding:10px; border-radius:5px; text-decoration:none; text-align:center;'>D</a>";
      html += "<div></div>";
      html += "<a href='/?dx=-0.1&dy=0&type=direct&submit=1' style='background:#2196F3; color:white; padding:10px; border-radius:5px; text-decoration:none; text-align:center;'>AR</a>";
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
      
      // Ajouter le bouton pour la séquence automatique (carré)
      // html += "<div style='margin-top:20px; margin-bottom:20px;'>";
      // html += "<h2>Séquence Automatique</h2>";
      // html += "<p>Dessiner un carré : 10 pas à droite, 10 pas en haut, 10 pas à droite</p>";
      // html += "<a href='/?sequence=1&submit=1' style='background:#FF5722; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Lancer la séquence</a>";
      // html += sequenceEnCours ? "<p><strong>Séquence en cours : Étape " + String(etapeSequence) + "/" + String(ETAPES_SEQUENCE_MAX) + "</strong></p>" : "";
      // html += "</div>"; // Fin du conteneur pour la séquence automatique
      
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
      html += "<p><strong>Position actuelle: X=" + String(robotState.x, 2) + " cm, Y=" + String(robotState.y, 2) + " cm, Angle=" + String(robotState.theta * 180.0 / PI, 1) + "°</strong></p>";
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
  
  // Si une séquence automatique est en cours, continuer son exécution
  if (sequenceEnCours && deplacementFait) {
    executerSequenceAutomatique();
  }
  

}