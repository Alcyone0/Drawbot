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
const int   PWM_ROTATION = 80;
const int   PWM_MARCHE   = 80;
const float DIST_STYLO_CM = 13.0;
const float LARGEUR_ROBOT = 8.5;
const float LONGUEUR_ROBOT = 13.0; // Distance entre l'axe des roues et le stylo

/* ===== VARIABLES INTERNES ===== */
const int basePWM_D = 80;
long seuilImpulsionsRoueGauche = 0;
long seuilImpulsionsRoueDroite = 0;
float distance_en_cm_roue_gauche = 0;
float distance_en_cm_roue_droite = 0;
float deltaX_wifi = 0;
float deltaY_wifi = 0;

/* ===== STRUCTURES ===== */
struct Point {
  float x;
  float y;
  
  // Constructeur par défaut
  Point() : x(0), y(0) {}
  
  // Constructeur avec paramètres
  Point(float _x, float _y) : x(_x), y(_y) {}
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
const int MAX_LOGS = 20;
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

// Calcule les distances des deux roues en fonction du déplacement demandé
WheelDistances calculerDistancesRoues(float deltaRobotX, float deltaRobotY) {
  WheelDistances distances;
  distances.left = deltaRobotY + (LARGEUR_ROBOT / LONGUEUR_ROBOT) * deltaRobotX;
  distances.right = deltaRobotY - (LARGEUR_ROBOT / LONGUEUR_ROBOT) * deltaRobotX;  
  return distances;
}

// Fonction pour convertir les coordonnées absolues en coordonnées relatives au robot
Point convertAbsoluteToRobotCoordinates(float deltaAbsoluteX, float deltaAbsoluteY) {
  // Calculer les coordonnées cibles absolues
  float targetAbsoluteX = RobotX + deltaAbsoluteX;
  float targetAbsoluteY = RobotY + deltaAbsoluteY;
  
  // Calculer le vecteur entre la position actuelle et la cible
  float vectorX = targetAbsoluteX - RobotX;
  float vectorY = targetAbsoluteY - RobotY;
  
  // Appliquer une rotation inverse à l'angle actuel du robot
  // Cette formule transforme les coordonnées absolues en coordonnées relatives au repère du robot
  float deltaRobotX = vectorX * cos(-RobotTheta) + vectorY * sin(-RobotTheta);
  float deltaRobotY = -vectorX * sin(-RobotTheta) + vectorY * cos(-RobotTheta);
  
  // Afficher les valeurs pour débogage
  addLog("Conversion: Absolue (" + String(deltaAbsoluteX, 2) + "," + String(deltaAbsoluteY, 2) + ") -> Robot (" + 
         String(deltaRobotX, 2) + "," + String(deltaRobotY, 2) + ")");
         
  // Retourner un Point contenant les coordonnées relatives
  return Point(deltaRobotX, deltaRobotY);
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
  
  // Parcourir les logs à partir du plus récent
  for (int i = 0; i < MAX_LOGS; i++) {
    int idx = (logIndex - 1 - i + MAX_LOGS) % MAX_LOGS;
    if (logs[idx].length() > 0) {
      allLogs += logs[idx] + "<br>";
      count++;
    }
  }
  
  return allLogs.length() > 0 ? allLogs : "Aucun log disponible";
}

// Variables globales pour stocker la direction des roues
bool directionAvantGauche = true; // true = avant, false = arrière
bool directionAvantDroite = true; // true = avant, false = arrière

// Fonction pour mettre à jour la position du robot après un déplacement
void updateRobotPosition(float deltaX, float deltaY) {
  // Si le déplacement est seulement en X (rotation)
  if (abs(deltaY) < 0.05 && abs(deltaX) > 0.05) {
    // Estimer le changement d'angle (en radians)
    float deltaTheta = atan2(deltaX, DIST_STYLO_CM);
    RobotTheta += deltaTheta;
    // Normaliser l'angle entre -PI et PI
    while (RobotTheta > PI) RobotTheta -= 2*PI;
    while (RobotTheta < -PI) RobotTheta += 2*PI;
    return;
  }
  
  // Si le déplacement est seulement en Y (ligne droite)
  if (abs(deltaX) < 0.05 && abs(deltaY) > 0.05) {
    RobotX += deltaY * sin(RobotTheta);
    RobotY += deltaY * cos(RobotTheta);
    return;
  }
  
  // Pour un déplacement combiné
  float deltaTheta = atan2(deltaX, deltaY);
  RobotTheta += deltaTheta;
  // Normaliser l'angle entre -PI et PI
  while (RobotTheta > PI) RobotTheta -= 2*PI;
  while (RobotTheta < -PI) RobotTheta += 2*PI;
  
  // Calculer la distance parcourue
  float distance = sqrt(deltaX*deltaX + deltaY*deltaY);
  RobotX += distance * sin(RobotTheta);
  RobotY += distance * cos(RobotTheta);
}

void demarer(float deltaX, float deltaY)
{
  // Vérifier si les valeurs sont zéro
  if (abs(deltaX) < 0.01 && abs(deltaY) < 0.01) {
    addLog("Valeurs trop petites, pas de mouvement");
    return;
  }

  // Calculer les distances pour chaque roue avec la nouvelle fonction
  WheelDistances distances = calculerDistancesRoues(deltaX, deltaY);
  distance_en_cm_roue_gauche = distances.left;
  distance_en_cm_roue_droite = distances.right;
  
  // Vérifier que les distances ne sont pas aberrantes
  if (abs(distance_en_cm_roue_gauche) > 50 || abs(distance_en_cm_roue_droite) > 50) {
    addLog("ATTENTION: Distances calculées anormalement grandes, limitation à 20cm");
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
  String logMsg = "Commande reçue: DX=" + String(deltaX, 2) + 
                  ", DY=" + String(deltaY, 2) + 
                  ", Distance Gauche=" + String(distance_en_cm_roue_gauche, 2) + " cm" +
                  ", Distance Droite=" + String(distance_en_cm_roue_droite, 2) + " cm" + 
                  ", Seuil G=" + String(seuilImpulsionsRoueGauche) + 
                  ", Seuil D=" + String(seuilImpulsionsRoueDroite);
  addLog(logMsg);
  
  // Réinitialiser les compteurs
  countLeft = 0;
  countRight = 0;
  
  // Activer le mouvement
  correctionActive = true;
  deplacementFait = false;
  addLog("Début du mouvement");
} 


/**void avancerCorrige() {
  digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
  int erreur = countRight - countLeft;
  float Kp = 4.5;
  int correction = Kp * erreur;
  long moyenne = (countLeft + countRight) / 2;
  long reste = seuilImpulsions - moyenne;
  int pwmBase = constrain(map(reste, 0, seuilImpulsions, basePWM_D, 110), basePWM_D, 110);
  int pwmD = constrain(pwmBase + correction, 70, 255);
  int pwmG = constrain(pwmBase - correction, 70, 255);
  analogWrite(EN_D, pwmD);
  analogWrite(EN_G, pwmG);
}**/


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

  int pwmD = constrain(80 + correctionDroite, 80, 110);
  int pwmG = constrain(80 + correctionGauche, 80, 110);

  analogWrite(EN_D, pwmD);
  analogWrite(EN_G, pwmG);

  bool fini = (countLeft >= seuilImpulsionsRoueGauche) && (countRight >= seuilImpulsionsRoueDroite);
  if (fini) {
    addLog("Déplacement terminé");
  }
  return fini;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  addLog("Drawbot démarré");
  addLog("Position initiale: X=0.0, Y=0.0, Theta=0.0°");

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

  WiFi.softAP(ssid, password);
  server.begin();
  String ipAddress = WiFi.softAPIP().toString();
  addLog("WiFi AP démarré - IP: " + ipAddress);
}

void loop() {
  // Assurer que le robot ne bouge pas au démarrage ou après un déplacement
  if (deplacementFait && !correctionActive) {
    arreter(); // S'assurer que le robot est bien arrêté
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
      
      // Vérifier si la requête contient "GET /?dx=" et le paramètre de soumission
      if (request.indexOf("GET /?dx=") != -1 && posSubmit != -1) {
        isFormSubmit = true;
        addLog("Formulaire soumis");
      } else if (request.indexOf("GET /?dx=") != -1) {
        // C'est un rechargement de page avec les paramètres dans l'URL
        addLog("Recharge de page détectée, mouvement ignoré");
      }
      
      if (posX != -1 && posY != -1 && isFormSubmit) {
        // Déterminer le type de coordonnées (absolu ou robot)
        bool isAbsoluteCoords = true; // Par défaut, on considère les coordonnées comme absolues
        
        if (posType != -1) {
          String typeValue = request.substring(posType + 5, request.indexOf('&', posType + 5));
          if (typeValue == "robot") {
            isAbsoluteCoords = false;
            addLog("Type de coordonnées: relatives au robot");
          } else {
            addLog("Type de coordonnées: absolues");
          }
        } else {
          addLog("Type non spécifié, utilisation des coordonnées absolues par défaut");
        }
        float dx = request.substring(posX + 3, request.indexOf('&', posX)).toFloat();
        float dy = request.substring(posY + 3).toFloat();
        addLog("Valeurs reçues: dx=" + String(dx) + ", dy=" + String(dy));
        
        // Mettre à jour les valeurs stockées
        deltaX_wifi = dx;
        deltaY_wifi = dy;
        
        if (isAbsoluteCoords) {
          // Convertir les coordonnées absolues en coordonnées relatives au robot
          Point robotCoord = convertAbsoluteToRobotCoordinates(dx, dy);
          addLog("Conversion de coordonnées absolues vers robot");
          
          // Démarrer le mouvement avec les coordonnées relatives
          demarer(robotCoord.x, robotCoord.y);
        } else {
          // Utiliser directement les coordonnées relatives au robot
          addLog("Utilisation directe des coordonnées relatives au robot");
          demarer(dx, dy);
        }
      } else {
        // Si c'est juste un chargement de page sans soumission
        addLog("Page chargée sans commande");
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
      
      // Mettre à jour la position du robot après le déplacement
      updateRobotPosition(deltaX_wifi, deltaY_wifi);
      
      addLog("Mouvement terminé - Robot arrêté");
      addLog("Position actuelle: X=" + String(RobotX, 1) + ", Y=" + String(RobotY, 1) + ", Theta=" + String(RobotTheta * 180.0 / PI, 1) + "°");
    }
  }
}