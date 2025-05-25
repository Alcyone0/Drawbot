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
const float DIST_STYLO_CM = 6.5;
const float LARGEUR_ROBOT = 8.0;

/* ===== VARIABLES INTERNES ===== */
const int basePWM_D = 80;
long seuilImpulsionsRoueGauche = 0;
long seuilImpulsionsRoueDroite = 0;
float distance_en_cm_roue_gauche = 0;
float distance_en_cm_roue_droite = 0;
float deltaX_wifi = 0;
float deltaY_wifi = 0;

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

float calculerTheta(float deltaX, float deltaY) {
  // Protection contre division par zéro
  if (deltaY == 0) {
    return (deltaX >= 0) ? PI/2 : -PI/2;
  }
  return atan2(deltaX, deltaY);
}

float calculerDeltaRoueGaucheY(float deltaStyloX, float deltaStyloY) {
  float theta = calculerTheta(deltaStyloX, deltaStyloY);
  float deltaCentreY = deltaStyloY - DIST_STYLO_CM * cos(theta);
  return deltaCentreY + (LARGEUR_ROBOT / 2.0) * sin(theta);
}

float calculerDeltaRoueDroiteY(float deltaStyloX, float deltaStyloY) {
  float theta = calculerTheta(deltaStyloX, deltaStyloY);
  float deltaCentreY = deltaStyloY - DIST_STYLO_CM * cos(theta);
  return deltaCentreY - (LARGEUR_ROBOT / 2.0) * sin(theta);
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

void demarer(float deltaX, float deltaY)
{
  distance_en_cm_roue_gauche = calculerDeltaRoueGaucheY(deltaX, deltaY);
  distance_en_cm_roue_droite = calculerDeltaRoueDroiteY(deltaX, deltaY);
  
  // Enregistrer les valeurs dans les logs
  String logMsg = "DX=" + String(deltaX) + 
                  ", DY=" + String(deltaY) + 
                  ", Distance Gauche=" + String(distance_en_cm_roue_gauche) + 
                  ", Distance Droite=" + String(distance_en_cm_roue_droite);
  addLog(logMsg);
  
  seuilImpulsionsRoueGauche = distance_en_cm_roue_gauche * IMPULSIONS_PAR_CM;
  seuilImpulsionsRoueDroite = distance_en_cm_roue_droite * IMPULSIONS_PAR_CM;
  countLeft = 0;
  countRight = 0;
  correctionActive = true;
  deplacementFait = false;
} 

bool avancerCorrige() {
  digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);

  float ratioG = (float)countLeft  / seuilImpulsionsRoueGauche;
  float ratioD = (float)countRight / seuilImpulsionsRoueDroite;

  float erreurRatio = ratioD - ratioG;
  float Kp = 60.0;
  int correction = Kp * erreurRatio;

  float ratioMoyen = (ratioG + ratioD) / 2.0;
  float resteRatio = 1.0 - ratioMoyen;
  int pwmBase = constrain(basePWM_D + resteRatio * 30, basePWM_D, 110);

  int pwmD = constrain(pwmBase + correction, 70, 255);
  int pwmG = constrain(pwmBase - correction, 70, 255);

  analogWrite(EN_D, pwmD);
  analogWrite(EN_G, pwmG);
  
  // Ajouter un log périodique (tous les 500ms) pour ne pas saturer
  static unsigned long lastLogTime = 0;
  if (millis() - lastLogTime > 500) {
    addLog("Progression: G=" + String(countLeft) + "/" + String(seuilImpulsionsRoueGauche) + 
           " D=" + String(countRight) + "/" + String(seuilImpulsionsRoueDroite) + 
           " (" + String(int(ratioMoyen*100)) + "%)");
    lastLogTime = millis();
  }

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
  // Assurer que le robot ne bouge pas au démarrage
  if (!deplacementFait && !correctionActive) {
    arreter();
  }
  
  WiFiClient client = server.available();
  if (client) {
    while (!client.available()) delay(1);
    String request = client.readStringUntil('\r');
    client.flush();

    int posX = request.indexOf("dx=");
    int posY = request.indexOf("dy=");
    if (posX != -1 && posY != -1) {
      float dx = request.substring(posX + 3, request.indexOf('&', posX)).toFloat();
      float dy = request.substring(posY + 3).toFloat();
      deltaX_wifi = dx;
      deltaY_wifi = dy;
      demarer(dx, dy);
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
    // Pas de rafraîchissement automatique
    html += "</head><body>";
    html += "<h1>Drawbot WiFi</h1>";
    html += "<form method='GET'>";
    html += "<h2>Commande</h2>";
    html += "deltaX (cm): <input type='number' name='dx' step='0.1' value='" + String(deltaX_wifi) + "'><br>";
    html += "deltaY (cm): <input type='number' name='dy' step='0.1' value='" + String(deltaY_wifi) + "'><br>";
    html += "<input type='submit' value='Envoyer'></form>";
    
    // Affichage des logs
    html += "<h2>Logs</h2>";
    html += "<div class='log-container'>";
    html += getAllLogs();
    html += "</div>";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println();
    client.println(html);
    client.stop();
  }

  if (correctionActive && !deplacementFait) {
    bool fini = avancerCorrige();
    if (fini) {
      arreter();
      correctionActive = false;
      deplacementFait = true;
    }
  }
}
