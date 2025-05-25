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
bool deplacementFait = false;

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

void demarer(float deltaX, float deltaY)
{
  distance_en_cm_roue_gauche = calculerDeltaRoueGaucheY(deltaX, deltaY);
  distance_en_cm_roue_droite = calculerDeltaRoueDroiteY(deltaX, deltaY);
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

  return (countLeft >= seuilImpulsionsRoueGauche) && (countRight >= seuilImpulsionsRoueDroite);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

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
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
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
    html += "<style>body{font-family:Arial;text-align:center;background:#f2f2f2;padding:40px;}";
    html += "form{background:#fff;padding:25px;border-radius:12px;box-shadow:0 0 10px #888;display:inline-block;}";
    html += "input{padding:10px;margin:10px;border-radius:5px;}";
    html += "input[type=submit]{background:#4CAF50;color:white;border:none;}";
    html += "input[type=submit]:hover{background:#45a049;}</style></head><body>";
    html += "<h1>Commande deltaX / deltaY</h1>";
    html += "<form method='GET'>";
    html += "deltaX (cm): <input type='number' name='dx' step='0.1'><br>";
    html += "deltaY (cm): <input type='number' name='dy' step='0.1'><br>";
    html += "<input type='submit' value='Envoyer'></form></body></html>";

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
