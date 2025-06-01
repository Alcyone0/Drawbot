#include <WiFi.h>
#include <Wire.h>
#include <SparkFunLSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6B);
float biaisGyroZ = 0.0;

const char* ssid = "Drawbot_WIFI";
const char* password = "12345678";
WiFiServer server(80);

const int encoderLeftA = 27;
const int encoderRightA = 33;

#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16
#define EN_D   23
#define EN_G   4

#define MAG_ADDR 0x1E
const float offsetX = 1973.5;
const float offsetY = -2641.5;
const float TOLERANCE = 2.0;
bool orienterVersNord = false;
bool nordAtteint = true;
bool avancerApresNord = false;

volatile long countLeft = 0;
volatile long countRight = 0;
const float IMPULSIONS_PAR_CM = 34.0;
float distanceCM = 0.0;
long seuilImpulsions = 0;
const int basePWM_G = 70;
const int basePWM_D = 70;
const int PWM_ROTATION = 100;
const int PWM_ESCALIER = 80;
bool deplacementFait = false;
bool correctionActive = false;
bool sequenceActive = false;

void countLeftEncoder() { countLeft++; }
void countRightEncoder() { countRight++; }

// ---------------------------------------------------ARRETER---------------------------------------------------

void arreter() {
  analogWrite(EN_D, 0); analogWrite(EN_G, 0);
  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, LOW);
}

// ---------------------------------------------------AVANCER PRECISEMENT---------------------------------------------------

void avancerPrecisement(float cm) {
  long seuil = cm * IMPULSIONS_PAR_CM;
  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < seuil) {
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
    analogWrite(EN_D, PWM_ESCALIER);
    analogWrite(EN_G, PWM_ESCALIER);
  }
  arreter();
  delay(150);
}

// ---------------------------------------------------RECULER PRECISEMENT---------------------------------------------------

void reculerPrecisement(float cm) {
  long seuil = cm * IMPULSIONS_PAR_CM;
  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < seuil) {
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
    analogWrite(EN_D, PWM_ESCALIER);
    analogWrite(EN_G, PWM_ESCALIER);
  }
  arreter();
  delay(150);
}

// ---------------------------------------------------ROTATION---------------------------------------------------

void rotation(float angleCible) {
  float angleActuel = 0.0;
  unsigned long t0 = millis();
  if (angleCible > 0) {
    digitalWrite(IN_1_D, LOW);  digitalWrite(IN_2_D, HIGH);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
  } else {
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
  }
  analogWrite(EN_D, PWM_ROTATION);
  analogWrite(EN_G, PWM_ROTATION);
  while (abs(angleActuel) < abs(angleCible)) {
    unsigned long t1 = millis();
    float dt = (t1 - t0) / 1000.0;
    t0 = t1;
    float vitesseZ = imu.readFloatGyroZ() - biaisGyroZ;
    angleActuel += vitesseZ * dt;
  }
  arreter();
  delay(150);
}

// ---------------------------------------------------LIRE ANGLE NORD---------------------------------------------------

float lireAngleNord() {
  int16_t mx = 0, my = 0;
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x28 | 0x80);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 6);
  if (Wire.available() == 6) {
    mx = Wire.read() | (Wire.read() << 8);
    my = Wire.read() | (Wire.read() << 8);
    Wire.read(); Wire.read();
  }
  float mxCorr = mx - offsetX;
  float myCorr = my - offsetY;
  float angle = atan2(mxCorr, -myCorr) * 180.0 / PI;
  if (angle < 0) angle += 360.0;
  return angle;
}

// ---------------------------------------------------CALCULER ERREUR ANGLE---------------------------------------------------

float calculerErreurAngle(float angle, float cible = 0.0) {
  float erreur = angle - cible;
  if (erreur > 180.0) erreur -= 360.0;
  if (erreur < -180.0) erreur += 360.0;
  return erreur;
}

// ---------------------------------------------------CALCULER PWM---------------------------------------------------

int calculerPWM(float erreur) {
  float absErr = abs(erreur);
  if (absErr <= 5.0) return 80;
  else if (absErr <= 30.0) return 100;
  else return 150;
}

// ---------------------------------------------------TOURNER GAUCHE---------------------------------------------------

void tournerGauche(int pwm) {
  digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
  analogWrite(EN_D, pwm);
  analogWrite(EN_G, pwm);
}

// ---------------------------------------------------TOURNER DROITE---------------------------------------------------

void tournerDroite(int pwm) {
  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
  digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, HIGH);
  analogWrite(EN_D, pwm);
  analogWrite(EN_G, pwm);
}

void gauche(int pwm, int duree) {
  tournerGauche(pwm);
  delay(duree);
  arreter();
}

void droite(int pwm, int duree) {
  tournerDroite(pwm);
  delay(duree);
  arreter();
}

// ---------------------------------------------------AVANCER CORRIGE---------------------------------------------------

void avancerCorrige() {
  digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);

  float erreur_encoder = countRight - countLeft;
  float correction_pid = 4.5 * erreur_encoder;
  
  long moyenne = (countLeft + countRight) / 2;
  long reste = seuilImpulsions - moyenne;
  
  int pwmBase = basePWM_D;
  int pwmD = constrain(pwmBase + correction_pid, 70, 255);
  int pwmG = constrain(pwmBase - correction_pid, 70, 255);
  
  analogWrite(EN_D, pwmD);
  analogWrite(EN_G, pwmG);
  
  if (countLeft % 100 == 0 || countRight % 100 == 0) {
    Serial.print("[avancerCorrige] Ticks D/G: ");
    Serial.print(countRight); Serial.print("/");
    Serial.print(countLeft); Serial.print(" PWM D/G: ");
    Serial.print(pwmD); Serial.print("/");
    Serial.println(pwmG);
  }
}

// ---------------------------------------------------CALIBRER GYRO---------------------------------------------------

void calibrerGyro() {
  float somme = 0.0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    somme += imu.readFloatGyroZ();
    delay(5);
  }
  biaisGyroZ = somme / N;
}

// ----------------------------------------------Escalier 1-----------------------------------------------

void Escalier1() {
  const int pwm_d_base = 80;
  const int pwm_g_base = 80;
  const int seuil_ticks = 20*IMPULSIONS_PAR_CM;

  const float Kp = 10;
  const float Ki = 0;
  const float Kd = 0;

  float erreur = 0;
  float erreur_precedente = 0;
  float somme_erreurs = 0;
  float correction = 0;
    
  while ((countLeft+ countRight)/2 < seuil_ticks) {
    erreur = (float)countRight - (float)countLeft;
    somme_erreurs += erreur;
    correction = Kp * erreur + Ki * somme_erreurs + Kd * (erreur - erreur_precedente);
    erreur_precedente = erreur;

    int pwm_d = pwm_d_base + correction;
    int pwm_g = pwm_g_base - correction;

    pwm_d = constrain(pwm_d, 0, 255);
    pwm_g = constrain(pwm_g, 0, 255);
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
    digitalWrite(IN_1_G, HIGH);  digitalWrite(IN_2_G, LOW);
    analogWrite(EN_D, pwm_d);
    analogWrite(EN_G, pwm_g);
  }

  arreter();
}

// ----------------------------------------------Escalier 2-----------------------------------------------

void Escalier2() {
  const int pwm_d_base = 190;
  const int pwm_g_base = 10;
  const int seuil_ticks = 80;

  const float Kp = 0;
  const float Ki = 0;
  const float Kd = 0;

  float erreur = 0;
  float erreur_precedente = 0;
  float somme_erreurs = 0;
  float correction = 0;
    
  while (countLeft < seuil_ticks) {
    erreur = (float)countRight - (float)countLeft;
    somme_erreurs += erreur;
    correction = Kp * erreur + Ki * somme_erreurs + Kd * (erreur - erreur_precedente);
    erreur_precedente = erreur;

    int pwm_d = pwm_d_base + correction;
    int pwm_g = pwm_g_base - correction;

    pwm_d = constrain(pwm_d, 0, 255);
    pwm_g = constrain(pwm_g, 0, 255);
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
    digitalWrite(IN_1_G, HIGH);  digitalWrite(IN_2_G, LOW);
    analogWrite(EN_D, pwm_d);
    analogWrite(EN_G, pwm_g);

    delay(20);
  }
  arreter();
}

// ----------------------------------------------Escalier 3-----------------------------------------------

void Escalier3() {
  const int pwm_d_base = 60;
  const int pwm_g_base = 110;
  const int seuil_ticks = 1000;

  float Kp = 0;
  float Ki = 0;
  float Kd = 0;

  float erreur = 0;
  float erreur_precedente = 0;
  float somme_erreurs = 0;
  float correction = 0;
    
  while (countRight < seuil_ticks) {
    erreur = (float)countRight - (float)countLeft;
    somme_erreurs += erreur;
    correction = Kp * erreur + Ki * somme_erreurs + Kd * (erreur - erreur_precedente);
    erreur_precedente = erreur;

    int pwm_d = pwm_d_base + correction;
    int pwm_g = pwm_g_base - correction;

    pwm_d = constrain(pwm_d, 0, 255);
    pwm_g = constrain(pwm_g, 0, 255);
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
    digitalWrite(IN_1_G, HIGH);  digitalWrite(IN_2_G, LOW);
    analogWrite(EN_D, pwm_d);
    analogWrite(EN_G, pwm_g);

    delay(20);
  }
  arreter();
}

// ----------------------------------------------Sequence Escalier-----------------------------------------------

void sequenceEscalier() {
    deplacementFait = true;
    correctionActive = false;
    nordAtteint = true;
    orienterVersNord = false;

    Escalier1();
    delay(300);
    Escalier2();
    delay(300);
    Escalier3();
    delay(300);
}

// ----------------------------------------------Fonction dessiner fleche nord-----------------------------------------------

void dessinerFlecheNord() {
  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < 100) {
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
    analogWrite(EN_D, 90);
    analogWrite(EN_G, 60);
  }
  arreter();
  delay(200);

  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < 100) {
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
    analogWrite(EN_D, 90);
    analogWrite(EN_G, 60);
  }
  arreter();
  delay(200);

  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < 100) {
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
    analogWrite(EN_D, 60);
    analogWrite(EN_G, 90);
  }
  arreter();
  delay(200);

  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < 100) {
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
    analogWrite(EN_D, 60);
    analogWrite(EN_G, 90);
  }
  arreter();
}

// ----------------------------------------------Sequence Pomme-----------------------------------------------

void sequencePomme() {
  rotation(360.0);
  dessinerFlecheNord();
  delay(300);
}

// ------------------------------------------------------SETUP---------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  if (imu.begin() != 0) {
    Serial.println("IMU non detectee");
    while (1);
  }
  calibrerGyro();
  Wire.begin();
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x20); Wire.write(0b01100000); Wire.endTransmission();
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x21); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x22); Wire.write(0x00); Wire.endTransmission();
  pinMode(IN_1_D, OUTPUT); pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT); pinMode(IN_2_G, OUTPUT);
  pinMode(EN_D, OUTPUT);   pinMode(EN_G, OUTPUT);
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderRightA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLeftA), countLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), countRightEncoder, RISING);
  WiFi.softAP(ssid, password);
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.softAPIP());
  server.begin();
}

void sendHtmlPage(WiFiClient client, String message = "");

// ------------------------------------------------------LOOP---------------------------------------------------

void loop() {
  WiFiClient client = server.available();
  if (client) {
    while (!client.available()) delay(1);
    String request = client.readStringUntil('\r');
    client.flush();
    int pos;
    
    pos = request.indexOf("angle90");
    if (pos != -1) {
      rotation(90.0);
      
      sendHtmlPage(client, "Rotation 90 effectuee");
      return;
    }
    
    pos = request.indexOf("angle360");
    if (pos != -1) {
      rotation(360.0);
      
      sendHtmlPage(client, "Rotation 360 effectuee");
      return;
    }
    
    pos = request.indexOf("escalier1");
    if (pos != -1) {
      sequenceActive = true;
      countLeft = 0; countRight = 0;
      Escalier1();
      delay(100);
      sequenceActive = false;
      
      sendHtmlPage(client, "Escalier 1 execute");
      return;
    }
    
    pos = request.indexOf("escalier2");
    if (pos != -1) {
      sequenceActive = true;
      countLeft = 0; countRight = 0;
      Escalier2();
      delay(100);
      sequenceActive = false;
      
      sendHtmlPage(client, "Escalier 2 execute");
      return;
    }
    
    pos = request.indexOf("escalier3");
    if (pos != -1) {
      sequenceActive = true;
      countLeft = 0; countRight = 0;
      Escalier3();
      delay(100);
      sequenceActive = false;
      
      sendHtmlPage(client, "Escalier 3 execute");
      return;
    }
    
    pos = request.indexOf("pomme");
    if (pos != -1) {
      sequenceActive = true;
      countLeft = 0; countRight = 0;
      sequencePomme();
      delay(100);
      sequenceActive = false;
      
      sendHtmlPage(client, "Sequence Pomme execute");
      return;
    }
    
    pos = request.indexOf("nord");
    if (pos != -1) {
      orienterVersNord = true;
      nordAtteint = false;
      
      sendHtmlPage(client, "Orientation vers le Nord lancee");
      return;
    }
    
    pos = request.indexOf("d=");
    if (pos != -1) {
      int fin = request.indexOf(" ", pos);
      String param = request.substring(pos+2, fin);
      distanceCM = param.toFloat();
      seuilImpulsions = distanceCM * IMPULSIONS_PAR_CM;
      countLeft = 0;
      countRight = 0;
      deplacementFait = false;
      correctionActive = true;
      
      sendHtmlPage(client, "Avancement de " + param + "cm demarre");  
      return;
    }
    
    sendHtmlPage(client, "");
    sendHtmlPage(client, "");
  }
  
  long moyenne = (countLeft + countRight) / 2;
  long reste = seuilImpulsions - moyenne;
  
  if (correctionActive && !deplacementFait) {
    if (reste <= 5) {
      arreter();
      deplacementFait = true;
      correctionActive = false;
    } else {
      avancerCorrige();
      if (countLeft % 100 == 0 || countRight % 100 == 0) {
      }
    }
  }
  
  if (!deplacementFait && moyenne >= seuilImpulsions) {
    arreter();
    correctionActive = false;
    deplacementFait = true;
  }
  
  if (orienterVersNord && !nordAtteint) {
    float angle = lireAngleNord();
    float erreur = calculerErreurAngle(angle);
    int pwm = calculerPWM(erreur);
    
    if (abs(erreur) <= TOLERANCE) {
      arreter();
      nordAtteint = true;
      avancerApresNord = true;
    } else if (erreur > 0) {
      gauche(pwm, 60);
    } else {
      droite(pwm, 60);
    }
    delay(500);
  }
  
  if (avancerApresNord && nordAtteint) {
    avancerApresNord = false;
    rotation(360.0);
    delay(500);
    dessinerFlecheNord();
    delay(500);
  }
}

// -------------------------------------------------HTML-----------------------------------------------

void sendHtmlPage(WiFiClient client, String message) {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>Drawbot</title>";
  html += "<style>";
  html += "body{font-family:Arial;text-align:center;margin-top:50px;background:#e6f2ff;}";
  html += "h1{color:#333;}";
  html += "form{background:#fff;padding:20px;border-radius:10px;box-shadow:0 0 5px #999;display:inline-block;margin-bottom:15px;}";
  html += "input{padding:8px;margin:8px;border-radius:4px;}";
  html += "input[type=submit]{background:#ff66b3;color:white;border:none;cursor:pointer;}";
  html += "input[type=submit]:hover{background:#ff3399;}";
  html += ".message{background:#e7f3fe;padding:10px;border-radius:5px;margin:10px auto;max-width:400px;}";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>Commandes Drawbot</h1>";
  
  if (message != "") {
    html += "<div class='message'>" + message + "</div><br>";
  }
  
  html += "<form method='GET'><label>Distance (cm)</label><br>";
  html += "<input type='number' name='d' min='1' max='100' value='10'><br>";
  html += "<input type='submit' value='Avancer'></form><br><br>";
  
  html += "<form method='GET'><input type='hidden' name='angle90' value='1'>";
  html += "<input type='submit' value='Angle 90'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='angle360' value='1'>";
  html += "<input type='submit' value='Cercle'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='nord' value='1'>";
  html += "<input type='submit' value='Indiquer Nord'></form><br><br>";
  
  html += "<h2>Commandes Escalier</h2>";
  
  html += "<form method='GET'><input type='hidden' name='escalier1' value='1'>";
  html += "<input type='submit' value='Escalier 1'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='escalier2' value='1'>";
  html += "<input type='submit' value='Escalier 2'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='escalier3' value='1'>";
  html += "<input type='submit' value='Escalier 3'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='pomme' value='1'>";
  html += "<input type='submit' value='Pomme'></form>";

  html += "</body></html>";
  
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println(); 
  client.println(html); 
  client.stop();
}