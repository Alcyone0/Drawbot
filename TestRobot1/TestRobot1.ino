#include <WiFi.h>
#include <Wire.h>
#include <SparkFunLSM6DS3.h>

// === IMU ===
LSM6DS3 imu(I2C_MODE, 0x6B);
float biaisGyroZ = 0.0;

// === WiFi ===
const char* ssid = "Drawbot_WIFI";
const char* password = "12345678";
WiFiServer server(80);

// === Encodeurs ===
const int encoderLeftA = 27;
const int encoderRightA = 33;

// === Moteurs ===
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16
#define EN_D   23
#define EN_G   4

// === Magnetometre ===
#define MAG_ADDR 0x1E
const float offsetX = -1178; // Ou 1973.5
const float offsetY = -844; // Ou -2641.5
const float TOLERANCE = 2.0;
bool orienterVersNord = false;
bool nordAtteint = true;
bool avancerApresNord = false; // Flag pour déclencher l'avancement après orientation nord

// === Variables ===
volatile long countLeft = 0;
volatile long countRight = 0;
const float IMPULSIONS_PAR_CM = 34.0;
float distanceCM = 0.0;
long seuilImpulsions = 0;
const int basePWM_G = 70;
const int basePWM_D = 70;
const int PWM_ROTATION = 80;
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

// ---------------------------------------------------CALCULER ERREUR---------------------------------------------------

float calculerErreur(float angle, float cible = 0.0) {
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

// ---------------------------------------------------PARAMETRES ESCALIER---------------------------------------------------

const int seuil1 = 195;             // Nombre de ticks pour la phase 1
const int seuil2 = 1400;            // Nombre de ticks pour la phase 2
const float coefP1 = 0.5;           // Coefficient proportionnel PID pour phase 1
const float coefP2 = 0.11;          // Coefficient proportionnel PID pour phase 2

// ---------------------------------------------------PWM BASE MOTEURS---------------------------------------------------

// PWM initialement déséquilibrés pour corriger la trajectoire dés le départ
const int vitesseDroite1 = 110;     // Moteur droit plus rapide
const int vitesseGauche1 = 15;      // Moteur gauche plus lent
const int vitesseDroite2 = 60;      // Valeurs équilibrées pour phase 2
const int vitesseGauche2 = 60;

// ---------------------------------------------------PID INTERNE---------------------------------------------------

float erreurActuelle = 0;           // Différence actuelle entre les deux roues
float erreurAvant = 0;              // Dernière erreur connue (pour dérivée)
float cumulErreurs = 0;             // Somme des erreurs (intégrale)

// ---------------------------------------------------FONCTION PID---------------------------------------------------
float ajustementPID(float err, float kp) {
  cumulErreurs += err;              // Mise à jour du cumul d'erreurs
  float sortie = kp * err;          // Seulement le terme proportionnel ici
  erreurAvant = err;                // Stockage de l'erreur pour la prochaine boucle
  return sortie;                    // Retourne la correction calculée
}

// ---------------------------------------------------FONCTION AVANCER CM---------------------------------------------------

void avancerCM(float cm) {
  long cible = cm * IMPULSIONS_PAR_CM;     // Conversion de la distance en ticks
  countLeft = 0;                    // Réinitialisation des compteurs d'encodeurs
  countRight = 0;
  erreurAvant = 0;                  // Réinitialisation PID
  cumulErreurs = 0;

  int pwmG = vitesseGauche1;        // PWM initial gauche (déséquilibré exprès)
  int pwmD = vitesseDroite1;        // PWM initial droit

  while ((countLeft + countRight) / 2 < cible) {   // Condition d'arrêt basée sur moyenne
    float delta = (float)countLeft - (float)countRight; // Erreur entre les roues
    float corr = ajustementPID(delta, coefP1);          // Correction PID

    int valD = constrain(pwmD + corr, 0, 255);     // Application correction droite
    int valG = constrain(pwmG - corr, 0, 255);     // Application correction gauche

    // Commande des moteurs : avancer (marche avant opposée entre D et G)
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
    analogWrite(EN_D, valD);    // PWM moteur droit
    analogWrite(EN_G, valG);    // PWM moteur gauche

    delay(20);                  // Temporisation pour éviter surcharge CPU
  }
  arreter();                    // Stoppe les moteurs à la fin
}

// ------------------------------------------------------PHASE 1---------------------------------------------------

void phase1() {
  countLeft = 0;
  countRight = 0;
  erreurAvant = 0;
  cumulErreurs = 0;
  
  Serial.println("Phase 1 démarrage");

  while (countRight < seuil1) {     // Avance jusqu'à un nombre de ticks
    float ecart = (float)countLeft - (float)countRight; // Écart entre les roues
    float corr = ajustementPID(ecart, coefP1);          // Correction PID

    int valD = constrain(vitesseDroite1 + corr, 0, 255); // PWM corrigée
    int valG = constrain(vitesseGauche1 - corr, 0, 255);

    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
    analogWrite(EN_D, valD);
    analogWrite(EN_G, valG);

    delay(20);
  }
  arreter();
  Serial.println("Phase 1 terminée");
}

// ---------------------------------------------------PHASE 2---------------------------------------------------

void phase2() {
  countLeft = 0;
  countRight = 0;
  erreurAvant = 0;
  cumulErreurs = 0;
  
  Serial.println("Phase 2 démarrage");

  while (countLeft < seuil2) {
    float ecart = (float)countLeft - (float)countRight;
    float corr = ajustementPID(ecart, coefP2);

    int valD = constrain(vitesseDroite2 + corr, 0, 255);
    int valG = constrain(vitesseGauche2 - corr, 0, 255);

    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
    analogWrite(EN_D, valD);
    analogWrite(EN_G, valG);

    delay(20);
  }
  arreter();
  Serial.println("Phase 2 terminée");
}

// ------------------------------------------------------SEQUENCE ESCALIER---------------------------------------------------

void sequenceEscalier() {
  Serial.println("=== Début de la séquence ESCALIER ===");
  avancerCM(20);        // Premier déplacement
  delay(300);           // Stabilisation
  phase1();             // Petite marche
  delay(300);           // Stabilisation
  phase2();             // Grande marche
  delay(300);           // Stabilisation
  Serial.println("=== Fin de la séquence ===");
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

// ------------------------------------------------------LOOP---------------------------------------------------

void loop() {
  WiFiClient client = server.available();
  if (client) {
    while (!client.available()) delay(1);
    String request = client.readStringUntil('\r');
    client.flush();
    int pos;
    pos = request.indexOf("d=");
    if (pos != -1) {
      float val = request.substring(pos + 2).toFloat();
      distanceCM = val;
      seuilImpulsions = distanceCM * IMPULSIONS_PAR_CM;
      countLeft = 0; countRight = 0;
      deplacementFait = false;
      correctionActive = true;
    }
    pos = request.indexOf("angle90=1");
    if (pos != -1) {
      rotation(90);
    }
    pos = request.indexOf("angle360=1");
    if (pos != -1) {
      rotation(360);
    }
    pos = request.indexOf("nord=1");
    if (pos != -1) {
      nordAtteint = false;
      orienterVersNord = true;
    }
    pos = request.indexOf("escalier=1");
    if (pos != -1) {
      nordAtteint = true;
      orienterVersNord = false;
      sequenceEscalier();
    }
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Drawbot</title>";
    html += "<style>body{background:#ffe6f0;font-family:Arial;text-align:center;padding:40px;}";
    html += "form{background:#fff;padding:25px;border-radius:12px;box-shadow:0 0 10px #cc0066;display:inline-block;}";
    html += "input{padding:10px;margin:10px;border-radius:5px;}";
    html += "input[type=submit]{background:#ff66a3;color:white;border:none;cursor:pointer;}";
    html += "input[type=submit]:hover{background:#cc0066;}</style></head><body>";
    html += "<h1>Commandes Drawbot</h1>";
    html += "<form method='GET'><label>Distance (cm)</label><br>";
    html += "<input type='number' name='d' min='1' max='100'><br>";
    html += "<input type='submit' value='Avancer'></form><br><br>";
    html += "<form method='GET'><input type='hidden' name='angle90' value='1'>";
    html += "<input type='submit' value='Angle droit 90 deg'></form><br><br>";
    html += "<form method='GET'><input type='hidden' name='angle360' value='1'>";
    html += "<input type='submit' value='Rotation compl\u00e8te 360 deg'></form><br><br>";
    html += "<form method='GET'><input type='hidden' name='nord' value='1'>";
    html += "<input type='submit' value='Indiquer le Nord'></form><br><br>";
    html += "<form method='GET'><input type='hidden' name='escalier' value='1'>";
    html += "<input type='submit' value='Tracer escalier'></form></body></html>";
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println(); client.println(html); client.stop();
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
    }
  }
  if (!deplacementFait && moyenne >= seuilImpulsions) {
    arreter();
    correctionActive = false;
    deplacementFait = true;
  }
  if (orienterVersNord && !nordAtteint) {
    float angle = lireAngleNord();
    float erreur = calculerErreur(angle);
    int pwm = calculerPWM(erreur);
    if (abs(erreur) <= TOLERANCE) {
      arreter();
      nordAtteint = true;
      avancerApresNord = true; // Déclencher l'avancement après avoir atteint le nord
      Serial.println("Nord atteint, déclenchement avance automatique de 10cm");
    } else if (erreur > 0) {
      gauche(pwm, 60);
    } else {
      droite(pwm, 60);
    }
    delay(250);
  }
  
  // Séquence automatique après avoir trouvé le nord
  if (avancerApresNord && nordAtteint) {
    avancerApresNord = false; // Réinitialiser le flag pour éviter de répéter l'action
    Serial.println("Démarrage séquence après orientation nord");

    avancerPrecisement(10);
    delay(500); // Pause pour stabilisation
    
    reculerPrecisement(10);
    delay(500); // Pause pour stabilisation

    avancerPrecisement(10);
  }
}
