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
const float offsetX = 1973.5; // Ou -1178
const float offsetY = -2641.5; // Ou -844
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

// Version simplifiée sans correction d'erreur dynamique
void avancerCorrige() {
  digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
  
  // Utilisation du même système PID que pour l'escalier
  float erreur_encoder = countRight - countLeft;
  float correction_pid = 4.5 * erreur_encoder; // On garde le Kp original
  
  long moyenne = (countLeft + countRight) / 2;
  long reste = seuilImpulsions - moyenne;
  
  // On simplifie pour une vitesse plus constante comme dans l'escalier
  int pwmBase = basePWM_D;
  int pwmD = constrain(pwmBase + correction_pid, 70, 255);
  int pwmG = constrain(pwmBase - correction_pid, 70, 255);
  
  analogWrite(EN_D, pwmD);
  analogWrite(EN_G, pwmG);
  
  // Log pour débogage
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

// --- PARAMETRES ESCALIER --- //

// Variables modifiables dans le site web
int distance_cm = 20;
const float TICKS_PAR_CM = IMPULSIONS_PAR_CM; // Utilise la même valeur que le reste du code

// La fonction reculer a été supprimée car non utilisée

// Étape 1 du déplacement en escalier
void etape1() {
  const int pwm_d_base = 190; // PWM roue droite base
  const int pwm_g_base = 10;  // PWM roue gauche base
  const int seuil_ticks = 80; // seuil ticks pour étape1 (réduit pour un virage plus court)

  const float Kp = 0.5;    // Coefficient proportionnel
  const float Ki = 0;
  const float Kd = 0;

  // Variables PID pour l'escalier
  float erreur = 0;
  float erreur_precedente = 0;
  float somme_erreurs = 0;
  float correction = 0;

  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
  digitalWrite(IN_1_G, HIGH);  digitalWrite(IN_2_G, LOW);
    
  while (countRight < seuil_ticks) {
    erreur = (float)countLeft - (float)countRight;
    somme_erreurs += erreur;
    correction = Kp * erreur + Ki * somme_erreurs + Kd * (erreur - erreur_precedente);
    erreur_precedente = erreur;

    int pwm_d = pwm_d_base + correction;
    int pwm_g = pwm_g_base - correction;

    pwm_d = constrain(pwm_d, 0, 255);
    pwm_g = constrain(pwm_g, 0, 255);
    analogWrite(EN_D, pwm_d);
    analogWrite(EN_G, pwm_g);

    delay(20);
  }
  //arreter();
  analogWrite(EN_D, 0); analogWrite(EN_G, 0);
  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, LOW);
}

// Étape 2 du déplacement en escalier
void etape2() {
  const int pwm_d_base = 50; // PWM roue droite base
  const int pwm_g_base = 110;  // PWM roue gauche base
  const int seuil_ticks = 1400; // seuil ticks pour étape1 (réduit pour un virage plus court)

  float Kp = 0.11;  // Coefficient proportionnel pour la 2e étape
  float Ki = 0;
  float Kd = 0;

  // Variables PID pour l'escalier
  float erreur = 0;
  float erreur_precedente = 0;
  float somme_erreurs = 0;
  float correction = 0;

  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH);
  digitalWrite(IN_1_G, HIGH);  digitalWrite(IN_2_G, LOW);
    
  while (countLeft < seuil_ticks) {
    erreur = (float)countLeft - (float)countRight;
    somme_erreurs += erreur;
    correction = Kp * erreur + Ki * somme_erreurs + Kd * (erreur - erreur_precedente);
    erreur_precedente = erreur;

    int pwm_d = pwm_d_base + correction;
    int pwm_g = pwm_g_base - correction;

    pwm_d = constrain(pwm_d, 0, 255);
    pwm_g = constrain(pwm_g, 0, 255);
    analogWrite(EN_D, pwm_d);
    analogWrite(EN_G, pwm_g);

    delay(20);
  }
  //arreter();
  analogWrite(EN_D, 0); analogWrite(EN_G, 0);
  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, LOW);
}

// Fonction escalier principale
void sequenceEscalier() {
    // Réinitialisation des variables globales pour éviter les conflits
    deplacementFait = true;
    correctionActive = false;
    nordAtteint = true;
    orienterVersNord = false;
    
    Serial.println("=== Début de la séquence ESCALIER ===");

    etape1();
    arreter();
    delay(300); // Pause pour stabiliser

    etape2();
    arreter();
    delay(300); // Pause pour stabiliser
}

void dessinerFlecheNord() {

  // Segment diagonal gauche
  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < 100) { // 3 cm environ
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH); // recule Droit
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW); // recule Gauche
    analogWrite(EN_D, 90);
    analogWrite(EN_G, 60);
  }
  arreter();
  delay(200);

  // Retour
  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < 100) {
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW); // avance D
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH); // avance G
    analogWrite(EN_D, 90);
    analogWrite(EN_G, 60);
  }
  arreter();
  delay(200);

  // Segment diagonal droit
  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < 100) {
    digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, HIGH); // recule D
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW); // recule G
    analogWrite(EN_D, 60);
    analogWrite(EN_G, 90);
  }
  arreter();
  delay(200);

  // Retour
  countLeft = 0; countRight = 0;
  while ((countLeft + countRight) / 2 < 100) {
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW); // avance D
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH); // avance G
    analogWrite(EN_D, 60);
    analogWrite(EN_G, 90);
  }
  arreter();
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

// Prototype de la fonction sendHtmlPage pour pouvoir l'utiliser dans loop()
void sendHtmlPage(WiFiClient client, String message = "");

// ------------------------------------------------------LOOP---------------------------------------------------

void loop() {
  // Gérer les requêtes WiFi entrantes
  WiFiClient client = server.available();
  if (client) {
    Serial.println("[loop] Nouvelle connexion client");
    while (!client.available()) delay(1);
    String request = client.readStringUntil('\r');
    client.flush();
    int pos;
    
    // Vérification pour etape1
    pos = request.indexOf("etape1");
    if (pos != -1) {
      sequenceActive = true;
      Serial.println("[loop] Commande reçue: Etape 1");
      etape1();
      delay(100);
      sequenceActive = false;
      
      // Envoi de la page HTML avec message de confirmation
      sendHtmlPage(client, "Étape 1 exécutée avec succès");
      return;
    }
    
    // Vérification pour etape2
    pos = request.indexOf("etape2");
    if (pos != -1) {
      sequenceActive = true;
      Serial.println("[loop] Commande reçue: Etape 2");
      etape2();
      delay(100);
      sequenceActive = false;
      
      sendHtmlPage(client, "Étape 2 exécutée avec succès");
      return;
    }
    
    // Vérification pour avancer_distance
    pos = request.indexOf("avancer_distance");
    if (pos != -1) {
      sequenceActive = true;
      Serial.println("[loop] Commande reçue: Avancer distance");
      avancerPrecisement(distance_cm);
      delay(100);
      sequenceActive = false;
      
      sendHtmlPage(client, "Avancé de " + String(distance_cm) + "cm");
      return;
    }
    
    // Vérification pour angle90
    pos = request.indexOf("angle90");
    if (pos != -1) {
      Serial.println("[loop] Commande reçue: Rotation 90 degrés");
      rotation(90.0);
      
      sendHtmlPage(client, "Rotation 90° effectuée");
      return;
    }
    
    // Vérification pour angle360
    pos = request.indexOf("angle360");
    if (pos != -1) {
      Serial.println("[loop] Commande reçue: Rotation 360 degrés");
      rotation(360.0);
      
      sendHtmlPage(client, "Rotation 360° effectuée");
      return;
    }
    
    // Vérification pour nord
    pos = request.indexOf("nord");
    if (pos != -1) {
      Serial.println("[loop] Commande reçue: Orienter vers le Nord");
      orienterVersNord = true;
      nordAtteint = false;
      
      sendHtmlPage(client, "Orientation vers le Nord lancée");
      return;
    }
    
    // Commande avancer d'une distance
    pos = request.indexOf("d=");
    if (pos != -1) {
      // Extraction du paramètre de distance
      int fin = request.indexOf(" ", pos);
      String param = request.substring(pos+2, fin);
      distanceCM = param.toFloat();
      seuilImpulsions = distanceCM * IMPULSIONS_PAR_CM;
      countLeft = 0;
      countRight = 0;
      deplacementFait = false;
      correctionActive = true;
      
      Serial.print("[loop] Commande reçue: Avancer de ");
      Serial.print(distanceCM);
      Serial.println(" cm");
      
      // Pour ce cas, on envoie la page HTML avec un message de confirmation
      sendHtmlPage(client, "Avancement de " + param + "cm démarré");
      return;
    }
    
    // Page d'accueil par défaut
    sendHtmlPage(client, "");
  }
  
  // PARTIE 1: GESTION DU DÉPLACEMENT NORMAL
  long moyenne = (countLeft + countRight) / 2;
  long reste = seuilImpulsions - moyenne;
  
  if (correctionActive && !deplacementFait) {
    if (reste <= 5) {
      // Fin du déplacement - objectif atteint
      arreter();
      deplacementFait = true;
      correctionActive = false;
      Serial.println("[loop] Déplacement terminé avec succès");
    } else {
      // Déplacement en cours avec PID
      avancerCorrige();
      // Log pour suivi des encodeurs
      if (countLeft % 100 == 0 || countRight % 100 == 0) {
        Serial.print("[loop] Encodeurs - G/D: "); 
        Serial.print(countLeft); Serial.print("/");
        Serial.println(countRight);
      }
    }
  }
  
  // Sécurité pour arrêter si le seuil est atteint mais le flag n'est pas mis à jour
  if (!deplacementFait && moyenne >= seuilImpulsions) {
    arreter();
    correctionActive = false;
    deplacementFait = true;
    Serial.println("[loop] Déplacement terminé par limite d'impulsions");
  }
  
  // PARTIE 2: GESTION DE L'ORIENTATION NORD
  if (orienterVersNord && !nordAtteint) {
    float angle = lireAngleNord();
    float erreur = calculerErreurAngle(angle);
    int pwm = calculerPWM(erreur);
    
    if (abs(erreur) <= TOLERANCE) {
      arreter();
      nordAtteint = true;
      avancerApresNord = true; // Déclencher l'avancement après avoir atteint le nord
      Serial.println("[loop] Nord atteint! Déclenchement de la séquence automatique");
    } else if (erreur > 0) {
      gauche(pwm, 60);
    } else {
      droite(pwm, 60);
    }
    delay(500);
  }
  
  // PARTIE 3: SÉQUENCE AUTOMATIQUE APRÈS NORD
  if (avancerApresNord && nordAtteint) {
    avancerApresNord = false; // Réinitialiser le flag pour éviter de répéter l'action
    rotation(360.0);
    delay(500);
    dessinerFlecheNord();
    delay(500); // Pause pour stabilisation
  }
}

// Fonction pour envoyer la page HTML avec interface utilisateur
void sendHtmlPage(WiFiClient client, String message) {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>Drawbot Controller</title>";
  html += "<style>";
  html += "body{font-family:Arial;text-align:center;margin-top:50px;background:#f2f2f2;}";
  html += "h1{color:#ff0066;}";
  html += "form{background:#fff;padding:25px;border-radius:12px;box-shadow:0 0 10px #cc0066;display:inline-block;margin-bottom:20px;}";
  html += "input{padding:10px;margin:10px;border-radius:5px;}";
  html += "input[type=submit]{background:#ff66a3;color:white;border:none;cursor:pointer;}";
  html += "input[type=submit]:hover{background:#cc0066;}";
  html += ".message{background:#ffe6f0;padding:15px;border-radius:8px;margin:10px auto;max-width:500px;color:#cc0066;font-weight:bold;}";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>Commandes Drawbot</h1>";
  
  // Affichage du message de statut si présent
  if (message != "") {
    html += "<div class='message'>" + message + "</div><br>";
  }
  
  // Formulaire pour avancer d'une distance
  html += "<form method='GET'><label>Distance (cm)</label><br>";
  html += "<input type='number' name='d' min='1' max='100' value='10'><br>";
  html += "<input type='submit' value='Avancer'></form><br><br>";
  
  // Commandes de rotation
  html += "<form method='GET'><input type='hidden' name='angle90' value='1'>";
  html += "<input type='submit' value='Angle droit 90 deg'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='angle360' value='1'>";
  html += "<input type='submit' value='Rotation complète 360 deg'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='nord' value='1'>";
  html += "<input type='submit' value='Indiquer le Nord'></form><br><br>";
  
  // Commandes d'escalier
  html += "<h2>Commandes Étapes Escalier</h2>";
  html += "<form method='GET'><input type='hidden' name='avancer_distance' value='1'>";
  html += "<input type='submit' value='1. Avancer 20cm'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='etape1' value='1'>";
  html += "<input type='submit' value='2. Étape 1 (Première marche)'></form><br>";
  
  html += "<form method='GET'><input type='hidden' name='etape2' value='1'>";
  html += "<input type='submit' value='3. Étape 2 (Deuxième marche)'></form>";
  
  html += "</body></html>";
  
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println(); 
  client.println(html); 
  client.stop();
  Serial.println("[loop] Page HTML envoyée");
}
