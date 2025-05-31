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
const float offsetX = 1973.5;
const float offsetY = -2641.5;
const float TOLERANCE = 2.0;
bool orienterVersNord = false;
bool nordAtteint = true;

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

void arreter() {
  analogWrite(EN_D, 0); analogWrite(EN_G, 0);
  digitalWrite(IN_1_D, LOW); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW); digitalWrite(IN_2_G, LOW);
}

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

float calculerErreur(float angle, float cible = 0.0) {
  float erreur = angle - cible;
  if (erreur > 180.0) erreur -= 360.0;
  if (erreur < -180.0) erreur += 360.0;
  return erreur;
}

int calculerPWM(float erreur) {
  float absErr = abs(erreur);
  if (absErr <= 5.0) return 80;
  else if (absErr <= 30.0) return 100;
  else return 150;
}

void tournerGauche(int pwm) {
  digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
  analogWrite(EN_D, pwm);
  analogWrite(EN_G, pwm);
}

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

void calibrerGyro() {
  float somme = 0.0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    somme += imu.readFloatGyroZ();
    delay(5);
  }
  biaisGyroZ = somme / N;
}

// === Système de logs ===
String logMessages = "";            // Buffer pour stocker les messages de log
int maxLogLines = 20;               // Nombre maximal de lignes de log à conserver

// Fonction pour ajouter un message au log
void addLog(String message) {
  String timestamp = String(millis()/1000.0, 1) + "s: "; // Horodatage en secondes
  logMessages = timestamp + message + "<br>" + logMessages;
  
  // Limiter le nombre de lignes dans le log
  int count = 0;
  int index = 0;
  while ((index = logMessages.indexOf("<br>", index + 1)) != -1) {
    count++;
    if (count >= maxLogLines) {
      logMessages = logMessages.substring(0, index + 4);
      break;
    }
  }
  
  // Afficher aussi sur le moniteur série
  Serial.println(timestamp + message);
}

// === Paramètres de l'escalier (variables ajustables) ===
int seuil1 = 195;             // Nombre de ticks pour la phase 1
int seuil2 = 1400;            // Nombre de ticks pour la phase 2
float coefP1 = 0.5;           // Coefficient proportionnel PID pour phase 1
float coefP2 = 0.11;          // Coefficient proportionnel PID pour phase 2

// === PWM de base pour les moteurs ===
// PWM initialement déséquilibrés pour corriger la trajectoire dès le départ
int vitesseDroite1 = 110;     // Moteur droit plus rapide pour phase 1
int vitesseGauche1 = 15;      // Moteur gauche plus lent pour phase 1
int vitesseDroite2 = 60;      // Valeurs équilibrées pour phase 2
int vitesseGauche2 = 60;      // Valeurs équilibrées pour phase 2

// Variables pour suivre l'état de l'exécution
bool logEncoders = false;     // Activer/désactiver le log des encodeurs
long lastLogTime = 0;         // Dernière fois qu'un log a été envoyé

// === PID interne ===
float erreurActuelle = 0;           // Différence actuelle entre les deux roues
float erreurAvant = 0;              // Dernière erreur connue (pour dérivée)
float cumulErreurs = 0;             // Somme des erreurs (intégrale)

// Fonction générique de calcul PID
float ajustementPID(float err, float kp) {
  cumulErreurs += err;              // Mise à jour du cumul d'erreurs
  float sortie = kp * err;          // Seulement le terme proportionnel ici
  erreurAvant = err;                // Stockage de l'erreur pour la prochaine boucle
  return sortie;                    // Retourne la correction calculée
}

// Fonction pour avancer d'une certaine distance (en cm) en ligne droite
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

// Étape 1 : avance d'un petit segment
void phase1() {
  countLeft = 0;
  countRight = 0;
  erreurAvant = 0;
  cumulErreurs = 0;
  
  addLog("Phase 1 démarrage - Seuil: " + String(seuil1) + ", PWM D/G: " + String(vitesseDroite1) + "/" + String(vitesseGauche1) + ", coefP: " + String(coefP1));

  // Timeout de sécurité pour éviter de tourner indéfiniment
  unsigned long startTime = millis();
  const unsigned long TIMEOUT = 5000; // 5 secondes maximum
  
  // On surveille la moyenne des deux encodeurs pour plus de stabilité
  // et on s'arrête si l'un des deux atteint 2x la valeur seuil (pour éviter les blocages)
  while (((countRight + countLeft)/2 < seuil1) && 
         (countRight < seuil1*2) && 
         (countLeft < seuil1*2) && 
         (millis() - startTime < TIMEOUT)) { // Critères d'arrêt multiples
    float ecart = (float)countLeft - (float)countRight; // Écart entre les roues
    float corr = ajustementPID(ecart, coefP1);          // Correction PID

    int valD = constrain(vitesseDroite1 + corr, 0, 255); // PWM corrigée
    int valG = constrain(vitesseGauche1 - corr, 0, 255);

    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
    analogWrite(EN_D, valD);
    analogWrite(EN_G, valG);

    // Log des encodeurs et corrections si activé
    if (logEncoders && millis() - lastLogTime > 200) { // Log toutes les 200ms
      addLog("P1 Enc D/G: " + String(countRight) + "/" + String(countLeft) + ", Écart: " + String(ecart, 1) + ", Corr: " + String(corr, 2) + ", PWM D/G: " + String(valD) + "/" + String(valG));
      lastLogTime = millis();
    }
    delay(20);
  }
  arreter();
  
  // Vérifier si on est sorti par timeout
  if (millis() - startTime >= TIMEOUT) {
    addLog("ATTENTION: Phase 1 interrompue par timeout de sécurité! Seuil non atteint.");
  }
  
  addLog("Phase 1 terminée - Encodeurs D/G: " + String(countRight) + "/" + String(countLeft));
}

// === Étape 2 : long déplacement (droit en diagonale) ===
void phase2() {
  countLeft = 0;
  countRight = 0;
  erreurAvant = 0;
  cumulErreurs = 0;

  // ⚙️ Réglages optimisés
  const int vitesseDroiteNew = 85;  // moins fort que la gauche 
  const int vitesseGaucheNew = 65;  // un peu plus fort
  const float coefPNew = 0.06;      // PID moins agressif

  // On utilise des constantes locales mais on peut mettre à jour les variables globales
  // pour que l'interface web montre les bonnes valeurs
  vitesseDroite2 = vitesseDroiteNew;
  vitesseGauche2 = vitesseGaucheNew;
  coefP2 = coefPNew;
  
  addLog("Phase 2 démarrage - Seuil: " + String(seuil2) + ", PWM D/G: " + 
          String(vitesseDroiteNew) + "/" + String(vitesseGaucheNew) + ", coefP: " + String(coefPNew));

  // Timeout de sécurité pour éviter de tourner indéfiniment
  unsigned long startTime = millis();
  const unsigned long TIMEOUT = 10000; // 10 secondes maximum (phase 2 plus longue)
  
  while (((countRight + countLeft)/2 < seuil2) && (countRight < seuil2*2) && 
         (countLeft < seuil2*2) && (millis() - startTime < TIMEOUT)) {
         
    float ecart = (float)countLeft - (float)countRight;
    float corr = ajustementPID(ecart, coefPNew);

    // 🔒 Planche minimale de PWM pour éviter blocage moteur
    int valD = constrain(vitesseDroiteNew + corr, 60, 255);
    int valG = constrain(vitesseGaucheNew - corr, 60, 255);

    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);
    analogWrite(EN_D, valD);
    analogWrite(EN_G, valG);
    
    // Log des encodeurs et corrections si activé
    if (logEncoders && millis() - lastLogTime > 200) {
      addLog("P2 Enc D/G: " + String(countRight) + "/" + String(countLeft) + 
             ", Écart: " + String(ecart, 1) + ", Corr: " + String(corr, 2) + 
             ", PWM D/G: " + String(valD) + "/" + String(valG));
      lastLogTime = millis();
    }
    delay(20);
  }
  
  arreter();
  
  // Vérifier si on est sorti par timeout
  if (millis() - startTime >= TIMEOUT) {
    addLog("ATTENTION: Phase 2 interrompue par timeout de sécurité! Seuil non atteint.");
  }
  
  addLog("Phase 2 terminée - Encodeurs D/G: " + String(countRight) + "/" + String(countLeft));
}

// Fonction principale de la séquence
void sequenceEscalier() {
  // Vider les logs précédents seulement si demandé
  if (logMessages.length() > 0) {
    addLog("=== Début de la séquence ESCALIER ===");
  } else {
    logMessages = "";
    addLog("=== Début de la séquence ESCALIER ===");
  }
  
  // Premier déplacement en ligne droite
  addLog("Démarrage avancerCM(20)");
  avancerCM(20);        
  delay(300);           // Stabilisation
  
  // Phase 1 - petite marche avec déséquilibre intentionnel
  phase1();            
  delay(300);           // Stabilisation
  
  // Phase 2 - grande marche droite
  phase2();             
  delay(300);           // Stabilisation
  
  addLog("=== Fin de la séquence ===");
}

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

void loop() {
  WiFiClient client = server.available();
  if (client) {
    while (!client.available()) delay(1);
    String request = client.readStringUntil('\r');
    client.flush();
    int pos;
    
    // Vérifier si c'est une requête de commande ou juste un chargement de page
    bool isCommandRequest = request.indexOf("GET /?" ) != -1;
    // Ne traiter les commandes que si c'est une requête avec paramètres
    
    // === Commandes de base ===
    if (isCommandRequest) {
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
    
      // === Contrôles des paramètres d'escalier ===
      // Paramètres Phase 1
      pos = request.indexOf("vd1=");
      if (pos != -1) vitesseDroite1 = request.substring(pos + 4).toInt();
      
      pos = request.indexOf("vg1=");
      if (pos != -1) vitesseGauche1 = request.substring(pos + 4).toInt();
      
      pos = request.indexOf("s1=");
      if (pos != -1) seuil1 = request.substring(pos + 3).toInt();
      
      pos = request.indexOf("kp1=");
      if (pos != -1) coefP1 = request.substring(pos + 4).toFloat();
      
      // Paramètres Phase 2
      pos = request.indexOf("vd2=");
      if (pos != -1) vitesseDroite2 = request.substring(pos + 4).toInt();
      
      pos = request.indexOf("vg2=");
      if (pos != -1) vitesseGauche2 = request.substring(pos + 4).toInt();
      
      pos = request.indexOf("s2=");
      if (pos != -1) seuil2 = request.substring(pos + 3).toInt();
      
      pos = request.indexOf("kp2=");
      if (pos != -1) coefP2 = request.substring(pos + 4).toFloat();
      
      // Contrôle des logs
      pos = request.indexOf("log_enc=1");
      if (pos != -1) logEncoders = true;
      
      pos = request.indexOf("log_enc=0");
      if (pos != -1) logEncoders = false;
      
      pos = request.indexOf("clear_log=1");
      if (pos != -1) logMessages = "";
    }
    
    // Construction de la page HTML
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Drawbot</title>";
    html += "<style>";
    html += "body{background:#f0f8ff;font-family:Arial;text-align:center;padding:20px;}";
    html += "h1{color:#003366;}";
    html += "h2{color:#0066cc;margin-top:30px;}";
    html += "form{background:#fff;padding:15px;border-radius:8px;box-shadow:0 0 5px #0099cc;display:inline-block;margin:10px;min-width:250px;vertical-align:top;}";
    html += "input, select{padding:8px;margin:5px;border-radius:4px;border:1px solid #ccc;}";
    html += "input[type=submit]{background:#0099cc;color:white;border:none;cursor:pointer;}";
    html += "input[type=submit]:hover{background:#0077aa;}";
    html += "label{display:inline-block;width:120px;text-align:right;margin-right:10px;}";
    html += "table{margin:10px auto;border-collapse:collapse;width:80%;}";
    html += "th, td{border:1px solid #ddd;padding:8px;text-align:left;}";
    html += "th{background-color:#f2f2f2;}";
    html += ".log-container{background:#fff;padding:15px;border-radius:8px;box-shadow:0 0 5px #0099cc;margin:20px auto;max-width:800px;max-height:400px;overflow-y:auto;text-align:left;}";
    html += ".current-values{font-weight:bold;color:#0066cc;}";
    html += "</style>";
    html += "<script>";
    html += "function updateSliderValue(sliderId, valueId) {";
    html += "  document.getElementById(valueId).innerHTML = document.getElementById(sliderId).value;";
    html += "}";
    html += "function refreshPage() {";
    html += "  setTimeout(function(){ location.reload(); }, 2000);";
    html += "}";
    html += "</script>";
    html += "</head><body>";
    
    // En-tête
    html += "<h1>Drawbot - Contrôle et Diagnostic</h1>";
    
    // Commandes de base
    html += "<h2>Commandes de Base</h2>";
    html += "<div style='display:flex;justify-content:center;flex-wrap:wrap;'>";
    html += "<form method='GET'><label>Distance (cm)</label>";
    html += "<input type='number' name='d' min='1' max='100' value='20'><br>";
    html += "<input type='submit' value='Avancer'></form>";
    
    html += "<form method='GET'><input type='hidden' name='angle90' value='1'>";
    html += "<input type='submit' value='Rotation 90°'></form>";
    
    html += "<form method='GET'><input type='hidden' name='nord' value='1'>";
    html += "<input type='submit' value='Orienter Nord'></form>";
    
    html += "<form method='GET' onsubmit='refreshPage()'><input type='hidden' name='escalier' value='1'>";
    html += "<input type='submit' value='Séquence Escalier'></form>";
    html += "</div>";
    
    // Paramètres Phase 1
    html += "<h2>Paramètres Phase 1 (Petite Marche)</h2>";
    html += "<div style='display:flex;justify-content:center;flex-wrap:wrap;'>";
    html += "<form method='GET'>";
    html += "<div><label>PWM Droite:</label>";
    html += "<input type='range' id='vd1Slider' name='vd1' min='0' max='255' value='" + String(vitesseDroite1) + "' oninput='updateSliderValue(\"vd1Slider\", \"vd1Value\")'>";
    html += "<span id='vd1Value'>" + String(vitesseDroite1) + "</span></div>";
    
    html += "<div><label>PWM Gauche:</label>";
    html += "<input type='range' id='vg1Slider' name='vg1' min='0' max='255' value='" + String(vitesseGauche1) + "' oninput='updateSliderValue(\"vg1Slider\", \"vg1Value\")'>";
    html += "<span id='vg1Value'>" + String(vitesseGauche1) + "</span></div>";
    
    html += "<div><label>Seuil (ticks):</label>";
    html += "<input type='number' name='s1' min='50' max='1000' value='" + String(seuil1) + "'></div>";
    
    html += "<div><label>Coef. PID:</label>";
    html += "<input type='number' name='kp1' min='0.01' max='2' step='0.01' value='" + String(coefP1, 2) + "'></div>";
    
    html += "<input type='submit' value='Mettre à jour Phase 1'></form>";
    html += "</div>";
    
    // Paramètres Phase 2
    html += "<h2>Paramètres Phase 2 (Grande Marche)</h2>";
    html += "<div style='display:flex;justify-content:center;flex-wrap:wrap;'>";
    html += "<form method='GET'>";
    html += "<div><label>PWM Droite:</label>";
    html += "<input type='range' id='vd2Slider' name='vd2' min='0' max='255' value='" + String(vitesseDroite2) + "' oninput='updateSliderValue(\"vd2Slider\", \"vd2Value\")'>";
    html += "<span id='vd2Value'>" + String(vitesseDroite2) + "</span></div>";
    
    html += "<div><label>PWM Gauche:</label>";
    html += "<input type='range' id='vg2Slider' name='vg2' min='0' max='255' value='" + String(vitesseGauche2) + "' oninput='updateSliderValue(\"vg2Slider\", \"vg2Value\")'>";
    html += "<span id='vg2Value'>" + String(vitesseGauche2) + "</span></div>";
    
    html += "<div><label>Seuil (ticks):</label>";
    html += "<input type='number' name='s2' min='500' max='5000' value='" + String(seuil2) + "'></div>";
    
    html += "<div><label>Coef. PID:</label>";
    html += "<input type='number' name='kp2' min='0.01' max='2' step='0.01' value='" + String(coefP2, 2) + "'></div>";
    
    html += "<input type='submit' value='Mettre à jour Phase 2'></form>";
    html += "</div>";
    
    // Contrôle des logs
    html += "<h2>Logs et Diagnostic</h2>";
    html += "<div style='display:flex;justify-content:center;flex-wrap:wrap;'>";
    html += "<form method='GET'>";
    html += "<input type='hidden' name='log_enc' value='" + String(logEncoders ? "0" : "1") + "'>";
    html += "<input type='submit' value='" + String(logEncoders ? "Désactiver" : "Activer") + " logs encodeurs'></form>";
    
    html += "<form method='GET'><input type='hidden' name='clear_log' value='1'>";
    html += "<input type='submit' value='Effacer logs'></form>";
    html += "</div>";
    
    // Affichage des logs
    html += "<div class='log-container'>";
    if (logMessages.length() > 0) {
      html += logMessages;
    } else {
      html += "<i>Aucun log disponible. Exécutez la séquence escalier pour générer des logs.</i>";
    }
    html += "</div>";
    
    // Tableau récapitulatif des valeurs actuelles
    html += "<h2>Valeurs Actuelles</h2>";
    html += "<table>";
    html += "<tr><th colspan='2'>Phase 1</th><th colspan='2'>Phase 2</th></tr>";
    html += "<tr><td>PWM Droite</td><td>" + String(vitesseDroite1) + "</td><td>PWM Droite</td><td>" + String(vitesseDroite2) + "</td></tr>";
    html += "<tr><td>PWM Gauche</td><td>" + String(vitesseGauche1) + "</td><td>PWM Gauche</td><td>" + String(vitesseGauche2) + "</td></tr>";
    html += "<tr><td>Seuil</td><td>" + String(seuil1) + "</td><td>Seuil</td><td>" + String(seuil2) + "</td></tr>";
    html += "<tr><td>Coef. PID</td><td>" + String(coefP1, 2) + "</td><td>Coef. PID</td><td>" + String(coefP2, 2) + "</td></tr>";
    html += "</table>";
    
    html += "</body></html>";
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
    } else if (erreur > 0) {
      gauche(pwm, 60);
    } else {
      droite(pwm, 60);
    }
    delay(250);
  }
}
