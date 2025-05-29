#include "PID.h"
#include <Arduino.h>

// Définition des constantes

// Constructeur
PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd, int POn, int ControllerDirection) {
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    
    // Valeurs par défaut
    SampleTime = 100; // 100ms
    SetOutputLimits(0, 255);
    
    SetControllerDirection(ControllerDirection);
    SetTunings(Kp, Ki, Kd, POn);
    
    lastTime = millis() - SampleTime;
}

// Calcul de la sortie PID
void PID::Compute() {
    if (!inAuto) return;
    
    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    
    if (timeChange >= SampleTime) {
        // Calcul de l'erreur
        double input = *myInput;
        double error = *mySetpoint - input;
        double dInput = (input - lastInput);
        
        // Composante intégrale
        outputSum += (ki * error);
        
        // Proportionnel sur mesure (si activé)
        if (pOnM) outputSum -= pOnMKp * dInput;
        
        // Anti-windup
        if (outputSum > outMax) outputSum = outMax;
        else if (outputSum < outMin) outputSum = outMin;
        
        // Calcul de la sortie
        double output = 0;
        
        // Proportionnel sur erreur (si activé)
        if (pOnE) output = pOnEKp * error;
        
        // Ajouter le terme intégral et dérivé
        output += outputSum - kd * dInput;
        
        // Limites de sortie
        if (output > outMax) output = outMax;
        else if (output < outMin) output = outMin;
        
        // Mise à jour de la sortie
        *myOutput = output;
        
        // Mémorisation des valeurs
        lastInput = input;
        lastTime = now;
    }
}

// Configuration des paramètres PID
void PID::SetTunings(double Kp, double Ki, double Kd, int POn) {
    if (Kp < 0 || Ki < 0 || Kd < 0) return;
    
    // Mode proportionnel
    pOnE = (POn == P_ON_E);
    pOnM = (POn == P_ON_M);
    
    // Conversion des paramètres pour le temps d'échantillonnage
    double SampleTimeInSec = ((double)SampleTime) / 1000.0;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
    
    // Inversion des paramètres si direction inversée
    if (controllerDirection == REVERSE) {
        kp = 0 - kp;
        ki = 0 - ki;
        kd = 0 - kd;
    }
    
    // Stockage des valeurs pour les modes proportionnels
    pOnEKp = pOnE ? kp : 0;
    pOnMKp = pOnM ? kp : 0;
}

// Configuration du temps d'échantillonnage
void PID::SetSampleTime(int NewSampleTime) {
    if (NewSampleTime > 0) {
        double ratio = (double)NewSampleTime / (double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

// Configuration des limites de sortie
void PID::SetOutputLimits(double Min, double Max) {
    if (Min >= Max) return;
    
    outMin = Min;
    outMax = Max;
    
    // Ajustement si déjà en mode automatique
    if (inAuto) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;
        
        if (outputSum > outMax) outputSum = outMax;
        else if (outputSum < outMin) outputSum = outMin;
    }
}

// Changement de mode (Auto/Manuel)
void PID::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    
    // Si passage de manuel à auto
    if (newAuto && !inAuto) {
        Initialize();
    }
    
    inAuto = newAuto;
}

// Initialisation lors du passage en mode automatique
void PID::Initialize() {
    lastInput = *myInput;
    outputSum = *myOutput;
    
    // Anti-windup
    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin) outputSum = outMin;
}

// Configuration de la direction du contrôleur
void PID::SetControllerDirection(int Direction) {
    if (inAuto && Direction != controllerDirection) {
        kp = 0 - kp;
        ki = 0 - ki;
        kd = 0 - kd;
    }
    
    controllerDirection = Direction;
}
