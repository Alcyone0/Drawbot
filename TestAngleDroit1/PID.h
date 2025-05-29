#ifndef PID_h
#define PID_h

class PID {
  public:
    // Constantes statiques pour les modes
    static const int AUTOMATIC = 1;
    static const int MANUAL = 0;
    
    // Constantes statiques pour la direction
    static const int DIRECT = 0;
    static const int REVERSE = 1;
    
    // Constantes statiques pour le mode proportionnel
    static const int P_ON_M = 0;
    static const int P_ON_E = 1;
    
    // Constructeur
    PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection);
    
    // Calcul de sortie PID
    void Compute();
    
    // Configuration
    void SetTunings(double Kp, double Ki, double Kd, int POn = P_ON_E);
    void SetSampleTime(int NewSampleTime);
    void SetOutputLimits(double Min, double Max);
    void SetMode(int Mode);
    void SetControllerDirection(int Direction);
    
    // Initialisation
    void Initialize();
    
  private:
    double kp;
    double ki;
    double kd;
    
    double *myInput;
    double *myOutput;
    double *mySetpoint;
    
    unsigned long lastTime;
    double outputSum, lastInput;
    
    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto;
    
    bool pOnE;
    bool pOnM;
    int controllerDirection;
    double pOnEKp;
    double pOnMKp;
};

#endif
