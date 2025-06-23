// Three - Phase - Motor - Control
// Author: Arshia Keshvari
// POWER ENGINEERING

// Duty cycle = (TON/(TON + TOFF)) *100


// Pin allocation and configuration
byte A = 0;                           // Phase A
byte B = 1;                           // Phase B
byte C = 2;                           // Phase C
byte OnOffCtrl = 3;                   // Shutdown control of the motor LOW or HIGH states
byte OffSw = 4;                       // Stop or Start Switch LOW or HIGH states
byte DirectionSw = 5;                 // Clockwise or anti-clockwise switch LOW or HIGH states
byte manualbrake = 6;                 // Slowly slows down the motor until it reaches zero
int potentiometer = 15;               // 10k ohm variable resistor manually controlled by user

// Declaring Values/Paramaeteres to be calculated
float input_frequency = 0;            // input frequency determined by potentiometer value
float t = 0;                          // Time delay
int potValue;                         // int variable to later on store the value fo the pot
unsigned long previousMillis = 0;   
unsigned long brakeMillis = 0;     

byte stepIndex = 0;                 // Current commutation step (0-5)
bool clockwise = true;              // Motor direction

// Braking variables
unsigned long brakeStartTime = 0;
bool brakingActive = false;
const unsigned long brakeDuration = 3000; // Braking time in milliseconds (3 seconds)
float initialDelay = 0; // Stores delay when braking starts

// Commutation sequence
const byte phaseSequence[6][3] = {
  {HIGH, LOW, LOW},
  {HIGH, HIGH, LOW},
  {LOW, HIGH, LOW},
  {LOW, HIGH, HIGH},
  {LOW, LOW, HIGH},
  {HIGH, LOW, HIGH}
};


// Runs at the start of the program
void setup () {
   
   pinMode(A,OUTPUT);                 // Phase A
   pinMode(B,OUTPUT);                 // Phase B
   pinMode(C,OUTPUT);                 // Phase C
   pinMode(OnOffCtrl,OUTPUT);         // On/Off Control Output to IGBTS to turn on/off motor
   pinMode(potentiometer,INPUT);      // Potentiometer input
   pinMode(OffSw,INPUT);              // On/Off Control Input using a switch
   pinMode(DirectionSw,INPUT);        // Direction Switch Input using a switch.
   pinMode(manualbrake,INPUT);        // Manual brake button

  ShutDown_All_Signals();
}


// Shuts down the signals A, B ,and C
void ShutDown_All_Signals(){
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  }


void Turn_On_Signals_clockwise (){
  digitalWrite(A, HIGH);
  delay(t);
  digitalWrite(B, LOW);
  delay(t);
  digitalWrite(C, HIGH);
  delay(t);
  digitalWrite(A, LOW);
  delay(t);
  digitalWrite(B, HIGH);
  delay(t);
  digitalWrite(C, LOW);
  delay(t);
  }


void Turn_On_Signals_anti_clockwise(){
  digitalWrite(C, HIGH);
  delay(t);
  digitalWrite(B, LOW);
  delay(t);
  digitalWrite(A, HIGH);
  delay(t);
  digitalWrite(C, LOW);
  delay(t);
  digitalWrite(B, HIGH);
  delay(t);
  digitalWrite(A, LOW);
  delay(t);
  }


// Direction proccess of the motor or crane
void Direction_Proccess(){
  
  if (digitalRead(DirectionSw)){
    Turn_On_Signals_clockwise();
    }
    
  else{
    Turn_On_Signals_anti_clockwise();
    }
    
}

void loop() {
  // Read potentiometer and calculate frequency using logarithmic scaling
  potValue = analogRead(potentiometer);
  float minFreq = 1;   // Prevent zero
  float maxFreq = 50;  // Max frequency
  float scale = log10(maxFreq / minFreq);
  input_frequency = minFreq * pow(10, scale * potValue / 1023.0);
  t = 1000 / (input_frequency * 6);

  // Read direction switch
  clockwise = !digitalRead(DirectionSw);


  if (digitalRead(OffSw) == LOW && !digitalRead(manualbrake)) {
    digitalWrite(OnOffCtrl, LOW);
    brakingActive = false; // Cancel braking if motor is running again
    updateMotor();
  } else if (digitalRead(manualbrake)) {
    gradualBrake();
  } else {
    digitalWrite(OnOffCtrl, HIGH);
    ShutDown_All_Signals();
    brakingActive = false; // Reset braking state
  }

}

// Non-blocking motor step update
void updateMotor() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= t) {
    previousMillis = currentMillis;

    if (clockwise) {
        stepIndex = (stepIndex + 1) % 6;
    } else {
        stepIndex = (stepIndex == 0) ? 5 : stepIndex - 1;
    }

    digitalWrite(A, phaseSequence[stepIndex][0]);
    digitalWrite(B, phaseSequence[stepIndex][1]);
    digitalWrite(C, phaseSequence[stepIndex][2]);
  }
}

void gradualBrake() {
  unsigned long currentMillis = millis();

  if (!brakingActive) {
    // Initialize braking
    brakingActive = true;
    brakeStartTime = currentMillis;
    initialDelay = t; // Store the current delay
  }

  // Calculate elapsed braking time
  unsigned long elapsed = currentMillis - brakeStartTime;

  if (elapsed <= brakeDuration) {
    // Linearly increase delay to reduce speed
    float progress = (float)elapsed / brakeDuration; // 0.0 to 1.0
    t = initialDelay + progress * (1000 - initialDelay); // Gradually increase t to a max value
    updateMotor();
  } else {
    // Braking complete, shut down the motor
    digitalWrite(OnOffCtrl, HIGH);
    ShutDown_All_Signals();
    brakingActive = false; // Reset for future braking
  }
}

