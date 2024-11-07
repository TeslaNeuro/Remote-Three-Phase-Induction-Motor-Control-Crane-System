// Three - Phase - Motor - Control
// Author: Arshia Keshvari Asl
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

   potValue = analogRead(potentiometer);
   input_frequency = map(potValue, 0, 1023, 0, 50);
   t = 1000/(input_frequency*6);

//   In case speed control dosent work uncomment this to perform debugging
//   lcd.setCursor(0,0);
//   lcd.print(input_frequency);
//   lcd.setCursor(0,1);
//   lcd.print(t);
//   lcd.setCursor(0,2);
//   lcd.print(potValue);
//   delayMicroseconds(1);

// Implement the on functionality code
  if (digitalRead(OffSw) == LOW){
    digitalWrite(OnOffCtrl, LOW);
    Direction_Proccess();
    }
    
  else if (digitalRead(OffSw) == HIGH){
    digitalWrite(OnOffCtrl, HIGH);
    ShutDown_All_Signals();
    }
  brake();
}


// Brake function
void brake(){

  while(digitalRead(manualbrake)){
    
    digitalWrite(OnOffCtrl,HIGH);  // Shutdown logic state
    ShutDown_All_Signals();

    }
    
  while(digitalRead(OffSw)){
    
    digitalWrite(OnOffCtrl,HIGH);
    ShutDown_All_Signals();
   
   } 
}
