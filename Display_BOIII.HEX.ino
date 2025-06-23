// Display for Crane System
// Author: Arshia Keshvari & Daniil Ivanov
// POWER ENGINEERING

# include "DHT.h"                   // Digital Humidity and Temperature sensor library
# include <LiquidCrystal_I2C.h>     // Liquid Crystal Display I2C Library

// Digital Humidity and Temperature sensor definitions
# define DHTPIN 3                   // digital pin 3         
# define DHTTYPE DHT11              // component version definition         
DHT dht(DHTPIN, DHTTYPE);           // dht variable class

// Pin allocation and configuration
byte OffSw = 6;                        // Stop or Start Switch LOW or HIGH states
byte DirectionSw = 5;                  // Clockwise or anti-clockwise switch LOW or HIGH states
byte manualbrake = 7;                  // Slowly slows down the motor until it reaches zero
int potentiometer = A0;                // 10k ohm variable resistor manually controlled by user
int load = A1;                         // load push button or platform
int stress = A2;                       // Stress push button or crane stress
const byte interpin = 2;               // Interrupt pin for anemometer                  
int ECHO = 10;                         // ECHO Pin of the ultrasonic sensor
int TRIG = 9;                          // Trigger pin of the ultrasonic sensor
const int dustpin = 8;                      // Data from dust sensor

// Declaring Values/Paramaeteres to be calculated
String Direction;                      // For storing direction status
float input_frequency = 0;             // Input frequency determined by potentiometer value
float t = 0;                           // Time delay (This controls pwm of motor)
float frequency = 0;                   // Frequency of the motor used in LCD display
float Speed = 0;                       // Speed of the motor
unsigned long pi = 3.141592653589;              // Pi value used for formulas
unsigned long angular_acc;                      // Angular Acceleration
unsigned long angular_velocity;                 // Angular Velocity
int RPM = 0;                           // Revolutions per minute
double moment_of_inertia = 0.67;       // Moment of inertia
float Torque;                          // Torque
float Power_Consumption;               // Power consumption of the motor
int potValue;                          // Used to store the value of the pot
int Mass;                              // Weight of the load
int Stress;
float distance;                        // Distance of the object

// LED Indicator thresholds
const int hot = 40;             // set hot parameter for dht
const int cold = 20;            // set cold parameter for dht

// Anemometer
volatile unsigned long sTime=0;           // Standard time 
unsigned long dataTimer=0;                // Data timer  
volatile float pulseTime=0;               // Pulse timing
volatile float culPulseTime=0;            // Current pulse time
volatile bool startt =true;               // Artificial starting time bolean state             
volatile unsigned int avgWindCount=0;     // Average wind speed count
float aSetting=60.0;                      // Anemometer setting set to 60
float aWSpeed;                            // Anemometer speed as a floating number    

// Dust sensor PPD42
unsigned long duration;               // Duration of pulses   
unsigned long sampletime_ms = 30000;  // Set sample time in milliseconds. Leave at 30,000 (30sec) for accuracy
unsigned long lowpulseoccupancy = 0;  // Low pulse occupancy
float ratio = 0;                      // Ratio of the dust particles
float concentration = 0;              // Concentration of the dust particles

// Address table for PCF8754 of LCD Display
LiquidCrystal_I2C lcd(0x20,20,4);     // LCD used for crane and motor status
LiquidCrystal_I2C lcd2(0x27,20,4);    // LCD used for sensors 
LiquidCrystal_I2C lcd3(0x25,20,4);    // LCD used for SOS or emergency

// For displaying at different times
unsigned long prevTime;
bool print_d1 = true;
bool print_d2 = false;


// Runs at the start of the program
void setup () {
   
   Serial.begin(9600);                // Set baud rate to 115200 can be 9600
   pinMode(potentiometer,INPUT);      // Potentiometer input
   pinMode(OffSw,INPUT);              // On/Off Control Input using a switch
   pinMode(DirectionSw,INPUT);        // Direction Switch Input using a switch.
   pinMode(manualbrake,INPUT);        // Manual brake button
   pinMode(load,INPUT);               // load push button
   pinMode(stress,INPUT);             // stress push button
   pinMode(interpin,INPUT_PULLUP);    // Anemometer wind pinout
   pinMode(dustpin,INPUT);            // Dust sensor pinout
   pinMode(TRIG,OUTPUT);              // Trigger output
   pinMode(ECHO,INPUT);               // ECHO input
   pinMode(11,OUTPUT);                // red led
   pinMode(12,OUTPUT);                // green led
   pinMode(13,OUTPUT);                // blue led

   dht.begin();                       // Activates dht sensor

   initializeLCDs();
   
   attachInterrupt(interpin,anemometerISR, RISING);
   prevTime = millis();
   dataTimer = millis();        
}

// ---------- Initialization ----------
void initializeLCDs() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print(F("Hi Dr. Nankoo"));
  lcd.setCursor(3, 1);
  lcd.print(F("Motor Control"));
  lcd.setCursor(3, 2);
  lcd.print(F("Running..."));

  lcd2.init();
  lcd2.backlight();
  lcd2.setCursor(3, 0);
  lcd2.print(F("Crane System!"));
  lcd2.setCursor(3, 1);
  lcd2.print(F("Sensors Activated"));

  lcd3.init();
  lcd3.backlight();

  delay(500);
  lcd.clear();
  lcd2.clear();
  lcd3.clear();
}

void Sensor_Display(){

  lcd2.setCursor(0,0);
  int RhA = dht.readHumidity();
  lcd2.print(F("Humidity: ")); lcd2.print(RhA); lcd2.print(F(" %"));
  
  lcd2.setCursor(0,1);
  int TempC = dht.readTemperature();
  lcd2.print(F("Temperature: ")); lcd2.print(TempC); lcd2.print(F("\xDF" "C"));

  // Dust sensor
  lcd2.setCursor(0,2);
  lcd2.print(F("Dust: "));
  lcd2.setCursor(6,2); 
  lcd2.print(ratio); 
  lcd2.setCursor(7,2);
  lcd2.print(F("/"));
  lcd2.setCursor(9,2);
  lcd2.print(concentration);
  
  
  // Anemometer
  detachInterrupt(interpin);                         
  aWSpeed=getAvgWindSpeed(culPulseTime,avgWindCount);  
  culPulseTime=0;                                    
  avgWindCount=0;                                     
  float aFreq=0;
  
  lcd2.setCursor(0,3);
  lcd2.print(F("Wind: ")); lcd2.print(aWSpeed); lcd2.print(F(" Km/h"));
  
  startt=true;                                        
  attachInterrupt(digitalPinToInterrupt(interpin),anemometerISR,RISING);
  dataTimer=millis();        
}

void Display_1(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Frequency: "); lcd.print(frequency); lcd.print(" Hz");
  
  lcd.setCursor(0,1);
  lcd.print("RPM: "); lcd.print(RPM); lcd.print(" rpm");

  lcd.setCursor(0,2);
  lcd.print("Direction: "); lcd.print(Direction);

  lcd.setCursor(0,3);
  lcd.print("Stress: "); lcd.print(Stress); lcd.print(" N/mm^2");
  }

void Display_2(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Torque: "); lcd.print(Torque, 4); lcd.print(" N.m");
  
  lcd.setCursor(0,1);
  lcd.print("Power: "); lcd.print(Power_Consumption, 4); lcd.print(" KW");
  
  lcd.setCursor(0,2);
  lcd.print("Direction: "); lcd.print(Direction);
  
  lcd.setCursor(0,3);
  lcd.print("Load Weight: "); lcd.print(Mass); lcd.print(" kg");
  }

void loop() {
  unsigned long currentTime = millis();

  // Load weighting
  int force = analogRead(load);
  Mass = map(force, 0, 205, 0, 5000);  // For simulation purposes max is set to 5 tons
  
  // Crane Stress
  int tension = analogRead(stress);
  Stress = map(tension, 0, 205, 0, 300);   // For simulation purposes highest stress is set to 300 N/mm^2
  
  // Dust sensor
  duration = pulseIn(dustpin, LOW); //Checks photovoltaic duration
  lowpulseoccupancy = lowpulseoccupancy + duration; //adds duration to our measure

  // PM10 values
  ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Percentage, values: 0 - 100
  concentration =  (2.5383 * pow(ratio, 2)) + (85.392 * ratio) - 5.8319; // Equations based on Samyoung DSM501A spec sheet - 
  
  // Speed control derivation and time delay
  potValue = analogRead(potentiometer);
  input_frequency = map(potValue, 0, 1023, 0, 50);   // Max frequency 50Hz
  t = 1000/(input_frequency*6);                      // Time delay of pulses (6 times delay PWM is accounted for)

  // Motor display formulas
  frequency = 1000/(t*6);                            // Frequency derivation using time delay
  Speed=(120*frequency)/4;                           // Synchronous speed (denominator is number of poles)
  RPM = Speed-(Speed/100);                           // Motor shaft actual speed or rotor speed (slip is 1%)
  angular_acc = (pi*RPM*frequency)/60;             // Angular acceleration of the motor
  angular_velocity = (RPM*2*pi)/60;                  // Angular velocity Rad/s
  Torque = (moment_of_inertia*angular_acc)/60;            // Torque of the motor
  Power_Consumption = (Torque*angular_velocity)/1000;       // Power Consumption of the motor

  Direction_Proccess();
  brake();
  Sensor_Display();
  SOS();
  
  // LCD Display of Crane/Motor
  if (currentTime - prevTime > 500){
    if (print_d1 == true){
      prevTime = currentTime;
      print_d1 = false;
      print_d2 = true;
      Display_1();
      }
    else if (print_d2 = true){
      prevTime = currentTime;
      print_d1 = true;
      print_d2 = false;
      Display_2();
      }
   }
}

// Anemometer timing
void anemometerISR(){
  unsigned long cTime=millis();                      
  if(!startt){                                       
    pulseTime=(float)(cTime-sTime)/1000;             
    culPulseTime+=pulseTime;                         
    avgWindCount++;                                  
  }
  sTime=cTime;                                       
  startt =false;                                     
}

// Wind speed Acquisition
float getAneFreq(float pTime){return(1/pTime);}      
float getWindMPH(float freq){return (freq*2.4);}   

float getAvgWindSpeed(float cPulse,int per){         
if(per)return getWindMPH(getAneFreq((float)(cPulse/per)));
  else return 0;
}

// Ultrasonic collision avoidance
void SonarSensor()
{
  delay(10);
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}

// SOS commands
void SOS(){
  lcd3.blink();
  SonarSensor();
  
  // Weather Status
  if (dht.readTemperature() < cold) { // Cold
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
  }
  else if (dht.readTemperature() >= hot) { // Hot
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  }
  else { // Crane weather good
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);
  }

  // Maximum rated stress is set to 200 N/mm^2
  if (Stress >= 200){
    lcd3.setCursor(0,0);
    lcd3.print("SOS Alert!!!");
    lcd3.setCursor(0,1);
    lcd3.print("Crane lift deformed");
    lcd3.setCursor(0,2);
    lcd3.print("Turn off crane");
    lcd3.setCursor(0,3);
    lcd3.print("Contact HQ");
  }
  
  // Sends an SOS if distance of an object is less than 20cm
  if (distance <= 20){
   lcd3.setCursor(0,1);
   lcd3.print("Near Collision");
  }

  // Windspeed that a crane can handle is 38 mpH anything higher can be catastrophic
  if (aWSpeed > 38){
    lcd3.setCursor(0,0);
    lcd3.print("SOS Alert!!!");
    lcd3.setCursor(0,1);
    lcd3.print("Weather bad");
    lcd3.setCursor(0,2);
    lcd3.print("Turn off crane");
    lcd3.setCursor(0,3);
    lcd3.print("Contact HQ");

    digitalWrite(11,HIGH);
    digitalWrite(12,LOW);
    digitalWrite(13,HIGH);
  }
  
  else if (aWSpeed < 38 && distance > 20 && Stress < 200){
    lcd3.clear();
  }
  
}

// Direction status of the crane motor
void Direction_Proccess(){
  
  if (!digitalRead(DirectionSw)){
    Direction = "Forward";
    }
    
  else{
    Direction = "Reverse";
    }
}

// Displaying braking or on and off switch
void brake(){

  if(digitalRead(manualbrake)){
    lcd.clear();
    lcd.setCursor(0,2);
    lcd.print("Manual Brake Active");
    }
    
  if(digitalRead(OffSw)) 
   {
    lcd.clear();
    lcd.setCursor(0,2);
    lcd.print("Motor is OFF");
   }
}
