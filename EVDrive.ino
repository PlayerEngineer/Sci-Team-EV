const byte HALL = 3;
const byte BUTTON = 12;
const byte LIGHT = 13;

//servo things
#include <Servo.h>
Servo leftServo;
const byte LSERVO = 9;

Servo rightServo;
const byte RSERVO = 10;


//motor driver pins
const byte PWMA = 5; //old ENA
const byte AIN1 = 4; //old IN1
const byte AIN2 = 7; //old IN2
const byte BIN1 = 8; //old IN3
const byte BIN2 = 11; //old IN4
const byte PWMB = 6; //old ENB

bool running = false;
volatile int hits = 0;
unsigned long startTime;
unsigned long endTime;

const int RADIUS = 1.125; //this is wheel radius (inches)
const float stopDist = 90; //distance to stop driving (inches)



void setup() {
  Serial.begin(9600);
  
  // servos
  leftServo.attach(LSERVO);
  rightServo.attach(RSERVO);
  leftServo.write(15);
  rightServo.write(165);
  delay(500);
  leftServo.detach();
  rightServo.detach();

  //button and light
  pinMode(BUTTON, INPUT_PULLUP);


  
  pinMode(LIGHT, OUTPUT);

  //hall effect
  pinMode(HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL), onMagnet, FALLING);
  
  Serial.println("Ready");
}

void onMagnet() {
  if(running)
    hits++;
}

void brakeServos(){
  leftServo.attach(LSERVO);
  rightServo.attach(RSERVO);

  leftServo.write(90);
  rightServo.write(90);

  delay(300);
  leftServo.detach();
  rightServo.detach();
}

void releaseServos(){
  leftServo.attach(LSERVO);
  rightServo.attach(RSERVO);

  for(int i = 0; i <= 75; i++){
    leftServo.write(90-i);
    rightServo.write(90+i);
    delay(20);

  }

  leftServo.write(15);
  rightServo.write(165);

  delay(300);
  leftServo.detach();
  rightServo.detach();
}

void loop() {
  buttonScan();
  distanceStopper();


}

void distanceStopper(){
  float distance = (float)hits*1.21196*((TWO_PI*RADIUS)/4)+2.41056;

  if(running){
  Serial.print("Hits: ");
  Serial.print(hits);

  Serial.print(" | Distance: ");
  Serial.println(distance);
}
  if((distance >= stopDist) && running)
    stop();
}

void start(){
  Serial.println("Started!");
  running = true;
  hits = 0;

  // releaseServos();
  leftServo.write(15);
  rightServo.write(165);
  delay(300);
  leftServo.detach();
  rightServo.detach();

  digitalWrite(LIGHT, HIGH);
  //startMotors(150);

  for(int i = 0; i<150; i+=5){
    startMotors(i);
    delay(20);
  }

  startTime = millis();
}

void stop(){
  stopMotors();
  running = false;
  endTime = millis();
  

  float tempdist = (float)hits*1.21196*((2*PI*RADIUS)/4) - 2.41056;

  Serial.println();
  Serial.print("Stopped! Average Speed: ");
  Serial.print((tempdist/(endTime - startTime)) * 56.8182);
  Serial.print(" mph");

  Serial.println();
  Serial.print("Supply Voltage: ");
  Serial.print(getSupplyVoltage());
  Serial.print(" V");

  brakeServos();
  delay(2000);
  releaseServos();

  digitalWrite(LIGHT, LOW);
  
}

void buttonScan(){

  static unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  static int lastButtonState = LOW;
  static int buttonState;

  unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


  int sensorVal = digitalRead(BUTTON);
  //Serial.println(sensorVal);

  if (sensorVal != lastButtonState){
    lastDebounceTime = millis();
  }

  if(((millis() - lastDebounceTime) > debounceDelay) && sensorVal != buttonState){

    buttonState = sensorVal;

    if (buttonState == HIGH && !running) {
      //not pushed
      digitalWrite(LIGHT, LOW);
    } else if(!running) {
      //button pushed
      start();

    } else if (running && buttonState == LOW){
      //button pushed and is running
      stop();

    }
  }
  
  lastButtonState = sensorVal;
}

void startMotors(int speed){
  digitalWrite(AIN1, LOW); 
  digitalWrite(AIN2, HIGH);  
  analogWrite(PWMA, speed);
  digitalWrite(BIN1, HIGH); 
  digitalWrite(BIN2, LOW);  
  analogWrite(PWMB, speed);
}


void stopMotors(){
  digitalWrite(AIN1, HIGH);  
  digitalWrite(AIN2, HIGH);  
  analogWrite(PWMA, 0);
  digitalWrite(BIN1, HIGH);  
  digitalWrite(BIN2, HIGH);  
  analogWrite(PWMB, 0);
}

//i did not write this
float getSupplyVoltage() {
  // Read 1.1V reference against AVcc
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ADCSRB &= ~_BV(MUX5);   // Clear MUX5 for ATmega2560
  #else
    return 0.0; // Unsupported board
  #endif  

  delay(2);                         // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);              // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // Wait for completion

  uint8_t low  = ADCL;              // Read low byte first
  uint8_t high = ADCH;              // Read high byte
  long rawValue = (high << 8) | low;

  // 1125300 = 1.1V * 1023 * 1000
  float millivolts = 1125300.0 / rawValue; 
  return millivolts / 1000.0;       // Return volts
}