/*
Class : Embedded Controller
Name : Gyeonheal An
Student No : 21900416
Date : 2023-09-15
*/

// State definition
#define S0  0
#define S1  1
#define S2  2

// Address number of output in array
#define PWM 0
#define LED 1

const int ledPin = 13;
const int pwmPin = 11;
const int btnPin = 3;
const int trigPin = 10;   // Trigger pin : PWM out (UltraSonic)
const int echoPin = 7;    // Echo pin : Interrupt in (UltraSonic)

float distance;
unsigned long duration;
unsigned char btn_input = 0;
unsigned char state = S0;
unsigned char nextstate = S0;
unsigned char input = 0;
unsigned char ledOut = LOW;
unsigned char fanOut = 0;
unsigned long currentMillis;
// State table definition
typedef struct {
	unsigned int next[4];       // nextstate = FSM[state].next[input]
	unsigned int led[4];        // output = FSM[state].out[input]
  unsigned int fan[4];
} State_t;

State_t FSM[3] = {
  { {S0, S0, S1, S1},{LOW, LOW, LOW, HIGH},{0, 0, 0, 128} },
  { {S1, S1, S2, S2},{HIGH, HIGH, HIGH, HIGH},{0, 128, 0, 255} },
  { {S2, S2, S0, S0}, {HIGH, HIGH, LOW, LOW}, {0, 255, 0, 0} }
};

void setup() {
  Serial.begin(9600);
	// initialize the LED pin as an output:
	pinMode(ledPin, OUTPUT);
	// initialize the pushbutton pin as an interrupt input:
	pinMode(btnPin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(btnPin), pressed, FALLING);
  // initialize the UnltraSonic sensor as an input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // initialize the DC motor pwm signal as an output
  pinMode(pwmPin, OUTPUT);
}

void loop() {
  // First, Output of current State. Then Update next state. Repeat
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  duration = pulseIn(echoPin, HIGH);
  distance = (float)duration / 58.0;

  // Has different input variable values depending on two inputs
  if(distance < 10){
    if(btn_input == 1)
      input = 3;
    else
      input = 1;
  }
  else{
    if(btn_input == 1)
      input = 2;
    else
      input = 0;
  }
	stateOutput();

  if(state != S0){
    currentMillis = millis();
    digitalWrite(ledPin,ledOut);
    while(1){
      if(millis()-currentMillis >= 1000){
        ledOut ^= 1;
        break;
      }
    }
  }
  else{
    digitalWrite(ledPin,ledOut);
  }

  currentMillis = millis();
	// 1. Output State
  digitalWrite(ledPin,ledOut);
  //digitalWrite(ledPin, ledOut);
  analogWrite(pwmPin, fanOut);
	// 2. Update State <-- Next State
	nextState();
  while(1){
    if(millis()-currentMillis>= 1000) break;
  }
  // print Serial monitor (Tera Term)
  Serial.println("---------------------------------------");
  Serial.printf("Distance : ");
  Serial.print(distance);
  Serial.println(" [cm]");
  Serial.printf("PWM duty ratio = ");
  Serial.println(fanOut);
}

void pressed() {
	btn_input = 1;
}

void nextState() {
	nextstate = FSM[state].next[input];
	state = nextstate;

	// Intialize button pressed
	btn_input = 0;
}

void stateOutput() {	
	ledOut = FSM[state].led[input];
  fanOut = FSM[state].fan[input];
}