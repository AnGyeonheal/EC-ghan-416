const int pwmPin = 11;  // PWM pin
const int btnPin = 3;   // button pin
const int dirPin = 9;   // direction pin

int cnt = 0;
int dir = LOW;

void setup() {
  // Initialize PWM pin as an output:
  pinMode(pwmPin, OUTPUT);

  // Initialize the direction pin as an output:
  pinMode(dirPin, OUTPUT);
  
  // Initialize the push button pin as an interrupt input:
  pinMode(btnPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btnPin), motorOperation, FALLING);
}

void loop() {
  // Write the direction and speed command to each pins.
  // Hint: speed value can be expressed by 'cnt' variable.
  // speed value : 0 ~ 255
  digitalWrite(dirPin, dir);
  analogWrite(pwmPin, 255/6 * cnt);  

}

void motorOperation(){
  // Use 'cnt' and 'dir' variables
  cnt++;
  if(cnt > 4){
    cnt = 0;
    dir = !dir;
  }
}
