const int ledPin = 13;  // LED pin
const int soundPin = 5; // Sound sensor pin

float measure;          // the value measured from sound sensor.
int ledState = LOW;

void setup() {
  // Initialize the LED pin as an output.
  pinMode(ledPin, OUTPUT);

  // Initialize the serial port.
  Serial.begin(9600);
}

void loop() {
  // the measured value needs to be transformed to voltage unit.
  // mapping(0 ~ 3.3V -> 0 ~ 1023)
  // [mV] (0 ~ 1023 -> 0 ~ 3300[mV])
  measure = analogRead(soundPin);
  measure = measure * 3300 / 1024;
    
  // print measured value
  Serial.print("measure = ");
  Serial.print(measure);
  Serial.println(" [mV}");
  
  // Change the LED state by measure threshold.
  if(measure > 500) ledState = HIGH;
  else              ledState = LOW;
  
  digitalWrite(ledPin, ledState);
  delay(200);
}
