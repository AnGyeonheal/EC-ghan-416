const int irSensorPin = 4;  // the number of IR sensor pin

int detectedState = HIGH;

void setup() {
  // Initialize the IR sensor pin as an input.
  pinMode(irSensorPin, INPUT);

  // Initialize the serial port.
  Serial.begin(9600);
}

void loop() {
  // Read value from IR sensor
  detectedState = digitalRead(irSensorPin);
  

  // print warning
  if(detectedState == LOW){
    Serial.println("Warning!");
    delay(1000);
  }
  Serial.println(detectedState);
}
