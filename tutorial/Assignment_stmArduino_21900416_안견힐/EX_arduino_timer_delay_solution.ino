unsigned int cnt = 0;
unsigned long beginTime, endTime;

void setup() {
  // Initialize Serial port
  Serial.begin(9600);
  Serial.println(sizeof(unsigned int));
  Serial.print("Program START\r\n");
}

void loop() {
  cnt = 0;

  // call delaycnt() function
  delaycnt(256);
  
  // Check how much time counting takes.
  Serial.printf("Counting %d takes %d [us]\r\n", cnt, endTime - beginTime);
  delay(500);
}

void delaycnt(unsigned int delayCnt){
  // check current time [us]
  beginTime = micros();

  // counting until delayCnt value.
  while(cnt < delayCnt){
    cnt++;
  }
  
  // check current time [us]
  endTime = micros();
}
