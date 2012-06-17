uint8_t buf[60]; //SPoT receive buffer
int count = 1, msg_sent = 0;

void spotSetup(){
  digitalWrite(6, HIGH);
  delay(1000);
  digitalWrite(7, LOW);
  delay(4000);
  pinMode(7, INPUT); //Release on button
  //digitalWrite(7, HIGH);
  delay(4000);
}

// Send a byte array of UBX protocol to the SPoT
void sendBytes(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

/**
 * Get data from SPoT, times out after 1 second.
 */
void spot_get_data()
{
    int i = 0;
    unsigned long startTime = millis();
    while (1) {
    // Make sure data is available to read
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
    // Timeout if no valid response in 1 seconds
    if (millis() - startTime > 1000) {
      break;
    }
    }
}

void spot_unitID(){
    Serial.flush();
    // Construct the request to the GPS
    uint8_t request[3] = {0xAA, 0x03, 0x01};
    sendBytes(request, 3);
    spot_get_data();
    
    print_buffer(20);
}

void spot_status(){
    Serial.flush();
    // Construct the request to the GPS
    uint8_t request[3] = {0xAA, 0x03, 0x52};
    sendBytes(request, 3);
    spot_get_data();
    
    print_buffer(43);
    if(buf[0] == 0xAA) { //We have found the header!
      Serial.print("Status: ");
      Serial.print(buf[7]);
      Serial.print(" ");
      
      Serial.print("Sats: ");
      Serial.print(buf[31]);
      Serial.print(" ");
      
      int timeToNextCheckin = (buf[11] << 8) | buf[12];
      Serial.print("Next Checkin: ");
      Serial.println(timeToNextCheckin);
      
    }
}

void spot_send(){
  
    Serial.flush();
    // Construct the request to the GPS
    uint8_t request[12] = {0xAA, 0x0C, 0x26, 0x01, 0x00, 0x01, 0x00, 0x01, 0x54, 0x65, 0x73, 0x74};
    sendBytes(request, 12);
    spot_get_data();
    
    print_buffer(43);
    
    msg_sent = 1;
}

void print_buffer(int z){
    int i;
    for (i = 0; i <= z; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  pinMode(6, OUTPUT);
  delay(1000);
  spotSetup();
  delay(1000);
  spot_unitID();

}

void loop() {
  count++;
  if(count == 10){
    Serial.println("Sending Msg");
    spot_send();
  }
  delay(5000);
  spot_status();
  
  
}
