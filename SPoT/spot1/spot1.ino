/*
* Eurus SPoT Tracker- James Coxon jacoxon@googlemail.com
* Long duration, East to West, High Altitude balloon flight.
* Components - Arduino328, Sparkfun SatUplink Board and a SPoT Connect
*
* Regular Sat Simplex Comms with powersaving
*
* Latest code can be found: https://github.com/jamescoxon/Eurus
*
* RTC lib can be found here: https://github.com/jamescoxon/RTClib

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <avr/sleep.h>
#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>
#include <RTC_DS3234.h>

// Create an RTC instance, using the chip select pin it's connected to
RTC_DS3234 RTC(10);

DateTime now;

uint8_t buf[60]; //SPoT receive buffer
int count = 1, msg_sent = 0, spot_stat = 0, timeToNextCheckin = 0;

void spotSetup(){
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH); //Start SPoT
  //Start from the beginning, turn off the regulator to ensure that we've fully powered down
  digitalWrite(6, LOW); // Power off Regulators
  delay(5000);
  digitalWrite(6, HIGH); // Power on Regulators
  delay(5000);
  
  digitalWrite(7, LOW); //Start SPoT
  delay(4000);
  digitalWrite(7, HIGH); //Start SPoT
  //pinMode(7, INPUT); //Release on button
  delay(4000); //Wait for SPoT to power-up
}

// Send a byte array to the SPoT
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
    uint8_t request[3] = {0xAA, 0x03, 0x01};
    sendBytes(request, 3);
    spot_get_data();
    
    print_buffer(20);
}

void spot_status(){
    Serial.flush();
    uint8_t request[3] = {0xAA, 0x03, 0x52};
    sendBytes(request, 3);
    spot_get_data();
    
    print_buffer(43);
    if(buf[0] == 0xAA) { //We have found the header!
      spot_stat = buf[7];
      Serial.print("Status: ");
      Serial.print(spot_stat);
      Serial.print(" ");
      
      Serial.print("Sats: ");
      Serial.print(buf[31]);
      Serial.print(" ");
      
      timeToNextCheckin = (buf[11] << 8) | buf[12];
      Serial.print("Next Checkin: ");
      Serial.println(timeToNextCheckin);
      
    }
}

void spot_send(){
  
    Serial.flush();
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

void wakeUpNow()        // here the interrupt is handled after wakeup
{  
  RTC.reset_alarm(); //Need to reset the alarm otherwise it'll keep triggering
}

void sleepNow()         // here we put the arduino to sleep
{
    /* The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     * In all but the IDLE sleep modes only LOW can be used.
     */  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 

    attachInterrupt(1,wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets LOW 

    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    detachInterrupt(1);      // disables interrupt 0 on pin 2 so the 
                             // wakeUpNow code will not be executed 
                             // during normal running time.
}

void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  pinMode(6, OUTPUT);
  SPI.begin();
  
  //Start up the RTC
  RTC.begin();  
  
  RTC.adjust(DateTime(__DATE__, __TIME__));
  
  //Set up alarms
  RTC.set_alarm(2, DateTime(__DATE__, __TIME__), 0x07);
  
  RTC.setup(0,1);
  delay(1000);

}

void loop() {
  
  //Setup SPoT
  spotSetup();
  delay(25000);
  spot_status();
  //Check that SPoT is communicating with us
  spot_status();
  delay(1000);
  if(buf[0] == 0xAA && buf[7] == 0x00){
    
    Serial.println("Sending Msg");
    //1) Send Message
    spot_send();
    delay(5000);
    spot_status();
    //2) Setup GPS to Airborne Mode via software serial
    
    //3) Wait for transmission to occur
    int timeToNextCheckin_old = 999;
    while(timeToNextCheckin > 0){
      
       delay(10000);
       spot_status();
       
       //Sometimes SPoT has a habit of freezing, this is to check if its not responding, but breaking out of the 
       // while loop we then turn off the SPoT and try again later.
       if(timeToNextCheckin >= timeToNextCheckin_old){
         Serial.println("Breaking loop");
         break;
       }
       else {
       timeToNextCheckin_old = timeToNextCheckin;
      }
    }
    //4) Power down SPoT
    digitalWrite(7, LOW); // Hold down ON/OFF button
    delay(3000);
    digitalWrite(7, HIGH);
    delay(500);
    digitalWrite(6, LOW); // Power off Regulators
    //SPoT should now be completely off
    
    //5) Set RTC alarm for next wake up
  }
    //6) Go to sleep
    Serial.println("Going to sleep");
    delay(1000);
    sleepNow();
  
}
