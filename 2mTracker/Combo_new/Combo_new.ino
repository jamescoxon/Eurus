/*
* Eurus 70cm Tracker (434.200Mhz) - James Coxon jacoxon@googlemail.com
* Long duration, East to West, High Altitude balloon flight - based up on the code of PicoAtlas.
* Components - Arduino328, UBlox 6 GPS (Falcom FSA-03), RFM22b Radio
*
* Continous Transmission of RTTY.
*
* Latest code can be found: https://github.com/jamescoxon/Eurus
*
* GPS Code from jonsowman and Joey flight computer CUSF
* https://github.com/cuspaceflight/joey-m/tree/master/firmware

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
#include <util/crc16.h>

#include <Plan13.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <avr/pgmspace.h>    // needed for PROGMEM
#include "ax25modem.h"

static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
};

#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (50)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

// Data to be transmitted
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

#define ONEPPM 1.0e-6
#define DEBUG false
Plan13 p13;

//updated 15/6/12
char * elements[1][3] ={
             {"ISS (ZARYA)",
             "1 25544U 98067A   13199.72157074  .00009635  00000-0  17713-3 0  2129",
             "2 25544 051.6501 312.0646 0004963 195.4055 315.5742 15.49907351839598"}
 };


//Variables
int32_t lat = 514981000, lon = -530000, alt = 0;
uint8_t hour = 0, minute = 0, second = 0, month = 0, day = 0, lock = 0, sats = 0;
int GPSerror = 0, count = 1, n, gpsstatus, navmode = 0, battV = 0;
int elevation = 0, aprs_status = 0, aprs_attempts = 0, iss_in_view = 0;

uint8_t buf[60]; //GPS receive buffer
char superbuffer [80]; //Telem string buffer

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

//************Other Functions*****************

uint16_t gps_CRC16_checksum (char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;
 
	crc = 0xFFFF;
 
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++)
	{
		c = string[i];
		crc = _crc_xmodem_update (crc, c);
	}
 
	return crc;
}

void setupGPS() {
  
  // Taken from Project Swift (rather than the old way of sending ascii text)
  uint8_t setNMEAoff[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};
  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  
  delay(500);
  // Check and set the navigation mode (Airborne, 1G)
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  
  delay(500);
  
  //set GPS to Eco mode (reduces current by 4mA)
  uint8_t setEco[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x04, 0x1D, 0x85};
  sendUBX(setEco, sizeof(setEco)/sizeof(uint8_t));
  
}

/**
 * Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
 */
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
        uint8_t* ckb)
{
    *cka = 0;
    *ckb = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
        *cka += *data;
        *ckb += *cka;
        data++;
    }
}

/**
 * Verify the checksum for the given data and length.
 */
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
    uint8_t a, b;
    gps_ubx_checksum(data, len, &a, &b);
    if( a != *(data + len) || b != *(data + len + 1))
        return false;
    else
        return true;
}

/**
 * Get data from GPS, times out after 1 second.
 */
void gps_get_data()
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
/**
 * Check the navigation status to determine the quality of the
 * fix currently held by the receiver with a NAV-STATUS message.
 */
void gps_check_lock()
{
    GPSerror = 0;
    Serial.flush();
    // Construct the request to the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
        0x07, 0x16};
    sendUBX(request, 8);

    // Get the message back from the GPS
    gps_get_data();
    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
      GPSerror = 11;
    }
    if( buf[2] != 0x01 || buf[3] != 0x06 ) {
      GPSerror = 12;
    }

    // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
    if( !_gps_verify_checksum(&buf[2], 56) ) {
      GPSerror = 13;
    }
    
    if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
        lock = buf[16];
    else
        lock = 0;

    sats = buf[53];
    }
    else {
      lock = 0;
    }
}

/**
 * Poll the GPS for a position message then extract the useful
 * information from it - POSLLH.
 */
void gps_get_position()
{
    GPSerror = 0;
    Serial.flush();
    // Request a NAV-POSLLH message from the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
        0x0A};
    sendUBX(request, 8);
    
    // Get the message back from the GPS
    gps_get_data();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        GPSerror = 21;
    if( buf[2] != 0x01 || buf[3] != 0x02 )
        GPSerror = 22;
        
    if( !_gps_verify_checksum(&buf[2], 32) ) {
      GPSerror = 23;
    }
    
    if(GPSerror == 0) {
      // 4 bytes of longitude (1e-7)
      lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
          (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
      //lon /= 1000;
      
      // 4 bytes of latitude (1e-7)
      lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
          (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
      //lat /= 1000;
      
      // 4 bytes of altitude above MSL (mm)
      alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
          (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
      alt /= 1000;
    }

}

/**
 * Get the hour, minute and second from the GPS using the NAV-TIMEUTC
 * message.
 */
void gps_get_time()
{
    GPSerror = 0;
    Serial.flush();
    // Send a NAV-TIMEUTC message to the receiver
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
        0x22, 0x67};
     sendUBX(request, 8);

    // Get the message back from the GPS
    gps_get_data();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        GPSerror = 31;
    if( buf[2] != 0x01 || buf[3] != 0x21 )
        GPSerror = 32;

    if( !_gps_verify_checksum(&buf[2], 24) ) {
      GPSerror = 33;
    }
    
    if(GPSerror == 0) {
      if(hour > 23 || minute > 59 || second > 59)
      {
        GPSerror = 34;
      }
      else {
        month = buf[20];
        day = buf[21];
        hour = buf[22];
        minute = buf[23];
        second = buf[24];
      }
    }
}

/**
 * Verify that the uBlox 6 GPS receiver is set to the <1g airborne
 * navigaion mode.
 */
uint8_t gps_check_nav(void)
{
    uint8_t request[8] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00,
        0x2A, 0x84};
    sendUBX(request, 8);

    // Get the message back from the GPS
    gps_get_data();

    // Verify sync and header bytes
    if( buf[0] != 0xB5 || buf[1] != 0x62 ){
      GPSerror = 41;
    }
    if( buf[2] != 0x06 || buf[3] != 0x24 ){
      GPSerror = 42;
    }
    // Check 40 bytes of message checksum
    if( !_gps_verify_checksum(&buf[2], 40) ) {
      GPSerror = 43;
    }

    // Return the navigation mode and let the caller analyse it
    navmode = buf[8];
}

double getElement(char *gstr, int gstart, int gstop)
{
  double retval;
  int    k, glength;
  char   gestr[80];

  glength = gstop - gstart + 1;

  for (k = 0; k <= glength; k++)
  {
     gestr[k] = gstr[gstart+k-1];
  }

  gestr[glength] = '\0';
  retval = atof(gestr);
  return(retval);
}

void readElements(int x)//order in the array above
{
 // for example ...
 // char line1[] = "1 28375U 04025K   09232.55636497 -.00000001  00000-0 12469-4 0   4653";
 // char line2[] = "2 28375 098.0531 238.4104 0083652 290.6047 068.6188 14.40649734270229";

        p13.setElements(getElement(elements[x][1],19,20) + 2000, getElement(elements[x][1],21,32), getElement(elements[x][2],9,16), 
         getElement(elements[x][2],18,25), getElement(elements[x][2],27,33) * 1.0e-7, getElement(elements[x][2],35,42), getElement(elements[x][2],44,51), getElement(elements[x][2],53,63), 
         getElement(elements[x][1],34,43), (getElement(elements[x][2],64,68) + ONEPPM), 0); 
 }
 
  
 void tx_aprs()
{
	char slat[5];
	char slng[5];
	char stlm[9];
	static uint16_t seq = 0;
        double aprs_lat, aprs_lon;
	
	/* Convert the UBLOX-style coordinates to
	 * the APRS compressed format */
	aprs_lat = 900000000 - lat;
	aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
        Serial.print(aprs_lat);
	Serial.print(" ");
	aprs_lon = 900000000 + lon / 2;
	aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
        Serial.println(aprs_lon);
	
	alt = alt * 32808 / 10000;
	
	/* Construct the compressed telemetry format */
	ax25_base91enc(stlm + 0, 2, seq);
	
        if(iss_in_view == 1){
	  ax25_frame(
		APRS_CALLSIGN, APRS_SSID,
		"APRS", 0,

                //ISS
                "ARISS", 0, "WIDE2", 1,
                
                //GROUND_APRS
                //"WIDE2", 1, 0, 0,
		
		"!/%s%sO   /A=%06ld|%s|%d/%d/%d",
		ax25_base91enc(slat, 4, aprs_lat),
		ax25_base91enc(slng, 4, aprs_lon),
		alt, stlm, count, aprs_attempts, elevation
	  );
        }
        else {
          ax25_frame(
		APRS_CALLSIGN, APRS_SSID,
		"APRS", 0,

                //ISS
                //"ARISS", 0, "WIDE2", 1,
                
                //GROUND_APRS
                "WIDE2", 1, 0, 0,
		
		"!/%s%sO   /A=%06ld|%s|%d/%d/%d",
		ax25_base91enc(slat, 4, aprs_lat),
		ax25_base91enc(slng, 4, aprs_lon),
		alt, stlm, count, aprs_attempts, elevation
	  );
        }
        
	
	seq++;
}

ISR(TIMER2_OVF_vect)
{
	static uint16_t phase  = 0;
	static uint16_t step   = PHASE_DELTA_1200;
	static uint16_t sample = 0;
	static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
	static uint8_t byte;
	static uint8_t bit     = 7;
	static int8_t bc       = 0;
	
	/* Update the PWM output */
	OCR2A = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
	phase += step;
	
	if(++sample < SAMPLES_PER_BAUD) return;
	sample = 0;
	
	/* Zero-bit insertion */
	if(bc == 5)
	{
		step ^= PHASE_DELTA_XOR;
		bc = 0;
		return;
	}
	
	/* Load the next byte */
	if(++bit == 8)
	{
		bit = 0;
		
		if(rest > REST_BYTES || !_txlen)
		{
			if(!--rest)
			{
				/* Disable radio and interrupt */
				//PORTA &= ~TXENABLE;
				TIMSK2 &= ~_BV(TOIE2);
				
				/* Prepare state for next run */
				phase = sample = 0;
				step  = PHASE_DELTA_1200;
				rest  = PREAMBLE_BYTES + REST_BYTES;
				bit   = 7;
				bc    = 0;
				
				return;
			}
			
			/* Rest period, transmit ax.25 header */
			byte = 0x7E;
			bc = -1;
		}
		else
		{
			/* Read the next byte from memory */
			byte = *(_txbuf++);
			if(!--_txlen) rest = REST_BYTES + 2;
			if(bc < 0) bc = 0;
		}
	}
	
	/* Find the next bit */
	if(byte & 1)
	{
		/* 1: Output frequency stays the same */
		if(bc >= 0) bc++;
	}
	else
	{
		/* 0: Toggle the output frequency */
		step ^= PHASE_DELTA_XOR;
		if(bc >= 0) bc = 0;
	}
	
	byte >>= 1;
}

void ax25_init(void)
{
	/* Fast PWM mode, non-inverting output on OC2A */
	TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS20);
	
	/* Make sure radio is not enabled */
	//PORTA &= ~TXENABLE;
	
	/* Enable pins for output (Port A pin 4 and Port D pin 7) */
	//DDRA |= TXENABLE;
	pinMode(11, OUTPUT);
}

static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid)
{
	char i;
	for(i = 0; i < 6; i++)
	{
		if(*callsign) *(s++) = *(callsign++) << 1;
		else *(s++) = ' ' << 1;
	}
	  *(s++) = ('0' + ssid) << 1;
        
	return(s);
}

void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid,
		char *path1, char ttl1, char *path2, char ttl2, char *data, ...)
{
	static uint8_t frame[100];
	uint8_t *s;
	uint16_t x;
	va_list va;
	
	va_start(va, data);
	
	/* Pause while there is still data transmitting */
	while(_txlen);
	
	/* Write in the callsigns and paths */
	s = _ax25_callsign(frame, dcallsign, dssid);
	s = _ax25_callsign(s, scallsign, sssid);
	if(path1) s = _ax25_callsign(s, path1, ttl1);
	if(path2) s = _ax25_callsign(s, path2, ttl2);
	
	/* Mark the end of the callsigns */
	s[-1] |= 1;
	
	*(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
	*(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */
	
	vsnprintf((char *) s, 100 - (s - frame) - 2, data, va);
	va_end(va);
	
	/* Calculate and append the checksum */
	for(x = 0xFFFF, s = frame; *s; s++)
		x = _crc_ccitt_update(x, *s);
	
	*(s++) = ~(x & 0xFF);
	*(s++) = ~((x >> 8) & 0xFF);
	
	/* Point the interrupt at the data to be transmit */
	_txbuf = frame;
	_txlen = s - frame;
	
	/* Enable the timer and key the radio */
	TIMSK2 |= _BV(TOIE2);
	//PORTA |= TXENABLE;
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
	/* Creates a Base-91 representation of the value in v in the string */
	/* pointed to by s, n-characters long. String length should be n+1. */
	
	for(s += n, *s = '\0'; n; n--)
	{
		*(--s) = v % 91 + 33;
		v /= 91;
	}
	
	return(s);
}

void send_APRS() {
  
    tone(11, 1200, 100);
    digitalWrite(11,LOW);
    delay(2000);
    ax25_init();
    delay(1000);
    tx_aprs();
    delay(2000);
    tone(11, 1200, 100);
    digitalWrite(11,LOW);
    
}

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  analogReference(DEFAULT);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  
  delay(1000);
  setupGPS();
  digitalWrite(13, HIGH);
  //ax25_init();
}

void loop() {
  count++;
  
  gps_check_nav();
  
  
  if(navmode != 6) {
    setupGPS();
  }
  
  
  gps_check_lock();
  gps_get_position();
  gps_get_time();

  //if ((lock == 3) && (count % 2 == 0)){
    digitalWrite(13, HIGH);
    //First setup plan13 stuff
    p13.setFrequency(437550000, 437550000);//ISS 70cm beacon frequency
    p13.setLocation(((double)lon / 10000000.0) , ((double)lat / 10000000.0), alt); //THIS NEEDS TO BE LON, LAT
    p13.setTime(2013, month, day, hour, minute, second);
    
    //ISS
    readElements(0);
    p13.calculate(); //crunch the numbers
    elevation = (int)p13.getElevation();
    
    if (elevation >= 5){
      aprs_status = 1;
      //Transmit APRS data now
      iss_in_view = 1;
      send_APRS();
      aprs_attempts++;
      delay(10000);
    }
    else {
      aprs_status = 0;
    }
      
    //delay(1000);
    digitalWrite(13, LOW);
  //}
  
  battV = analogRead(0);
  
  
  n=sprintf (superbuffer, "$$EURUS,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%d", count, hour, minute, second, lat, lon, alt, sats, lock, navmode, battV, elevation, aprs_status, aprs_attempts);
  n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
  
  //rtty_txstring(superbuffer);

  Serial.println(superbuffer);
  
  if (count % 40 == 0){
      //Transmit APRS data now
      iss_in_view = 0;
      send_APRS();
      aprs_attempts++;
  }
  if (count % 50 == 0){
    
    //Send commands to GPS
    setupGPS();
  }

}
