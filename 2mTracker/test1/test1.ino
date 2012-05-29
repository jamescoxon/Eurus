#include <Plan13.h>
#include <Time.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "ax25modem.h"

static const uint8_t _sine_table[] = {
#include "sine_table.h"
};

#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (25)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

/* Data to be transmitted */
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

#define ONEPPM 1.0e-6
#define DEBUG false
Plan13 p13;

char * elements[1][3]={
             {"ISS (ZARYA)",
             "1 25544U 98067A   12146.08237655  .00018542  00000-0  26305-3 0  6222",
             "2 25544 051.6413 237.9310 0010868 357.8450 061.6602 15.56427442774416"}
 };
 
int32_t lat = 512765000, lon = 10640000, alt = 20000;
double elevation;
 
void setup () {
  Serial.begin(38400);
  setTime((1338273957+60)); 
  
  //setTime(hr,min,sec,day,month,yr);
  p13.setFrequency(145825000, 145825000);//ISS frequency
  p13.setLocation(1.0760, 51.2760, 20); // Canterbury THIS NEEDS TO BE LON, LAT
  
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  ax25_init();
  delay(5000);
  send_APRS();
  
}
void loop() { 
  lat = 512765000;
  lon = 10640000;
  alt = 20000;
  time_t t = now();
  Serial.print(year(t)); Serial.print(month(t)); Serial.print(day(t)); Serial.print(hour(t));Serial.print(minute(t));Serial.println(second(t));
  p13.setTime(year(t), month(t), day(t), hour(t), minute(t), second(t)); //Oct 1, 2009 19:05:00 UTC

  //ISS
  readElements(0);
  p13.calculate(); //crunch the numbers
  p13.printdata();
  Serial.println();
  elevation = p13.getElevation();
  Serial.print("El: ");
  Serial.println(elevation);
  if (elevation >= 10){
    send_APRS();
  }
  else{
    Serial.println("Not in view");
  }
  delay(60000);
}

void send_APRS() {
    Serial.print("Sending APRS");
    digitalWrite(5, HIGH);
    delay(1000);
    tx_aprs();
    delay(1000);
    digitalWrite(5, LOW);
    Serial.println(" Done");
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
	
	/* Convert the UBLOX-style coordinates to
	 * the APRS compressed format */
	lat = 900000000 - lat;
	lat = lat / 26 - lat / 2710 + lat / 15384615;
        Serial.print(lat);
	Serial.print(" ");
	lon = 900000000 + lon / 2;
	lon = lon / 26 - lon / 2710 + lon / 15384615;
        Serial.println(lon);
	
	alt = alt / 1000 * 32808 / 10000;
	
	/* Construct the compressed telemetry format */
	ax25_base91enc(stlm + 0, 2, seq);
	
	ax25_frame(
		APRS_CALLSIGN, APRS_SSID,
		"ARISS", 0,
		//0, 0, 0, 0,
                "WIDE1", 1, 0,0,
		//"WIDE2", 1,
		"!/%s%sO   /A=%06ld|%s|",
		ax25_base91enc(slat, 4, lat),
		ax25_base91enc(slng, 4, lon),
		alt, stlm
	);
	
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
	OCR2A = _sine_table[(phase >> 7) & 0x1FF];
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

