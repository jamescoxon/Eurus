#include <Plan13.h>
#include <Time.h>
#include "ax25modem.h"

#define ONEPPM 1.0e-6
#define DEBUG false
Plan13 p13;

char * elements[2][3]={
             {"ISS (ZARYA)",
             "1 25544U 98067A   12146.08237655  .00018542  00000-0  26305-3 0  6222",
             "2 25544 051.6413 237.9310 0010868 357.8450 061.6602 15.56427442774416"},
             {"NO-44",
             "1 26931U 01043C   12147.56621725  .00000202  00000-0  11064-3 0   482",
             "2 26931  67.0505 176.0650 0007706 266.8517  93.1692 14.29798094556151"}
 };
 
void setup () {
  Serial.begin(38400);
  setTime((1338129814+60)); 
  
  //setTime(hr,min,sec,day,month,yr);
  p13.setFrequency(145825000, 145825000);//ISS frequency
  p13.setLocation(51.2760, 1.0760, 20); // Canterbury
  
}
void loop() { 
  time_t t = now();
  Serial.print(year(t)); Serial.print(month(t)); Serial.print(day(t)); Serial.print(hour(t));Serial.print(minute(t));Serial.println(second(t));
  p13.setTime(year(t), month(t), day(t), hour(t), minute(t), second(t)); //Oct 1, 2009 19:05:00 UTC

  //ISS
  readElements(0);
  p13.calculate(); //crunch the numbers
  p13.printdata();
  Serial.println();
  
  //PCSAT
  readElements(1);
  p13.calculate(); //crunch the numbers
  p13.printdata();
  Serial.println();
  
  delay(10000);
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
 
 void tx_aprs(int32_t lat, int32_t lon, int32_t alt)
{
	char slat[5];
	char slng[5];
	char stlm[9];
	static uint16_t seq = 0;
	
	/* Convert the UBLOX-style coordinates to
	 * the APRS compressed format */
	lat = 900000000 - lat;
	lat = lat / 26 - lat / 2710 + lat / 15384615;
	
	lon = 900000000 + lon / 2;
	lon = lon / 26 - lon / 2710 + lon / 15384615;
	
	alt = alt / 1000 * 32808 / 10000;
	
	/* Construct the compressed telemetry format */
	ax25_base91enc(stlm + 0, 2, seq);
	
	ax25_frame(
		APRS_CALLSIGN, APRS_SSID,
		"APRS", 0,
		0, 0, 0, 0,
		//"WIDE1", 1,
		//"WIDE2", 1,
		"!/%s%sO   /A=%06ld|%s|",
		ax25_base91enc(slat, 4, lat),
		ax25_base91enc(slng, 4, lon),
		alt, stlm
	);
	
	if(seq % 60 == 0)
	{
		char s[10];
		
		/* Make up the callsign */
		strncpy_P(s, PSTR(APRS_CALLSIGN), 7);
		if(APRS_SSID) snprintf(s + strlen(s), 4, "-%i", APRS_SSID);
		
		/* Transmit telemetry definitions */
		ax25_frame(
			APRS_CALLSIGN, APRS_SSID,
			"APRS", 0,
			0, 0, 0, 0,
			":%-9s:PARM.Battery", s
		);
		ax25_frame(
			APRS_CALLSIGN, APRS_SSID,
			"APRS", 0,
			0, 0, 0, 0,
			":%-9s:UNIT.mV", s
		);
	}
	
	seq++;
}


