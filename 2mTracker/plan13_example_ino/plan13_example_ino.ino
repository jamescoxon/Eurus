#include <Plan13.h>
#include <Time.h>

#define ONEPPM 1.0e-6
#define DEBUG false
Plan13 p13;

char * elements[1][3]={
             {"ISS (ZARYA)",
             "1 25544U 98067A   12146.08237655  .00018542  00000-0  26305-3 0  6222",
             "2 25544 051.6413 237.9310 0010868 357.8450 061.6602 15.56427442774416"},
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

  readElements(0);
  
  p13.calculate(); //crunch the numbers
  p13.printdata();
  Serial.println();
  delay(1000);
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


