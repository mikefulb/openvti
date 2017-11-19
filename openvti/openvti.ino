/*

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


This is VERSION 4 - it uses the hardware serial port of the Arduino!!

ChangeLog

Version 1 
- initial using SoftwareSerial for GPS comms

Version 2 
- switch to HardwareSerial
- implemented field detection using VSYNC/HSYNC outputs from MAX7456 using external 555 or ATTINY13

Version 3
- use "arduino only" solution for field detection
- final version has working limovie time insertion

Version 4
- start with working limovie compatible time insertion
- move LED handler into VSYNC pin change interrupt handler
- make display more sparse to provide more area for stars!

VTI for following setup:

TinySine Video Overlay using Max-7456
Gowoops GPS Module U-blox NEO-6M
  (Amazon: https://www.amazon.com/Gowoops-Module-Antenna-Arduino-Microcomputer/dp/B01MRNN3YZ)

GPS uses +5V supply but 3.3V logic levels so be careful connecting to 5V Arduino

Configure ublox GPS to the following:

- output GPGGA and PUBX,04 NMEA messages
- Timepulse set to UTC time (defaults to GPS)


LIBRARY NOTES:

I decided to start including library files in sketch directory - this way I can keep it self contained

TinyGPS++
MAX7456

I included zip files as well for original library
      
*/



// Included Libraries //////////////////////////////////////////////////////////

#include "pinconfig.h"
#include <SPI.h>
#include "MAX7456.h"
#include "TinyGPS++.h"

#define VERSION  3

// Global Macros ///////////////////////////////////////////////////////////////

#define LEAPSEC_UPDATE_MSEC  10000
#define LOCCYCLE_UPDATE_MSEC 2500

// Global Constants ////////////////////////////////////////////////////////////
const unsigned long gpsBaud = 9600;                                              
  
// we are going to use the hardware serial port for gps
#define gpsSerial Serial
  
MAX7456 OSD( osdChipSelect );

TinyGPSPlus gps;

// custom sentences
TinyGPSCustom  datum(gps, "GPDTM", 1);
TinyGPSCustom  leapsec(gps, "PUBX", 6);
TinyGPSCustom  fixvalid(gps, "GPRMC", 2);

// millis of last PPS pulse
unsigned long lastpps_millis;

// millis of last VSYNC
volatile unsigned long lastvsync_millis;

// set true each pulse
bool newpps=false;

// value to set Timer1 to
int timer1_preset;

  // pulse control
#define MAX_PULSE  10
int npulse=0;  // number of steps in pulse sequence
int pulseseqlen[MAX_PULSE]; // length of each pulse (multiples of 10msec interrupt interval)
bool pulseseqval[MAX_PULSE]; // pulse state ON or OFF

// configure LED timer
// 65410 works out to about 2msec
#define TIMER1_PRESET 65410
#define TIMER1_MSEC   2

// field parity 0 = even, 1 = odd
  volatile int lastfield=0;
  volatile int curfield=0; 
  volatile int fieldnum = 0;
  volatile long totalfields = 0;

// double pulse on 00 sec?
//#define DOUBLE_PULSE

// location of UI elements
#define TOP_ROW 0
#define BOTTOM_ROW  12

// solid character that blinks each PPS we receive
#define PPSCHAR_COL 1
#define PPSCHAR_ROW (BOTTOM_ROW)

#define VSYNCCHAR_COL 0
#define VSYNCCHAR_ROW (BOTTOM_ROW)

#define FIELD1TS_COL 12
#define FIELD1TS_ROW BOTTOM_ROW

#define FIELD2TS_COL 17
#define FIELD2TS_ROW BOTTOM_ROW

#define FIELDTOT_COL 22
#define FIELDTOT_ROW BOTTOM_ROW

#define FIX_ROW TOP_ROW
#define FIX_COL 1

#define CYCLE_ROW TOP_ROW
#define CYCLE_COL 4

#define TIME_ROW BOTTOM_ROW
#define TIME_COL 3

// pixel offset of display    
#define OSD_X_OFFSET -4
#define OSD_Y_OFFSET  0

#ifdef DEBUG_PIN
// OPTIONAL - enable define to use DEBUG_PIN to debug video field decoding
//#define VSYNC_DEBUG_PIN
#endif

// VSYNC pin change interrupt handler
VSYNC_PINCHANGE_ISR()
{
  byte v;

  //read hsync first thing
  v = HSYNC_READ();

  lastvsync_millis = millis();

  // FIXME need to somehow use pin number (arduino) defines instead of hard coded!
  // pin change triggers on falling and rising so have to filter
  if (!VSYNC_READ())
  {
    if (v)
    {
      // we should set curfield = 0
      // but maybe we just missed HSYNC pulse
      // compare to last value and make sure we don't get a duplicate
      // if we didn't see negative HSYNC pulse then check
      // to see if we need to force field value
      if (lastfield==0)
        curfield=1;    
      else
        curfield=0;          
    }
    else
    {
      curfield=1;
    }

#ifdef VSYNC_DEBUG_PIN
#ifdef DEBUG_PIN  
      if (curfield == 0)
        DEBUG_SET();
      else
        DEBUG_CLR();
#endif
#endif

    lastfield=curfield;

    totalfields++;
  }
}

void initializeSYNCPins()
{
  // VSYNC
  VSYNC_CFG_INPUT();
  VSYNC_CFG_PCMSK();
  VSYNC_CFG_PCICR();
  
  // HSYNC
  HSYNC_CFG_INPUT();
}

//
// Blink LED with interrupt
//
// For first pulse set state of LED manually, then load up sequence arrays for the pulse
// The length will be decremented and then when reaches zero if another entry in sequence
// it will be loaded, and so on.
// When all sequences run LED is turned OFF

ISR(TIMER1_OVF_vect)        // interrupt service routine 
{

//DEBUG_SET();
  
  TCNT1 = timer1_preset;   // preload timer

  // check current
  if (npulse > 0)
  {
    pulseseqlen[0] -= 1;
    if (pulseseqlen[0] == 0)
    {
      // shift sequence up
      int i;
      for (i=0; i < npulse; i++)
      {
        pulseseqlen[i] = pulseseqlen[i+1];
        pulseseqval[i] = pulseseqval[i+1];
      }

      npulse--;

      if (npulse == 0)
      {
        digitalWrite(PPSLED, 0);
DEBUG_CLR();
        return;
      }

      digitalWrite(PPSLED, pulseseqval[0]);    
if (pulseseqval[0])
  DEBUG_SET();
else
  DEBUG_CLR();
    }
  }

//delayMicroseconds(10);
//DEBUG_CLR();  
}

#ifdef DEBUG_PIN
void initializeDEBUGPins()
{
    // debug pin
    DEBUG_CFG_OUTPUT();
}
#endif 

void initializeLEDTimer()
{
  // Timer1 for LED blinking
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0; 
  
  // Set timer1_counter to the correct value for our interrupt interval
  // MSF - 2msec
  timer1_preset = TIMER1_PRESET;   // preload timer 65536-16MHz/256/100Hz
  
  // MSF - using 10msec
  //timer1_preset = 64911;   // preload timer 65536-16MHz/256/100Hz
  
  TCNT1 = timer1_preset;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts  
}

void initializeGPS()
{
  
  gpsSerial.begin(gpsBaud);
  
  sendGPSCommand("$PUBX,40,RMC,0,1,0,0,0,0*46");         // RMC ON
  delay(250);
  sendGPSCommand("$PUBX,40,GGA,0,1,0,0,0,0*5B");         // GGA ON
  delay(250);
  sendGPSCommand("$PUBX,40,DTM,0,1,0,0,0,0*47");         // DTM ON 
  delay(250);
  sendGPSCommand("$PUBX,40,VTG,0,0,0,0,0,0*5E");         // VTG OFF
  delay(250);
  sendGPSCommand("$PUBX,40,GSA,0,0,0,0,0,0*4E");         // GSA OFF
  delay(250);
  sendGPSCommand("$PUBX,40,GSV,0,0,0,0,0,0*59");         // GSV OFF
  delay(250);
  sendGPSCommand("$PUBX,40,GLL,0,0,0,0,0,0*5C");         // GLL OFF
}

void testPattern()
{
  int i;
  int j;

  for (j=0; j<13; j++)
  {
    for (i=0; i<30; i++)
    {
      OSD.setCursor(i, j);
      OSD.print(i%10);
    }
  }   
}

void setup() 
{

#ifdef DEBUG_PIN
    initializeDEBUGPins();
#endif

    initializeSYNCPins();

    // misc pin setup
    pinMode(PPSLED, OUTPUT);
    pinMode(GPSPPS, INPUT);

    initializeLEDTimer(); 
    
    initializeGPS();

    // Initialize the SPI connection:
    SPI.begin();
    SPI.setClockDivider( SPI_CLOCK_DIV2 );      // Must be less than 10MHz.
    
    // Initialize the MAX7456 OSD:
    uint8_t rows = OSD.safeVideoRows[MAX7456_NTSC][MAX7456_FULLSCREEN];
    uint8_t cols = OSD.safeVideoCols[MAX7456_NTSC][MAX7456_FULLSCREEN];    
    OSD.begin();                // Use NTSC with full area.
    OSD.setDefaultSystem(MAX7456_NTSC) ;
    OSD.setTextArea(rows, cols, MAX7456_FULLSCREEN);
    //OSD.setSyncSource(MAX7456_EXTSYNC);
    OSD.setSyncSource(MAX7456_AUTOSYNC);
    OSD.setWhiteLevel(0);  // should be 0% black 120% white

    // align with IOTA-VTI
    OSD.setTextOffset(OSD_X_OFFSET, OSD_Y_OFFSET);
    
    OSD.display();                              // Activate the text display.

#if 0
  testPattern();

  while(1)
    ;
#endif
  }
  // setup()

// Main Code ///////////////////////////////////////////////////////////////////

void loop() 
{
    while (OSD.notInVSync());                   // Wait for VSync to start to 
                                                //   prevent write artifacts.
    OSD.home();

#define STATE_WAITRISE 1
#define STATE_WAITFALL 2

    int state=STATE_WAITRISE;
                  
    while (true)
    {
      if (state==STATE_WAITRISE)
      {
        if (digitalRead(GPSPPS))
        {
          lastpps_millis=millis();
          
          fieldnum = 0;

          newpps = true;

          // turn on LED immediately to avoid possible delay waiting on next
          // interrupt to turn it on
          digitalWrite(PPSLED, 1);

DEBUG_SET();

          // setup pulse to turn off in 50msec
          noInterrupts();           // disable all interrupts
          npulse = 1;
          pulseseqlen[0] = 4/TIMER1_MSEC;
          pulseseqval[0] = true;

#if 0
          static long r=0;

#if 0
          // second test pulse of 10msec after 500msec        

          r = 16+random(980);
#else
          r+=25;
          if (r>974)
            r=25;
#endif
          
          pulseseqlen[npulse] = (r-4)/TIMER1_MSEC;
          OSD.setCursor(1, 6);
          printOSDZeroPaddedN(r, 3); 
            
//          pulseseqlen[npulse] = 490/TIMER1_MSEC;
          pulseseqval[npulse] = false;
          npulse++;
          pulseseqlen[npulse] = 4/TIMER1_MSEC;
          pulseseqval[npulse] = true;
          npulse++;
#endif     
          interrupts();             // enable all interrupts  
                   
          OSD.setCursor(PPSCHAR_COL, PPSCHAR_ROW);
          OSD.write(0xff);  
         
          state=STATE_WAITFALL;

          // flush serial
          gpsSerial.flush();
        }
      }
      else if (state==STATE_WAITFALL)
      {
        if (!digitalRead(GPSPPS))
        {
            OSD.setCursor(PPSCHAR_COL, PPSCHAR_ROW);
            OSD.write((uint8_t)0x00);              
            state=STATE_WAITRISE;
        }
      }

      // handle gps data
      handleGPSOutput();

      handleVSYNC();
       
    }
 } 
  // loop()

int sendGPSCommand(char *cmd)
{
  gpsSerial.println(cmd);
}

// print v with d digits, zero padded
void printOSDZeroPaddedN(long v, int n)
{
  int i;
  long p=1;
  long j;

  j=v;

  for (i=0; i<(n-1); i++)
    p *= 10;

  while (p>0)
  {
    long r;

    r = j/p;

    OSD.print(r);

    j = j-r*p;

    p /= 10;
  }

}

void handleVSYNC()
{
  static boolean first=true;
  static boolean last;
  static boolean toggle=true;
  static int count=0;
  boolean cur;
  boolean ppsgood;

  // do we have a good pps - if not seen for ~1 second
  ppsgood = (millis()-lastpps_millis) < 1050;
  
  if (first)
  {
    first=false;
    last = VSYNC_READ();
    return;
  }

  cur = VSYNC_READ();

  if (cur && (cur != last))
  {
    count++;

     OSD.setCursor(FIELD1TS_COL, FIELD1TS_ROW);     
     OSD.print("              ");

    // alternate printing timestamp at two locations depending on odd/even field
    if (curfield == 1)
    {
      OSD.setCursor(VSYNCCHAR_COL, VSYNCCHAR_ROW);   
      if (toggle)
        OSD.write(0xff); 
      else
        OSD.write((uint8_t)0x00);              

      OSD.setCursor(FIELD1TS_COL, FIELD1TS_ROW);    
      if (ppsgood) 
        printOSDZeroPaddedN(10*(lastvsync_millis - lastpps_millis), 4);
      else
        OSD.print("XXXX");

      toggle=!toggle;
      count=0;      
    }
    else
    {
      OSD.setCursor(FIELD2TS_COL, FIELD2TS_ROW);  
      if (ppsgood) 
        printOSDZeroPaddedN(10*(lastvsync_millis - lastpps_millis),4);
      else
        OSD.print("XXXX");
       
    }

    OSD.setCursor(FIELDTOT_COL, FIELDTOT_ROW);   
    OSD.print(totalfields);
     
    fieldnum++;
  }

  last=cur;
}



int handleGPSOutput()
{
  static unsigned long query_leapsec_nextmillis=0;
  static unsigned long loccycle_nextmillis=0;
  static unsigned long loccycle=0;

  while (gpsSerial.available())
  {
    char c = gpsSerial.read();

    handleVSYNC();

    // did we capture a new sentence
    if (gps.encode(c))
    {
      // do nothing for now
    }
  
    // wait till newpps done before
    if (gps.time.isUpdated() && (gps.time.age() < (millis()-lastpps_millis)) && newpps)
    {
      unsigned int sec, vmin;

      unsigned int h, m, s, y, mo, d;

      // set false so we only do once per second
      // newpps will be set true on next PPS pulse received
      newpps = false;

      //OSD.setCursor(0, 12);
      h=gps.time.hour();
      //printOSDZeroPaddedN(h,2);   
      m=gps.time.minute();
      //printOSDZeroPaddedN(m,2);
      vmin=m;
      s=gps.time.second();
      //printOSDZeroPaddedN(s,2);
      sec=s;

      OSD.setCursor(TIME_COL, TIME_ROW);
      printOSDZeroPaddedN(h,2); 
      OSD.print(" ");  
      printOSDZeroPaddedN(m,2);
      OSD.print(" ");  
      printOSDZeroPaddedN(s,2);

      boolean leapok=false;
      
      if (leapsec.isValid())
      {
        char *p;
        char *s;

        // scan for a 'D' in leapsec - means it isn't updated
        p=leapsec.value();
        for (s=p; *s; s++)
          if (*s == 'D')
            break;

        if (*s != 'D')
          leapok = true;
      }
      // put '?' after UTC time if leap sec not updated
      if (leapok)
        OSD.print(" ");
      else
        OSD.print("?");

#ifdef DOUBLE_PULSE
      // do double pulse for 00 seconds
      // turn off and then on to separate from PPS blink
      if (sec==0)
      {
        noInterrupts();
        pulseseqlen[npulse] = 5;
        pulseseqval[npulse] = false;
        npulse++;
        pulseseqlen[npulse] = 5;
        pulseseqval[npulse] = true;   
        npulse++;
        pulseseqlen[npulse] = ((vmin % 10)+1)*5;
        pulseseqval[npulse] = false;   
        npulse++;
        pulseseqlen[npulse] = 5;
        pulseseqval[npulse] = true;   
        npulse++;      
        interrupts();  
      }
#endif // DOUBLE_PULSE      
    }
    else if ((gps.time.age() < (millis()-lastpps_millis)) > 1000)
    {
      // no time update for last 1500msec
      OSD.setCursor(TIME_COL, TIME_ROW);
      OSD.print("XX XX XX"); 
    }
  }

  if (millis() > query_leapsec_nextmillis)
  {
    sendGPSCommand("$PUBX,04*37");                         // PUBX,04 POLL

    query_leapsec_nextmillis += LEAPSEC_UPDATE_MSEC;
  }


  if (1 && (millis() > loccycle_nextmillis))
  {
    loccycle_nextmillis += LOCCYCLE_UPDATE_MSEC;  

    // always update fix
    OSD.setCursor(FIX_COL, FIX_ROW);

    if (fixvalid.isValid() && fixvalid.age() < 5000)
    {
      if (fixvalid.value()[0] == 'A')
      {
        OSD.print("F");
        if (gps.satellites.value() < 10)
          OSD.print(gps.satellites.value());
        else
          OSD.print(">");
      }
      else
      {
        OSD.print("NO ");      
      }
    }
    else
    {
      OSD.print("NO ");
    }

    // now cycle other
  
    OSD.setCursor(CYCLE_COL,CYCLE_ROW);
    OSD.print("                  ");
    OSD.setCursor(CYCLE_COL,CYCLE_ROW);
    if (loccycle == 0 && gps.location.isValid())
    {
      OSD.print("LAT:");   
      OSD.print(gps.location.lat(),6);
      if (gps.location.age() > 1000)
        OSD.print(" OLD");
    }
    else if (loccycle == 1 && gps.location.isValid())
    {
      OSD.print("LNG:");   
      OSD.print(gps.location.lng(),6);
      if (gps.location.age() > 1000)
        OSD.print(" OLD");
    }
    else if (loccycle == 2 && gps.location.isValid())
    {
      OSD.print("ALT:");   
      OSD.print(gps.altitude.meters());
      OSD.print("m");
      if (gps.altitude.age() > 1000)
        OSD.print(" OLD");      
    }
    else if (loccycle == 3 && gps.location.isValid())
    {
      OSD.print("SAT:");
      OSD.print(gps.satellites.value());
      if (gps.satellites.age() > 1000)
        OSD.print(" OLD");  
    }
    else if (loccycle == 4 && datum.isValid())
    {
      OSD.print("DAT:");
      OSD.print(datum.value());
      if (datum.age() > 1000)
        OSD.print(" OLD");      
    }
    else if (loccycle == 5 && leapsec.isValid())
    {
      OSD.print("SEC:");
      OSD.print(leapsec.value());
      if (leapsec.age() > 15000)
        OSD.print(" OLD");
    }
    else if (loccycle == 6 && gps.date.isValid())
    {
      OSD.print("UTC:");
      OSD.print(gps.date.value());
      if (gps.date.age() > 15000)
        OSD.print(" OLD");
    }
    loccycle++;
    if (loccycle > 6)
      loccycle = 0;
  }
}

