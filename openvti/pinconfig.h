#ifndef PINCONFIG_H_
#define PINCONFIG_H_

// Pin Mapping /////////////////////////////////////////////////////////////////
  
  // pinValue = 0 means "not connected"

  //  FDTI Basic 5V                   ---  Arduino  VCC      (AVCC,VCC)
  //  FDTI Basic GND                  ---  Arduino  GND      (AGND,GND)
  //  FDTI Basic CTS                  ---  Arduino  GND      (AGND,GND)
  //  FDTI Basic DTR                  ---  Arduino  GRN
  //  FDTI Basic TXO                  ---> Arduino  TXO [PD0](RXD)
  //  FDTI Basic RXI                 <---  Arduino  RXI [PD1](TXD)
  
  
  //  Max7456 +5V   [DVDD,AVDD,PVDD]  ---  Arduino  VCC      (AVCC,VCC)
  //  Max7456 GND   [DGND,AGND,PGND]  ---  Arduino  GND      (AGND,GND)
  //  Max7456 CS    [~CS]            <---  Arduino  10  [PB2](SS/OC1B)
  //  Max7456 CS    [~CS]            <---  Mega2560 43  [PL6]
  const byte osdChipSelect             =            10;
  
  //  Max7456 DIN   [SDIN]           <---  Arduino  11  [PB3](MOSI/OC2)
  //  Max7456 DIN   [SDIN]           <---  Mega2560 51  [PB2](MOSI)
  const byte masterOutSlaveIn          =                      MOSI;
  
  //  Max7456 DOUT  [SDOUT]           ---> Arduino  12  [PB4](MISO)
  //  Max7456 DOUT  [SDOUT]           ---> Mega2560 50  [PB3](MISO)
  const byte masterInSlaveOut          =                      MISO;
  
  //  Max7456 SCK   [SCLK]           <---  Arduino  13  [PB5](SCK)
  //  Max7456 SCK   [SCLK]           <---  Mega2560 52  [PB1](SCK)
  const byte slaveClock                =                      SCK;
  
  //  Max7456 RST   [~RESET]          ---  Arduino  RST      (RESET)
  const byte osdReset                  =            0;
  
  //  Max7456 VSYNC [~VSYNC]          -X-
  //  Max7456 HSYNC [~HSYNC]          -X-
  //  Max7456 LOS   [LOS]             -X-

  // VTI mapping

  // GPS PPS [3.3v] [INPUT]    Arduino 7
#define GPSPPS 7

  // PPS LED OUTPUT
#define PPSLED  4

  // VSYNC 
//#define VSYNC 2  // legacy definition when using digitalWrite/digitalRead
#define VSYNC_PIN   PD2
#define VSYNC_DDR   DDRD
#define VSYNC_PINR  PIND
#define VSYNC_CFG_INPUT() (VSYNC_DDR &= ~_BV(VSYNC_PIN))
#define VSYNC_READ() (VSYNC_PINR & _BV(VSYNC_PIN))

// set these macros to enable the pin change interrupt for the 
// pin select for VSYNC
#define VSYNC_CFG_PCMSK()     (PCMSK2 |= _BV(VSYNC_PIN))
#define VSYNC_CFG_PCICR()     (PCICR |= 0b00000100)
#define VSYNC_PINCHANGE_ISR() ISR(PCINT2_vect)


  // HSYNC
#define HSYNC 3
#define HSYNC_PIN   PD3
#define HSYNC_DDR   DDRD
#define HSYNC_PINR  PIND
#define HSYNC_CFG_INPUT() (HSYNC_DDR &= ~_BV(HSYNC_PIN))
#define HSYNC_READ() (HSYNC_PINR & _BV(HSYNC_PIN))

// optional debug pin output
#define DEBUG_PIN   PB1

#ifdef DEBUG_PIN
#define DEBUG_DDR   DDRB
#define DEBUG_PORT  PORTB
#define DEBUG_CFG_OUTPUT(x) (DEBUG_DDR |= _BV(DEBUG_PIN))
#define DEBUG_SET() (DEBUG_PORT |= _BV(DEBUG_PIN))
#define DEBUG_CLR() (DEBUG_PORT &= ~_BV(DEBUG_PIN))
#endif // DEBUG_PIN

#endif // PINCONFIG_H_

