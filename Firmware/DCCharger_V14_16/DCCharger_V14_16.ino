/*
This is the firmware for EMotorWerks Intelligent DC Charging Systems - covering the following products:
* SmartCharge-12000 - a 12kW+ charging system
* QuickCharge-25000 - a 25kW+ charging system
* ISOCharge-20000 - a 20kW+ isolated charging system
* any DC-DC versions of the above

Controller: Arduino Pro Mini 5V (based on a ATmega328P microcontroller)

TODO: clean up ISR_ADC - we should not need to disable / enable interrupts within ISR! See post #8 at http://www.avrfreaks.net/forum/interrupt-handling-2

See QuickStart doc for details on charger operations etc: https://docs.google.com/a/emotorwerks.com/document/d/14axudenSziPm8gjc8Sv2n5Np-XDPIssUk-YkXAVsxSw/edit

********** HARDWARE MODS ARE REQUIRED to run this firmware on pre-V14 hardware!!! ************
* change RC filter on maxC line (pin 10 Arduino) - ensure C is no more than 1uF
* ensure filter capacitance is less or equal to 0.1uF on all current and voltage senging

This software is released as open source and is free for personal use ONLY.
Commercial use PROHIBITED without written approval from the Author and / or Electric Motor Werks, Inc.
Absolutely NO WARRANTY is provided and no guarantee for fit for a specific purpose

Original version created Jan 2011 by Valery Miftakhov, Electric Motor Werks, LLC & Inc. All rights reserved. Copyright 2011-...
*/
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "EEPROM_VMcharger.h"
#include <TimerOne.h>

//============================== MAIN SWITCHES FOR EMotorWerks chargers =========================
#define VerStr "V14.17"

#define SmartCharge // 12kW chargers
#define SC_LV // if using low-voltage output below 300V (e.g., Zero chargers) and high current output
const float MAX_SC_POWER=12000.; // normally 12000

#define MAX_OUT_CURRENT 95 // 70A by default, limit to 50A for old IGBTs

// derating starts here (C). should be 55 for older chargers with <200A IGBTs, 65 for 300A+
//        70-75C for slow-cooling conditions
const float deratingT=70;

// current-dependent offset for the output voltage measurement
float bV_C_offset;

const unsigned int linefreq=60; // 60Hz in the US - for best performance, set this to your country's line frequency!

// ================= SUB-SWITCHES DEFAULT VALUES FOR EMotorWerks chargers ========================
  #define OUTC_SENSOR_Allegro_100U // default is Allegro_100U
  // #define OUTC_SENSOR_Allegro_100B // default is Allegro_100U
  //#define OUTC_SENSOR_Allegro_150U // used for units with output current above 70A
  #define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
  #define PC817 // mains voltage sensing based on a crude regular opto (V13 boards)
  #undef NEG_CSENSE // got to be sure about this one of risk breaking the charger...
  #define DCDC_OUT_HIGH 0 // make sure this is zero to avoid swap of voltage readings
  // for 60hz line frequency, period has to ideally be 260 / N, where N is an integer
  // for 50hz line frequency, 312 / N
  const int period0=52; // 52us (~20khz) for 240VAC SC-12K applications; 60hz line freq

  const float A7520_negbias=-2.5; // -2.5 for units post-Aug 9, -3.3 for CREE, -5v for pre-Aug 9 units
  const float A7520_upperresistor=33; // 33k for units post-Aug 9, 46k for CREE, 68k for pre-Aug 9 units
  //const float A7520_negbias=-5.; // -2.5 for units post-Aug 9, -3.3 for CREE, -5v for pre-Aug 9 units
  //const float A7520_upperresistor=68; // 33k for units post-Aug 9, 46k for CREE, 68k for pre-Aug 9 units
  //const float A7520_negbias=-3.3; // -2.5 for units post-Aug 9, -3.3 for CREE, -5v for pre-Aug 9 units
  //const float A7520_upperresistor=46; // 33k for units post-Aug 9, 46k for CREE, 68k for pre-Aug 9 units


    const float maxC_default=MAX_OUT_CURRENT; // 80A for LV SC units
    const float maxPwr_default=12000; // 12kW for high voltage units
    const float upperR0_bV=2000.;

  const float upperR0_mV=2000.; // this normally does not need to change - it is only relevant for QC units anyway
  float bV_o_C=0.0; // 3V on a 30A current (as observed on V15.2 boards)

//----------------------------------- some hardcoded constants - DO NOT CHANGE
// scaler from PWM frequency. There is a method to the madness here - basically we want to be sampling things at 4x the line
// frequency and then use two adjacent readings to produce an average. This cancels out most of the 120Hz ripple from the readings
// in the sampling interrupt, every variable is sampled only every 1/16th period, hence the 16 divider below
// with 50uS period, 60Hz, this will produce 10.4 which will be rounded to 10, producing 4% phase error which is fine
int period=period0;
int MEASFREQPWMPRESCALE=(1000/period)*1000/(linefreq*4)/16;

// need to limit the minimum IGBT off period to more than 1 us to avoid super-short off pulses that can damage IGBTs
// see http://www.fujielectric.com/products/semiconductor/technical/application/box/doc/REH984b/REH984b_04a.pdf page 4-11 for more info
// 1us at 20kHz is ~2% so we could go to 98% duty cycle; 16kHz - 98.4%, 10kHz - 99%, etc.
// if you expect the system to run at the very high duty cycle, it may be beneficial to set the frequency to a lower value
// and this parameter to a higher value according to the above formula.
const unsigned long MAXDMILLIDUTY=9700000;

const unsigned int serialspeed=19200; // 115 is unstable, 38.4 is a bit unstable...

#define SerialStrSize 15 // M,ccc,vvv,sss,E
char SerialStr[SerialStrSize+2]; // buffer for the serial command
char SerialCommand[SerialStrSize+2]; // this is where the command will actually be stored

int cmd[2]={0, 0}; // command variables, used in serial comms; has to be signed value to avoid overwrap

//---------------- pin-out constants ----------------
//========== analog pins
const byte pin_C=0; // output current pin
const byte pin_bV=1; // output / battery voltage pin
const byte pin_heatSinkT=2; // charger heatsink temp - for thermal derating
const byte pin_12Vsense=3; // implementing undervoltage protection
const byte pin_temp2=4; // 4 - spare prewired as temp input
const byte pin_mV=5;
// const byte pin_mC=7; // will only work in V12+ control boards (June 2013). needed only for full digital PFC control
//========== digital pins
// 0/1 reserved for serial comms with display etc
const byte pin_pwrCtrlButton=2; // this is wired to the button (used for menu step)
const byte pin_pwrCtrl2Button=3; // this is wired to the button2 (used for menu select)
const byte pin_inrelay=4; // precharges input caps - normally pin 4, in some units pin 6 running fan relay
#ifdef SmartCharge
  const byte pin_outrelay=5; // output relay - pin 5 in SmartCharge
#else
  const byte pin_outrelay=4; // output relay - for non-SmartCHarge units, same pin to free up pin 5 for dual-direction units
#endif
const byte pin_dirCtrl=5; // control of the direction of power flow - LOW=BUCK, HIGH=BOOST
const byte pin_DELTAQ=6; // deltaQ pin
const byte pin_J1772=7; // J1772 pilot input. 1k is hardwired on V14+ pcbs so J1772 will power on on connect
const byte pin_fan=8; // fan control - this is pin4 in all kits shipped before Mar '12
const byte pin_PWM=9; // main PWM pin

// max current reference voltage (using PWM) -  was 6 in the V13 pcb (kits shipped before March 2012)
// now moved to pin 10 so that we can use higher PWM frequency 20kHz PWM
const byte pin_maxC=10;

const byte pin_EOC=12; // end-of-charge output (see pinout diagram) - pulled low when charge is complete

// end-of-charge input from BMS. Pull low / disconnect from positive TTL signal to activate
//     (normallly will be realized via connecting NC BMS loop between this pin and EOC pin (or +5V)
const byte pin_BMS=13;
//---------------- END PINOUTS -----------------------


//============= BATTERY INFO  =====
struct config_t {
  int nCells;
  byte period; // PWM period in microseconds
  int AH;
  int CV; // per cell
  int CC; // max output current
  int mainsC; // max input current
  // sensor config
  float Vcal;
  float Vcal_k;
  float mVcal;
  float Ccal;
  byte marker; // marker of the end of config - used to detect if config struct has changed between firmware updates - prevents config corruption
} configuration;
const byte testMarker=0xA5; // test marker for config rebuild
byte forceConfig=255, init_forceConfig=255; // default is 255 - has to be different from 0 or 1


//---------------- MAX CURRENTS
// absolute maximum average output current (used in CV mode) - leave at 0 here - will be set via power menu
const float min_CV_Crating=0.05; // wait until the current goes to XC (use values from your battery's datasheet)
const float Cstep=0.5; // how quickly the current tapers off in CV step - A / second. cannot be too high to prevent false stops
byte CVreached=0;
int CVreached_count=0;
int CVreached_count_threshold=2500; // 10 sec at 250Hz PID cycle
float maxOutC=0., maxOutC1=0;

// POWER_DIRECTION defines the orientation of the sensor with respect to the current flow
// =1 (DEFAULT) for when the current sensor output ramps up with the actual current (current flows in the positive sensing direction)
// =-1 for when the current flows in the opposite direction - such as when we flip the input and output for bidirectional operation
int POWER_DIRECTION=1; // has to be integer as it's a signed value
byte VSource=0; // are we using the system in Voltage Source mode (as opposed to current source)

// ratio between the average and max current in the inductor before overcurrent kicks in. 1.8 is a good value
// as it allows some ripple and some margin on top of that
const float peakMaxC=2.0;

int n=0;
byte selection = 0;
byte temp_selection = 1;
byte digit[3];
byte ji=0, si=0, x=0, xx=0, k=0;
byte PWM_enable_=0;
byte PWM_disable_reason=0;

float absMaxChargerPower_adjustment=1.;
      const float absMaxChargerCurrent=maxC_default;

  float absMaxChargerPower=maxPwr_default;

//------------- THERMAL DERATING OF CHARGER
// for now, simple protection by pausing charger until cooldown to certain temp
// note that heatSink temp at the point of measurement is generally 20-30 deg C LOWER than temperature
// of critical components attached to heatsink (due to distance from components to probe)
// use maxHeatSinkT of <60 to ensure <85 deg C temp of components
// this assumes thermistor placement near the heat generating components
// BTW, modest airflow (a single 120mm PC fan) with a large (8x10x3" heatsink should be sufficient for
// up to 30A output at max power
const byte maxHeatSinkT=deratingT; // in Centigrades - will start derating here
const byte ABSmaxHeatSinkT=85; // in Centigrades - will stop the charger altogether here
const byte midHeatSinkT=45; // turn on the fans here; also wait until cool down to this temp before resuming at the prev power level
const byte lowHeatSinkT=35; // turn off the fans here
//--------------------------------------------------------

const float Aref=5.0; // 5V for ATMega328 (Pro Mini), 3.3V for ATSAM (Due)

//=============== voltage dividers settings ===========================
//--------- some constants for 7520 chips
const float gain_7520=5./0.512*0.99; // calculate effective gain (Vcc/0.512 per datasheet, plus 1% correction for input resistance)
const float lowerR0_V_7520=2.7;

//--------- mains voltage
// INPUT side constants
float divider_k_mV=-1.;
#ifdef A7520_mV
  // resistor from -5V regulator; should form a ~-.25V divider together with the
  // bottom resistor => >20x * bottom resistor
  // for 2.7k bottom 3resistor, pick between 60k and 82k; 68k is a good choice...
  const float V_o_mV0=2.5+A7520_negbias*lowerR0_V_7520/A7520_upperresistor*gain_7520; // -5/3.3V input, 2.7k bottom resistor, 9.76 gain; // ~2.5V for A7520
  const float lowerR0_mV=lowerR0_V_7520*gain_7520; // +-0.256V range for input, ~10x gain, 2.7k bottom resistor
  const float lowerR_mV=lowerR0_mV;
#else
  const float V_o_mV0=0;
  const float lowerR_mV=23.79; // 1/(1/27.+1/200.) - in parallel with ISO124 input resistance of 200k
#endif
float V_o_mV=V_o_mV0; // need to reassign to non-const as it will be adjusted below
float Vdrop=2.; // voltage drop in the system - used for recalc of the currents

//--------- battery voltage
// OUTPUT side constants
float divider_k_bV=-1.;
#ifdef A7520_V
  // resistor from -5V regulator; should form a ~-.25V divider together with the
  // bottom resistor => >20x * bottom resistor
  // for 2.7k bottom resistor, pick between 60k and 82k; 68k is a good choice...
  // ((V-2.5)/10+2.5*2.7/33)*6000/2.7
  const float V_o_bV0=2.5+A7520_negbias*lowerR0_V_7520/A7520_upperresistor*gain_7520; // -5V input, 2.7k bottom resistor, 9.76 gain; // ~2.5V for A7520
  const float lowerR0_bV=lowerR0_V_7520*gain_7520;   // +-0.256V range for input, Vref/0.512x gain
  const float lowerR_bV=lowerR0_bV;
#else
  const float V_o_bV0=0;
  const float lowerR_bV=23.79; // in parallel with 200k input resistance of the iso124
#endif
float V_o_bV=V_o_bV0; // need to reassign to non-const as it will be adjusted below
//==================================== end voltage dividers setup =========================

//=================================== charger current sensor ==============================
// V/A constant for the charger output current sensor
float V_o_C=
#ifdef OUTC_SENSOR_Allegro_100U
                  0.6; // unidir allegros are 0.6
#else
  #ifdef OUTC_SENSOR_Allegro_150U
                  0.6; // unidir allegros are 0.6
  #else
                  2.5; // all bidirectionals are 2.5
  #endif

#endif

// sensitivity of the sensor
const float k_V_C=
#ifdef OUTC_SENSOR_Tamura_50B
                  0.03;
#endif
#ifdef OUTC_SENSOR_Tamura_150B
                  0.01;
#endif
#ifdef OUTC_SENSOR_Tamura_600B
                  0.0025;
#endif
#ifdef OUTC_SENSOR_Allegro_100U
                  0.04;
#endif
#ifdef OUTC_SENSOR_Allegro_100B
                  0.02;
#endif
#ifdef OUTC_SENSOR_Allegro_150U
                  0.0267;
#endif

const float e_V_C=  //error correction for 150U current sensors
#ifdef OUTC_SENSOR_Allegro_150U
                  0.05;
#else
                  0.00;  //no error correction
#endif
//=================================== END charger current sensor ==========================

//===================== charger cycle timers =====================================
// stepDelay is a delay between checking for serial commands and printing out the status to Serial line in a serial control mode
// to get the actual delay between such reports, add ~30ms for the actual duration of the data transmission over serial line
// when changing stepDelay, keep (stepDelay+30)*measCycle_len in the 500-1000 range
// higher stepDelay are advisable in case of non-CHADEMO applications
  const byte stepDelay=200; // primary charger loop delay in milliseconds
  const byte measCycle_len=4; // how many primary loop cycles per display cycle

const byte AVGCycles=10; // how many readings of C,V,T (taken every 4ms)  to average for reporting (to CHAdeMO) and display
const byte stopCycles=5; // how many primary charger cycles to require stop condition to exist before exiting
const byte CV_timeout=20; // what is the max duration (in secs) CV loop is allowed to spend below C stop; should be > ramp time of charger
byte breakCnt=0;
byte breakCycle=0;
//===================== end charger cycle timers =================================

unsigned long timer=0, timer_ch=0, timer_comm=0, timer_irq=0, timerBMS=0, deltat=0;
unsigned int sec_up=0;

float mainsV=0, init_mainsV=0, outV=0, outC=0;
float AH_charger=0;
char str[96];

// define state markers for state machines
// initial setup
const byte STATE_DONE = 0xff;
const byte STATE_CV = 0x1;
const byte STATE_CELLS = 0x2;
const byte STATE_PERIOD = 0x3;
const byte STATE_CAPACITY = 0x4;
const byte STATE_CALIBRATE = 0x5; // sensitivity calibration only. zero point calibration done automatically on power-on
// every run
const byte STATE_TOP_MENU = 0x00;
const byte STATE_CONFIG_PWR = 0x01;
const byte STATE_CONFIG_TIMER = 0x02;
const byte STATE_CHARGE = 0x04;
const byte STATE_WAIT_TIMEOUT = 0x05;
const byte STATE_SERIALCONTROL = 0x10;
const byte STATE_SHUTDOWN = 0xff;
byte state;

byte jumpedSerial=0;
byte charger_run=0;
byte normT=0;
const byte minMains=30; // min mains voltage to (1) test sensor connectivity and (2) detect mains disconnect
int timeOut=0; // in min, 0 means no timeout
float maxOutV=0; // absolute maximum output voltage - will be set later in the code
float maxMainsC=0; // allowed charger power - will be changed in code

unsigned long J1772_dur;

const byte rampsteps=50; // 20 steps to ramp - largely because every step can be as short as 100ms under serial command mode

byte rampcount=rampsteps;
//---------------------------------------------------------------------------------------------------------
// as of V13, completely new way to control the charger! proper PID loop and interrupt-based fast ADC
// this was originally driven by the need to meet requirements from the Leaf CHAdeMO protocol
//---------------------------------------------------------------------------------------------------------
// all these have to be ints or very unpleasant wrapping will occur in PID loop
// having these unsigned has cost EmotorWerks over $1,000 in parts during testing ;-)
int targetC_ADC=0.; // this is an ADC reference point for our output current
int targetV_ADC=0.; // this is an ADC reference point for our output voltage
int outC_ADC_0=0, outC_ADC=0, outV_ADC, bV_ADC=0, mV_ADC=0, T_ADC=0, T2_ADC=0;
float outC_ADC_f=0;

// ADC interrput handler
// this is always aligned with Timer1 interrupt
// ADC conversions are always done at 4kHz frequency (or every 250uS) so by the next Timer1 interrupt,
// we should ALWAYS have the result!
byte ul, uh; // all vars should be defined globally!
unsigned int val;
ISR(ADC_vect) { // Analog->Digital Conversion Complete
  cli(); // disable interrupt until function exit. otherwise nested interrupts...
  ul=ADCL;
  uh=ADCH;
  sei();

  val= (uh << 8 | ul); // assuming ADLAR=0

  // just load things into the variables and exit interrupt - processing all in main loop
  // for most variable, average 2 values offset 180 degrees wrt haversine wave
  // for current measurement, average 16 measurements over 8ms, or one full haversine period
  switch(ADMUX & B00000111) {
   case pin_C: { // this is measured at 2kHz frequency
     if(outC_ADC==0) {
       outC_ADC_f=outC_ADC=val;
     } else {
       // 16 cycles is 8ms here or a full haversine period
       outC_ADC_f=(outC_ADC_f*15+val)/16; // this emulates an RC filter with time constant of ~half of averaged periods
       outC_ADC=int(outC_ADC_f);
       outC=readC();
     }
     break;
   }
   // rest of vars measured at 250Hz
   case pin_bV: {
     if(bV_ADC==0) {
       bV_ADC=val;
     } else {
       bV_ADC=(bV_ADC+val)/2;
     }
     // set the ADC value for output voltage
     if(POWER_DIRECTION==1) outV_ADC=bV_ADC;
     outV=readV();
     break;
   }
   case pin_mV: {
     if(mV_ADC==0) {
       mV_ADC=val;
     } else {
       mV_ADC=(mV_ADC+val)/2;
     }
     // set the ADC value for output voltage
     if(POWER_DIRECTION==-1) outV_ADC=mV_ADC;
     break;
   }
   case pin_heatSinkT: {
     if(T_ADC==0) {
       T_ADC=val;
     } else {
       T_ADC=(T_ADC+val)/2;
     }
     break;
   }
   case pin_temp2: {
     if(T2_ADC==0) {
       T2_ADC=val;
     } else {
       T2_ADC=(T2_ADC+val)/2;
     }
     break;
   }
   default: break;
  }

} // end ADC interrupt


//---------------- interrupt magic to initiate ADC and calc PID loop ---------------------------
// PID loop setup - see http://en.wikipedia.org/wiki/PID_controller for some definitions
// using only PI part of it here
// parameter approximations ------------
// all constants below are effectively in 0.0001 units for the formula
// at 10-bit duty counter, 250Hz loop speed and 1000 unit range
// Example: 50A target ramp from zero, measured with a 100A unidir sensor: error is ~300
//          ramp rate is pids_Kp * 8 duty pts / sec (pids_Kp * 300 / 10000 duty points in one cycle (~4ms))
// for CHAdeMO unit with 50A max C, ramp to 50A in 2 seconds requires pids_Kp>60 (assuming full duty sweep would be required)
// OTOH, typical single-stage charger's stiffness is 10-20A per 10 duty points
// so we don't want to be making changes of more than 10 duty points per cycle
// which corresponds to pids_Kp<330
// so the meaningfull range is probably between 50 and 300
// our motor controller has Kp=3200, Ki=30, Kd=0
//----------------------- tuning charger PID:
// Zieglerâ€“Nichols method: the Ki and Kd gains are first set to zero.
// The P gain is increased until it reaches the ultimate gain, Ku, at which the output of the loop
// starts to oscillate. Ku and the oscillation period Pu are used to set the gains as shown:
// Control Type	Kp	Ki	        Kd
//    P    	0.50Ku	-	        -
//    PI	        0.45Ku	1.2Kp / Pu	-
//    PID	        0.60Ku	2Kp / Pu	KpPu / 8
// for this charger:
// on 330V pack (LiFePo4, milli-ohm total IR), at Kp=1000, see oscillations at Hz) - hence
// setting Kp=, Ki=
const long pids_Kp_FAST=200; // fast PID to start with
long pids_Kp=0;
const long pids_Ki=1; // need small integral term - otherwise we get some constant offset error
const long pids_Kd=0; // for now, just PI loop. if later decide to use, make sure uncomment the actual processing code below

long pids_err=0, pids_perr=0, pids_i=0, pids_d=0; // all have to be signed longs in order to not screw up pid calcs
long deltaDuty=0, milliduty=0; // have to be signed
byte tickerPWM=0; // short counter used only to skip cycles
byte tickerPWM1=0; // counter counting unskipped cycles - ok to overwrap

// called on overflow of Timer1 - called every 'period' uS (20 kHz by default)
// overflow with TimerOne library means we are in the center of the PWM cycle (TimerOne is phase correct)
void sampleInterrupt() {
  // trigger actual work only on every Nth period
  tickerPWM++;

  // prescale is defined based on period to make constant frequency of work below
  if(tickerPWM < MEASFREQPWMPRESCALE) return;
  // prescaler is calculated at startup so that we always end up with ~4kHz frequency here
  // therefore, every ADC conversion has 250 microseconds - which should be ok given that ADC on ATMega328P takes 100us
  // PID loop needs to also finish in much less than 250 microseconds or risk dragging the whole system down

  tickerPWM=0;
  tickerPWM1++; // this counts at lower frequency

  ADMUX &= B11111000; // reset the channel to zero

  // then current is measured every second cycle - or at ~2kHz frequency
  if(tickerPWM1 & 0x1) {
     ADMUX |= pin_C;
     ADCSRA |= B11000000; // manually trigger next one
  } else {
    // Every other parameter is measured once every 16 cycles => 250 Hz measurement frequency for every variable
    // PID loop runs at the same frequency, as well
    switch(tickerPWM1/2 & 0x7) {
       // case set below is MISSING 7 - available for other sensors
       case 0: {
         // average outC - this is only for the purposes of display. note that ADC value used here is ALREADY averaged in the ADC ISR
//         if(fabs(outC)<1.) outC=readC(); //
//         outC=(outC*float(AVGCycles-1)+readC())/AVGCycles;
         break;
       }
       case 1: {
         ADMUX |= pin_bV;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 2: {
         ADMUX |= pin_mV;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 3: {
         ADMUX |= pin_heatSinkT;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 4: {
         // average outV
//         if(fabs(outV)<1.) outV=readV();
//         outV=(outV*float(AVGCycles-1)+readV())/AVGCycles;
         break;
       }
       case 5: {
         ADMUX |= pin_temp2;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 6: {
          //====================   PID loop   ====================
          // PID loop needs to also finish in much less than 250 microseconds or risk dragging the whole system down
          // so all math should be integer to the extent possible!

          // determine which variable to operate PID loop on
          if(VSource==1) {
            // voltage source operates on outV_ADC - a 10-bit ADC target voltage representation
            pids_err = targetV_ADC - outV_ADC;
            if(PWM_enable_==0 && PWM_disable_reason==1) {
              PWM_disable_reason=0;
              PWM_enable_=1; // re-enable if the reason to stop was overvoltage
            }
          } else {
            // current source operated on outC_ADC - a 10-bit ADC target current representation
            pids_err = targetC_ADC - outC_ADC;
    #ifdef NEG_CSENSE
            pids_err *= -1; // the current signal (and hence the error sign) runs in a different direction
    #endif
            pids_err *= POWER_DIRECTION; // bidirectional - the current runs in opposide direction for POWER_DIRECTION=-1 modes!

            if(PWM_enable_ && (outV_ADC > targetV_ADC) ) {
              // since we are in a normal CSource mode, set the CVreached var - the actual C ramp=down is managed in the main loop
              // note that this we need to clear this flag right after setting targetV_ADC for the first time in the runChargeStep function
              CVreached_count++;
              if(CVreached_count>CVreached_count_threshold) CVreached=1; // this is set one-time and forever
            } else {
              if(CVreached_count>0) CVreached_count--; // don't allow underflow
            }
          }

          deltaDuty = pids_Kp * pids_err;

          pids_i += pids_err;
          deltaDuty += pids_Ki * pids_i;

//          pids_d = pids_err - pids_perr;
//          deltaDuty += pids_Kd * pids_d;

          pids_perr = pids_err;
          //==================== end PID loop ====================

          // immediate protection from overvoltage - zero out duty
          // this is a relatively slow protection (generally takes at least 4 ms to kick in) in addition to hardware overvoltage circuit
          // this also stops any term's accumulation before PWM_enable_ is turned on (e.g. before charger start)
          if( (PWM_enable_ == 0) || (outV > maxOutV+20) ) {
            milliduty=0;
            pids_i=0; // need to stop accumulation, as well
            // this is an emergency so stop all PWM
            PWM_enable_=0;
            if(outV > maxOutV+20) {
              PWM_disable_reason=1; // =1 means overvoltage
            }
          } else {
            // no hard stops - proceed
            // protect against overpowering (exceeding max C)
            // this operates on the average current so is not super-fast
            // it simply complements hardware protection that is implemented on the control board
            if(outC > 1.05*maxOutC1) {
              if(VSource==1) {
                deltaDuty=-10000;
              } else {
                deltaDuty-=10000; // 0.1% drop
              }
              if(pids_i>0) pids_i=0; // stop positive accumulation
            }

            milliduty += deltaDuty;
            if(milliduty < 0) {
              milliduty=0;
              // stop accumulation - both positive and negative
              pids_i=0;
            }
            if(milliduty > MAXDMILLIDUTY) {
              milliduty=MAXDMILLIDUTY;
              // stop positive error accumulation
              if(pids_i>0) pids_i=0;
            }
          }

          Timer1.setPwmDuty(pin_PWM, milliduty/10000);

          break;
       }

       default: break;

    } // end switch

  } // end if(tickerPWM1 & 0x1)

}  // end timer interrupt


//-------------------------------------------- START MAIN CODE ---------------------------------------------
void setup() {
  // digital inputs
  pinMode(pin_J1772, INPUT);

  // set output digital pins
  pinMode(pin_dirCtrl  , OUTPUT);
  pinMode(pin_inrelay, OUTPUT);
  pinMode(pin_outrelay, OUTPUT);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_maxC, OUTPUT);
  pinMode(pin_EOC, OUTPUT);
  pinMode(pin_fan, OUTPUT);

  // setup ADC
  ADMUX = B01000000;  // default to AVCC VRef, ADC Right Adjust, and ADC channel 0 (current)
  ADCSRB = B00000000; // Analog Input bank 1
  // ADC enable, ADC start, manual trigger mode, ADC interrupt enable, prescaler = 128 (3 bits in the end)
  // standard prescaler is 128 resulting in 125kHz ADC clock. 1 conversion takes 13 ADC cycles = 100uS using standard prescaler
  // 64 prescaler results in ~50uS conversion time
  ADCSRA = B11001111;

  //=================================== finalize init of the sensors =============================
  // reset voltage dividers to account for the input resistance of ISO124
  divider_k_mV=upperR0_mV/lowerR_mV;
  divider_k_bV=upperR0_bV/lowerR_bV;
  //=============================== END finalize init of the sensors =============================

  //==================================== ONE-TIME CONFIG =================================
  // check if needed to go into config
  forceConfig=255; // default is 255 - has to be different from 0 or 1
  EEPROM_readAnything(0, configuration);
  // reset configuration if the green button is pressed at charger start
  // on first connection, do zero cal of mainsV, as well
  if(configuration.marker!=testMarker) {
      forceConfig=1; // first time running the charger after assembly
      init_forceConfig=1; // copy
      configuration.CC=6;
      configuration.CV=415;
      configuration.nCells=28;
      configuration.AH=27;
      configuration.period=period0;
      // set the rest of the vars
      configuration.Vcal=0;
      configuration.Vcal_k=1.; // prefill the calibration with unity so we don't get zero readings if calibration menu is skipped
      configuration.mVcal=0;

      outC_ADC_0=outC_ADC; // ADC reference

      configuration.Ccal=POWER_DIRECTION*outC*k_V_C;

      configuration.marker=testMarker;
      // write it out
      EEPROM_writeAnything(0, configuration);
  }

  period=configuration.period; // load the PWM period setting
  MEASFREQPWMPRESCALE=(1000/period)*1000/(linefreq*4)/16;

  // setup timer - has to be before any ADC readouts
  Timer1.initialize(period);
  Timer1.pwm(pin_PWM, 0); // need this here to enable interrupt
  Timer1.pwm(pin_maxC, 0); // need this here to enable interrupt
  Timer1.attachInterrupt(&sampleInterrupt); // attach our main ADC / PID interrupt
  delay(50); // allow interrupts to fill in all analog values


  state = STATE_CV;


    // reset serial to faster speed
    Serial.end();
    jumpedSerial=isJumpedSerial(); // check if serial is jumped for timeout
    Serial.begin(serialspeed);

    // skip config altogether
    state=STATE_DONE;

  // parameters calculated from config variables go here
  // adjust core sensor constants
  V_o_bV=V_o_bV0+configuration.Vcal;
  V_o_mV=V_o_mV0+configuration.mVcal;
  V_o_C+=configuration.Ccal;
  divider_k_bV*=configuration.Vcal_k;

  // write out the configuration to EEPROM for next time
  EEPROM_writeAnything(0, configuration);

  init_mainsV=read_mV();      //check for input voltage level
  if(init_mainsV < minMains)  //if input voltage is too low (or input not plugged in)
  {

    sprintf(str, "INPUT: %dV, %d", int(init_mainsV),int(mV_ADC*100));
    printMsg(str);
    printParams(outV, outC, normT, AH_charger, maxOutC1, maxOutV);

    while(1)        //loop until input voltage > threshold
    {
      init_mainsV = read_mV();
      if(init_mainsV >= minMains)
      {
        break;
      }
    }
    delay(50);  //delay for CAPs to charge
    init_mainsV=read_mV();
  }

  // generally for boost topology, the higher the voltage, the better we are
  // reverse for buck topology
  absMaxChargerPower_adjustment=1.;

  // if too fast slow it down
  // this is ~5 seconds after power-up, by this time, the input caps should be charged (using a )
  // input precharge CANNOT be done with regular resistors due to constant power consumption by the PFC stage (switching losses)
  digitalWrite(pin_inrelay, HIGH);
}


void loop() {
  // ---------------real loop()
  x=255; // default, has to be different from 0 or 1

  mainsV=read_mV();
  outV=readV();

  maxOutV=118.;

  // run charger if:
  //         (1) charger has NOT been run yet in this cycle, or
  //         (2) has been run over a week ago
  //         (3) green button is pressed to override
  if(charger_run==0) {
    // check J1772 - has to be here in order to work for both serial and manual control
    readJ1772();

      // run state machine:
      //
      //state = STATE_WAIT_TIMEOUT; // default state
      // drop us directly into a serial control loop
      state=STATE_SERIALCONTROL;

      if(configuration.CC<=0) state=STATE_CONFIG_PWR;

      while(state != STATE_SHUTDOWN)
      {
        // reload voltages
        mainsV=read_mV();
        outV=readV();

        //======================== MAIN STATE MACHINE ======================
        switch(state)
        {

        case STATE_SERIALCONTROL:
          // serial control of the charger is enabled
          configuration.mainsC=160; // limit input current to something high but not crazy - 120A = 40% of a 400A IGBT rating
          //sprintf(str, "R:M%03d,V%03d,c%03d,v%03d", int(mainsV), int(outV), int(configuration.CC), int(maxOutV));
          sprintf(str, "R:M%03d,V%03d,c%03d,v%03d,T%03d", int(mV_ADC*100), int(outV), int(configuration.CC), int(maxOutV),int(getNormT()));

          EMWserialMsg(str); // send 'ready' status - expect controller to respond within 200ms

          // listen to commands - format 'M,ccc,vvv,sss,E' where sss is a checksum
          cmd[0]=cmd[1]=0; // reset all
          readSerialCmd(cmd);

          if(cmd[0]>0 && cmd[1]>0) {
            // echo reception
            sprintf(SerialCommand, "M,%03d,%03d,%03d,E", cmd[0], cmd[1], getCheckSum(cmd[0], cmd[1]) ); // this is needed here to prep command for loop function
            EMWserialMsg(SerialCommand);

            POWER_DIRECTION=1;
            digitalWrite(pin_dirCtrl, LOW);

            VSource=0;

            delay(100);

            configuration.CC=cmd[0];
            maxOutV=cmd[1];
            // move to charge stat
            state=STATE_CHARGE;
          } else {
            delay(200); // wait a bit and do another check for a command - cannot wait too long due to QC timing.
            readJ1772();
          }
          break;

         case STATE_CHARGE:
           // cannot delay from here to charging function QC operation requires quick ramp after the command
           // write out the configuration to EEPROM for next time
           EEPROM_writeAnything(0, configuration);

            //  set max hardware current protection to the fixed `hargeCurrent * [peakMaxC multiple] value
            // setMaxC(peakMaxC*getAllowedC(absMaxChargerCurrent));
            setMaxC(peakMaxC*absMaxChargerCurrent);

            timer_ch=millis(); // set the timer
            AH_charger=0; // reset AH counter

            // reset the EOC pin (this pin is active LOW so reset means setting to HIGH)
            // high means charging is commencing. this will be pulled down by the charger when charge ends
            // this also feeds a closed-loop BMS
            digitalWrite(pin_EOC, HIGH);

            //========================== MAIN RUN CHARGER FUNCTION=======================
            // by this point, at least 15sec have passed since AC connection
            //                and at least 10sec since battery connection
            //                therefore, all caps should be pre-charged => close relays
            //   (note: this requires precharge resistors across relays - MAX of 1k for outrelay.
            //          >10W power rating. Place small 1000V diode in
            //          series with the outrelay resistor - anode to battery - to avoid precharge on
            //          reverse polarity connection)
            digitalWrite(pin_outrelay, HIGH);
            // make sure in relay is on, as well!
            digitalWrite(pin_inrelay, HIGH);

            // check for invalid sensor configs
            // most dangerous is disconnection of the current sensor
            // generally will manifest itself by non-zero current reading while duty is zero
            // (which it should be at this point)
            // 10A is a lot of margin for that
            if(fabs(readC())>10) {
              printMsg("C>10A, CHECK SENSOR");
              return;
            }

            // CC-CV profile, end condition - voltage goes to CV and current goes to X% of CC
            // only run if the previous BMS stope was more than 10 min ago
            if(timerBMS==0 || (millis() - timerBMS > 60000*10) ) {
              runChargeStep();
            } else {
              printMsg("BMS lockout");
            }
            PWM_enable_=0; // HAS to be here to ensure complete stop on any condition

            // make sure everything is off
            digitalWrite(pin_outrelay, LOW);
            digitalWrite(pin_fan, LOW);
            digitalWrite(pin_EOC, LOW); // active low
            //==================== charger routine exited ===============================

            sprintf(str, "%dAH", int(AH_charger));

            EMWserialMsg(str);
            state = STATE_SERIALCONTROL; // ready for next run

            break;

          default: break;
        }
        //=========================== END MAIN STATE MACHINE
      }
    }

}


//-------------------------------- main charger routine ------------------------------------------------
int runChargeStep() {
  rampcount=0;
  pids_Kp=pids_Kp_FAST; // start with fast PID - will be changed to slow when we see some current

  maxOutC=getAllowedC(configuration.CC, 1);  //flag = 1 so that current ramps up from zero.
  maxOutC1=maxOutC;

  // reset V,C readings - otherwise averaging gets screwed up really badly
  outC=0;
  outV=0;
  outC_ADC_0=0, outC_ADC=0, bV_ADC=0, mV_ADC=0, T_ADC=0, T2_ADC=0;

    // machine-readable
    // this assumes that only CC commands will be issued to the charger via serial
    sprintf(str, "I:%d,%d,%d", int(configuration.AH*min_CV_Crating), int(maxOutC), int(maxOutV));
    EMWserialMsg(str);

  // reset timers - for AH metering and serial comms
  timer=millis(); // this will be reset every cycle below after AH delta is calculated
  timer_comm=timer;

  // turn on PWM output
  setTargetC();
  setTargetV();
  CVreached=0; // status of CV state - must be reset here, right after the targetV_ADC is set for the first time!
  PWM_enable_=1;

  //============================================== MAIN CHARGER LOOP =============================================
  while(1) {
    // NOTE THAT outC / outV readings are all set in the interrupts
    mainsV=read_mV();

    setTargetC();
    setTargetV();

  while(Serial.available()) {
    str[0]=Serial.read();
    if(str[0]=='M') si=0; // reset to the beginning of command
    if(si<SerialStrSize-1) {
      SerialStr[si++]=str[0];
    } else {
      SerialStr[SerialStrSize-1]=0; // this is supposed to be the end of the command
      if(str[0]=='E') strcpy(SerialCommand, SerialStr);
      si=0;
    }
  }

    // process serial commands and print out status only every 50ms or so
    if(millis()-timer_comm > stepDelay) {
      n++;
      timer_comm=millis();

      normT=getNormT();

      // if in Serial mode, check for commands here
    cmd[0]=cmd[1]=0; // reset all
    str[0]=SerialCommand[2]; // skipping 'M,'
    str[1]=SerialCommand[3];
    str[2]=SerialCommand[4];
    str[3]=0;
    cmd[0]=atoi(str);
    str[0]=SerialCommand[6]; // skipping ','
    str[1]=SerialCommand[7];
    str[2]=SerialCommand[8];
    str[3]=0;
    cmd[1]=atoi(str);
    str[0]=SerialCommand[10]; // skipping ','
    str[1]=SerialCommand[11];
    str[2]=SerialCommand[12];
    str[3]=0;
    if(atoi(str)!=getCheckSum(cmd[0], cmd[1])) { // checksum did not check out
      cmd[0]=0;
      cmd[1]=0;
    }
    if(cmd[0]>1 && cmd[1]>1) {
      // valid output power command
      // voltage
      maxOutV=cmd[1];

      configuration.CC=cmd[0]; // this also allows for temp derating (in getAllowedC)
      PWM_enable_=1; // re-enable if needed
    } else {
      // could be a special command
      // 'M,001,000,001,E' is STOP
      if(cmd[0]==1 && cmd[1]==0) {
        return 0; // full stop
      }
    }
    // send status now - this is ~45 symbols. At 19200 bps, this is  ~30ms
    printParams(outV, outC, normT, AH_charger, maxOutC1, maxOutV);

      // set the current to the requested value BUT ONLY if we are not exceeding maxOutV (which we will know via a CVreached var)
      // if maxOutV exceeded, the charger takes control and starts reducing the current
      // this has to be here so that the current gets adjusted regardless of whether we are in LCD or Serial mode
      // and regardless of whether we have commands coming in on Serial or not
      // this is critical for responding to a dynamic J1772 signal
      if(!CVreached) maxOutC=maxOutC1=getAllowedC(configuration.CC, 0);

      // slow voltage control cycle here. AT Cstep=0.5A default, we are ramping down at ~5A/second
      // this may not be enough to avoid a bit of overvoltage beyond CV
      // but only in non-VSource mode
      if(outV > maxOutV && CVreached==1) { // CVreached is now set in PID loop processing
        maxOutC-=Cstep;
        if(maxOutC<0) maxOutC=0;
      }

      //ramp up done on this line
      maxOutC1=getAllowedC(maxOutC, 1); // recalc maxOutC1 - this will also account for temp derating

    } // end stepDelay cycle

    //------------------------------------------------ print out stats ----------------------------------------
    // but only every few cycles. defaults: measCycle_len=20, stepDelay=30
    if(n>measCycle_len) {
      n=0;

      // re-check J1772 - in case we are being controlled etc
      // this changes configuration.mainsC and maxMainsC (which is used in getAllowedC() and affects max charge current)
      // this is a blocking call for up to 100ms (usually 50ms)
      readJ1772();

      // timer
      sec_up=(unsigned int)1.*(millis()-timer_ch)/1000;

      // check for break conditions
      // mask the first few seconds
      // if VSource is set, the loop never exits on this condition! (because CVreached stays at 0)
      if(sec_up>CV_timeout && CVreached && maxOutC1 < configuration.AH*min_CV_Crating) {
        breakCycle=1;
      } else {
        breakCycle=0; // reset
      }
      // do we REALLY need to break?
      if(breakCycle) {
        breakCnt++;
        if(breakCnt>stopCycles) {
          return 0;
        }
      } else {
        breakCnt=0;
      }

      // AH meter
      AH_charger+=outC*int(millis()-timer)/1000/3600;
      timer=millis();

      // check the timer
      if(timeOut>0 && (millis()-timer_ch)/60000>timeOut) {
        return 0;
      }

      //==================== print all parameters

      if(outV < -10 || outC < -10) {
        return 1; // full stop
      }

    }  // end measCycle loop

  }; //======================================== END MAIN CHARGER LOOP ===================================

  return 0;
} // end runChargeStep()


//============================ HELPER FUNCTIONS ====================================
//================== serial comms ========================
// message FROM the charger via serial
void EMWserialMsg(const char *txt) {
  Serial.print("M,");
  Serial.print(txt);
  Serial.println(",E");
}

// message TO the charger via serial
// command syntax: M,ccc,vvv,sss,E
void readSerialCmd(int *cmd_) {
      //+++++++++++++++++++ REWRITE into either async buffer or blocking reads +++++++++++++++++++++
  if(Serial.available()>0) {
    if(Serial.read()=='M') {
      // this is a legit command
      Serial.read(); // dispose of a comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      cmd_[0]=atoi(str);
      Serial.read(); // dispose of a comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      cmd_[1]=atoi(str);
      Serial.read(); // dispose of a comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      Serial.read(); // dispose of a comma
      if( Serial.read()!='E' || atoi(str)!=getCheckSum(cmd_[0], cmd_[1]) ) {
        cmd_[0]=cmd_[1]=0;
      }
    }
  }
}

int getCheckSum(int val1, int val2) {
  return (val1+val2)%1000;
}
//================== END serial comms ========================


unsigned long MYpulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
	// cache the port and bit of the pin in order to speed up the
	// pulse width measuring loop and achieve finer resolution.  calling
	// digitalRead() instead yields much coarser resolution.
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	uint8_t stateMask = (state ? bit : 0);
	unsigned long width = 0; // keep initialization out of time critical area

	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes 16 clock cycles per iteration.
	unsigned long numloops = 0;
	unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;

	// wait for any previous pulse to end
	while ((*portInputRegister(port) & bit) == stateMask)
		if (numloops++ == maxloops)
			return 0;

	// wait for the pulse to start
	while ((*portInputRegister(port) & bit) != stateMask)
		if (numloops++ == maxloops)
			return 0;

        // take a reading of current micros
        width=micros();
	// wait for the pulse to stop
	while ((*portInputRegister(port) & bit) == stateMask) {
		if (numloops++ == maxloops)
			return 0;
	}
        width=micros()-width;
        if(width>timeout) return 0; // this protects against overflows of micros

	return width;
}


//------------ check for jumper on serial lines - would indicate that we want timeout even if no LCD
byte isJumpedSerial() {
    //DISALLOW SERIAL JUMP
    return 0;
}


//------------ reading J1772 -------------------------
void readJ1772() {
  // check J1772
  // average over many periods - let's say 50
  const byte nsamples=50;
  unsigned long Jread=0;
  J1772_dur=0;
  ji=0;
  for(si=0; si<nsamples; si++) {
    Jread=MYpulseIn(pin_J1772, HIGH, 2000); // only block for 3ms max so that we are not left without PID control for more than that
    if(Jread>0) {
      ji++;
      J1772_dur+=Jread; // wait for 10ms for the pulse to appear - this will result in timeout of this function in 1 second
    }
  }
  J1772_dur/=ji;
  if(J1772_dur>50 && J1772_dur<1050) { // noise control. also, deals with the case when no J1772 signal present at all
    configuration.mainsC=0.06*J1772_dur; // J1772 spec - every 100uS = 6A input - this will work up to 48A
      if(configuration.mainsC<15){
          configuration.mainsC=15;
      }
      sprintf(str, "J: %d, %dA  ", int(J1772_dur), configuration.mainsC);
      Serial.println(str);

  } else {
    J1772_dur=-1;
  }

  // reset maxMainsC which is used in getAllowedC function. needs to be here because we are resetting the whole thing
  // in the main charging cycle
  maxMainsC=configuration.mainsC;
  mainsV=read_mV(); // for power adjustments
}


//------------ ensure output power does not exceed other limits
float getAllowedC(float userMaxC, byte flag) {

  userMaxC=min(userMaxC, absMaxChargerPower/maxOutV );

  userMaxC=min(userMaxC, absMaxChargerCurrent);

  userMaxC=min(userMaxC, maxMainsC*mainsV/maxOutV);

  // check thermal
  if(normT>=maxHeatSinkT) {
    // start derating
    // map 0-100% derating (relative to the current max current) to
    // maxHeatSinkT-ABSmaxHeatSinkT heatsink range
    userMaxC=userMaxC*abs(ABSmaxHeatSinkT-normT)/(ABSmaxHeatSinkT-maxHeatSinkT);

    if(normT>ABSmaxHeatSinkT) {
      // overheating, stop charger, wait until temp drops enough to restart
      PWM_enable_=0;
      while(1) {
        sprintf(str, "Cool from %dC", (int)normT);
        printMsg(str);
        normT=getNormT();
        if(normT<midHeatSinkT) {
          PWM_enable_=1; // restart PWM
          maxOutC1=userMaxC; // full power
          break; // exit cycle when the temp drops enough
        }
      }

    } // ABSmaxHeatSink condition

  } // end thermal management


  //current ramp up
  if(rampcount != rampsteps && flag == 1) {
    rampcount++;
    return userMaxC*rampcount/(rampsteps*1.0);
  }

  return userMaxC;
}

void setMaxC(float maxC) {
  Timer1.setPwmDuty(pin_maxC, min(1023, int(1024./Aref*(V_o_C+k_V_C*maxC)) ) ); // min() prevents wrapping the function - with disastrous results
}

//------------ set target current for the PID loop
void setTargetC() {
  // track targetC to maxOutC1
  // normal direction of current - current sensor is sitting on the output and works in the positive direction
    targetC_ADC=1024*(k_V_C*maxOutC1+V_o_C)/Aref;
}

//============================ current readout functions =====================
//------------ calc current value from ADC value
float readC() {
    return (1.0+e_V_C)*(Aref/1024.*outC_ADC-V_o_C)/k_V_C;
}

//============================ voltage readout functions =====================
// set targetV_ADC
void setTargetV() {
  targetV_ADC=(maxOutV/divider_k_bV+V_o_bV)/Aref*1024.;
}

// output voltage
float readV() {
  bV_C_offset=bV_o_C*outC; // bV_o_C is 0.1 for new V15.2 boards where the ground loop results in voltage reading dependent on the output current
  if(DCDC_OUT_HIGH*POWER_DIRECTION==1) return (Aref/1024.*mV_ADC-V_o_bV)*divider_k_bV;
  return (Aref/1024.*bV_ADC-V_o_bV)*divider_k_bV + bV_C_offset;
}

// input voltage
float read_mV() {
    if(outC>1.0) {
        if(init_mainsV<50){
            //sometimes does not work so always default to 208
            return 208;
        }
        return init_mainsV; // return start-up value if there is any current draw
    } else {
    //  // if zero current, then do measurement
    //  // 3V is a threashold between 120V and 208V - but may require adjustment on a per-unit basis
      if(Aref/1024.*mV_ADC < 4.1){
        return 208;
      }
      return 120;
    }
}
//============================ end voltage readout functions =====================


//====================== temperature readout functions ===========================
// compute the equivalent (normalized) charger temp
byte getNormT() {
  // assume max sink T is 55, max t2 (inductor) is 85 - approx but reasonably close to reality
  // therefore need to rescale t2 by 55/85=0.65
  // BUT ONLY IF WE HAVE THIS SECONDARY MEASUREMENT
  return read_T(T_ADC);
}
// master temp readout, using formulas from http://en.wikipedia.org/wiki/Thermistor
byte read_T(unsigned int ADC_val) {
  return (byte)1/( log(1/(1024./ADC_val-1)) /4540.+1/298.)-273;
}


//=========================================== Communication (LCD / Serial) Functions =========================
void printMsg(char *str_) {
  EMWserialMsg(str_);
}

// main parameter printing function
void printParams(float outV, float outC, int t, float curAH, float maxC, float maxV) {
      // machine-readable
    // format: [D]uty in 0.1%, [C]urrent in 0.1A, [V]oltage in 1.0V, [T]emp in C, [O]utput AH in 0.1AH, [S]um (checksum)

    //orig
    //sprintf(str, "S:D%03d,C%03d,V%03d,T%03d,O%03d,S%03d", int(milliduty/10000), int(outC*10), int(outV), t, int(curAH*10), getCheckSum(int(outC*10), int(outV)));

    sprintf(str, "S:D%03d,C%03d,V%03d,T%03d,O%03d,M:%03d,S%03d", int(milliduty/10000), int(outC*10), int(outV), t, int(curAH*10), int(mV_ADC*100),

    getCheckSum(int(outC*10), int(outV)));

    EMWserialMsg(str);
}
