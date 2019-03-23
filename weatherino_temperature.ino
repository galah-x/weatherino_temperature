//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'weatherino temp humidity pressure rainfall wind for moteino Time-stamp: "2019-03-01 13:07:33 john"';

// $ grabserial -b 19200 -d /dev/ttyUSB1 | ts [%y%m%d%H%M%S]



#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //included with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>






//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        12   // unique for each node on same network
#define GATEWAYID     1    // node Id of the receiver we are sending data to
#define NETWORKID     100  //the same on all nodes that talk to each other including this node and the gateway
#define FREQUENCY     RF69_915MHZ //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
// #define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
// #define USE_ENCRYP
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!

//*********************************************************************************************

#define TEMP_LOOPS 240
// 2s per loop, 8 minutes updates, 8 min is 480s , at 2s per is 240
// note temp, pressurure, humid, othertemp come out on successive 2s intervals
// #define VOLT_LOOPS 10800
// 2s per loop, 6 hr updates, ie 21600 secs, at 2s per is 10800
 #define VOLT_LOOPS 3000
// 2s per loop, ~2 hr updates



//*********************************************************************************************
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8
#endif

// IOs
// BMP280 is on I2C at default address
// SHT31  is on I2C at address 0x44
// sda = A4
// scl = A5
#define BATT_IN A7
#define BATT_MEAS_GND A0


byte sendLen;
#define BUFLEN 50

char buff[BUFLEN]; //this is just an empty string used as a buffer to place the payload for the radio
char buff2[10];    // for float conversion

RFM69 radio;

int temp_loop;
int volt_loop;

Adafruit_BMP280 bme;    // I2C
Adafruit_SHT31 sht31 = Adafruit_SHT31();

long unsigned current_msec;
  

void setup() {
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif	 
#ifdef USE_ENCRYPT
  radio.encrypt(ENCRYPTKEY);
#else
  radio.encrypt(null);
#endif
  
  sprintf(buff, "%02x weatherino_temp 201903011305", NODEID );  
  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
  delay(200);
  
  temp_loop = 4;
  volt_loop = 0;
  
  pinMode(BATT_MEAS_GND, OUTPUT);
  digitalWrite(BATT_MEAS_GND, HIGH);
  // turn off divider to save power. 
  // pinMode(BATT_MEAS_GND, INPUT);
  
  if (!bme.begin())
    {  
      sprintf(buff, "%02x Could not find a valid BMP280 sensor", NODEID);  
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      delay(200);
      // I suspect the radio.sendWithRetry has to wait for my gateway..
    }
  
  if (!sht31.begin(0x44))
    {   // Set to 0x45 for alternate i2c addr
      sprintf(buff, "%02x Could not find a valid SHT31 sensor", NODEID );  
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      delay(200);
    }
    
  radio.sleep();
}

/************************** MAIN ***************/

  
void loop() {
  int batt_adc;
  int ref_adc;
  float batt_v;
  
  if (temp_loop == 3) {

    // *************** bmp280 temp and pressure **************
    // no %f in sprintf
    dtostrf(bme.readTemperature(), 5, 2, buff2);
    
    sprintf(buff, "%02x bmp280Temperature=%s°C", NODEID, buff2  );  
    radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    radio.sleep();
  }

  if (temp_loop == 2) {
    dtostrf(bme.readPressure(), 8, 2, buff2); // this returns a 32 bit integer.
    snprintf(buff, BUFLEN, "%02x Pressure=%s Pa", NODEID, buff2   );  
    radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    radio.sleep();
  }

    // *************** sht31 temp and humidity **************

  if (temp_loop == 1) {
    float t = sht31.readTemperature();
    dtostrf(t, 5, 2, buff2);
    sprintf(buff, "%02x sht31Temperature=%s°C", NODEID, buff2  );  
    radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    radio.sleep();
  }
        
  if (temp_loop <= 0) {
    float h = sht31.readHumidity();
    dtostrf(h, 5, 2, buff2);
    sprintf(buff, "%02x sht31Humidity=%s%%", NODEID, buff2  );  
    radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    radio.sleep();
    
    temp_loop = TEMP_LOOPS;
    // temp_loop = 4;
  }
  temp_loop--;


  // do a batt voltage update about 4 times per day
  if (volt_loop <= 0)
    {
      // turn on battery divider
      pinMode(BATT_MEAS_GND, OUTPUT);
      digitalWrite(BATT_MEAS_GND, LOW);
      // do a few adc reads to let is settle. Use the last one
      delay(2); // Wait for Vin to settle
      batt_adc =  analogRead(BATT_IN);
      
      ref_adc =  readVref();
      if (ref_adc == 0)
	{
	  ref_adc = 1;
	}
      
      analogReference(DEFAULT);
      
      
      // reading the reference as a fraction of Vdd allows me to calculate vdd
      // (1) Vdd = 1023 * v_ref / ref_v;
      // check:  vdd = 1023 * 1.1 / 337 = 3.34
      
      // then reading the scaled external voltage as a fraction of Vdd allows me to calculate ext voltage
      // (2) batt_v = (1000000 + 390000) / 1000000 * Vdd * (batt_adc / 1023);
      // check: batt_adc = 928, batt_v = 4.12
      // expanding
      // (3) batt_v = (1000000 + 390000) / 1000000 * 1023 * (v_ref /ref_v) * (batt_adc / 1023);
      // extracting constants
      // (4) batt_v = v_ref * (1000000 + 360000) / 1000000 )  * batt_adc / ref_v );
      //     batt_v =  1.36  * v_ref * batt_adc / ref_v ;
      //     batt_v =  1.36  * 1.1 * batt_adc / ref_v ;
      //     batt_v =  1.496 * batt_adc / ref_v ;
      // implies ref is really 4.12 / 4.07 = 1.085
      
      batt_v = 1.4934 * batt_adc / ref_adc; 
      dtostrf(batt_v, 5, 2, buff2);
      sprintf(buff, "%02x Batt=%sV", NODEID, buff2  );  
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      radio.sleep();
      
      // turn off divider to save power. 
      pinMode(BATT_MEAS_GND, INPUT);
      
      volt_loop = VOLT_LOOPS;
    }
  
  volt_loop--;
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);  //put microcontroller to sleep to save battery life

  // Blink(LED);
}

void Blink(byte pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(2);
  digitalWrite(pin, LOW);
}


int readVref() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  int result = (high<<8) | low;
  
  return result; // Vref as a fraction of vcc
}




 
 
