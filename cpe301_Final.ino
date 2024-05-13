/*
 *  Name: Ryan Ransom 
 *  Assignment: Final Project
 *  Class: CPE301 Spring 2024
 *  Date: 05/12/2024
 */

#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include "uRTCLib.h"

#define RDA 0x80
#define TBE 0x20

// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;
// GPIO Pointers
volatile unsigned char *portB = (unsigned char *)0x25;
volatile unsigned char *portDDRB = (unsigned char *)0x24;
// Timer Pointers
volatile unsigned char *myTCCR1A = (unsigned char *)0x80;
volatile unsigned char *myTCCR1B = (unsigned char *)0x81;
volatile unsigned char *myTCCR1C = (unsigned char *)0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *)0x6F;
volatile unsigned char *myTIFR1 = (unsigned char *)0x36;
volatile unsigned int *myTCNT1 = (unsigned int *)0x84;

volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;

// led
#define BLUE_LED 4  // pin 2
#define RED_LED 5 // pin 3
#define YELLOW_LED 5  // pin 4
#define GREEN_LED 3 // pin 5

// water level
#define WATER_POWER 3  // pin 6
#define WATER_SIGNAL 0  // pin A0
unsigned int waterLevel;  // value holds current water level

// temp and humidity
dht DHT;
#define DHT11_PIN 7 // pin 7

// lcd
LiquidCrystal lcd(52, 50, 48, 46, 44, 42);  // pins 52, 50, 48, 46, 44, 42

// stepper motor
const int stepsPerRevolution = 200;
// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 13, 11, 12, 10);  // pins 13, 11, 12, 10
#define ventCW 0  // pin 49
#define ventCCW 4 // pin 47

// fan motor
#define speedPin 6  // pin 31
#define dir1 4  // pin 33
#define dir2 2  // pin 35

// rtc setup
uRTCLib rtc(0x68);
int currentState = 0;
int previousState;


// buttons
#define START_STOP 3   // pin 18
#define resetButton 2  // pin 19

//states
bool disabled = true;  // start on disabled state
bool idle = false;
bool error = false;
bool running = false;

// flags
int ledSelect = -1;      // controls which color led is powered
bool fan = false;        // controls if fan is on or off
bool display = false;    // controls if temperature and humidity are displayed on the lcd
bool vent = false;       // controls if vent positioning can be changed
bool monitor = false;    // controls if temp and humidity are monitored
bool showError = false;  // determines when error message is shown

// control variables
int minimumWaterLevel = 100;  // sets minimum water level
int minimumTemp = 26; // sets minim temperature
int currentTemp;  // holds current temperature
float currentHumidity;  // holds current humidity

// one minute delay
unsigned long currentMillis;  
unsigned long previousMillis = 0;
const long interval = 60000;


void setup() {
  U0Init(9600);
  adc_init();

  // leds
  ledSetup();

  // buttons
  buttonSetup();

  // interupt
  attachInterrupt(digitalPinToInterrupt(18), startStop, FALLING);

  // water level monitor
  DDRH |= (0x01 << WATER_POWER);
  waterMonitorOff();

  // lcd setup
  lcd.begin(16, 2);

  // fan motor setup
  fanSetup();

  // rtc setup
  URTCLIB_WIRE.begin();

  monitorTempHumidity();  // call to provide temp and humidity data when moving from disable to start
}

void loop() {
  if (disabled) {   // disabled state
    ledSelect = 4;  // yellow led
    display = false;  // turn off display
    showError = false;
    vent = false; // turn off vent control
    monitor = false;  // turn off temp and humidity monitoring
    controlFan(false);  // turn off fan
    waterMonitorOff();  // turn off water level monitoring
    waterLevel = 0; // reset water level
    currentTemp = 0;  // reset temperature
    lcd.clear();
    if (previousState == 1 || previousState == 2 || previousState == 3) {
      displayChange();  // if previous state is idle, running, or error display change to disabled
    }
  }

  if (idle) {       // idle state
    ledSelect = 5;  // green led
    display = true; // display temp and humidity
    showError = false;
    vent = true;  // enable vent controls
    monitor = true; // enable temp and humidity monitoring 
    if (previousState == 0) {
      displayChange();  // display change from disabled to idle state
    }
    waterMonitorOn(); // enable water level monitoring
    if (minimumWaterLevel > waterLevel) { // if water level is too low change to error state displaying state change
      previousState = currentState;
      currentState = 3;
      displayChange();
      lcd.clear();
      error = true;
      idle = false;
    }
    if (currentTemp > minimumTemp) {  // if temperature is too high change to running state displaying state change
      previousState = currentState;
      currentState = 2;
      displayChange();
      running = true;
      idle = false;
    }
  }

  if (running) {    // running state
    ledSelect = 2;  // blue led
    controlFan(true); // turn on fan
    display = true; // enable temperature and humidity display
    showError = false;
    vent = true;  // turn on vent controls
    monitor = true; // turn on temperature and humidity monitoring
    waterMonitorOn(); // turn on water level monitoring
    if (minimumWaterLevel > waterLevel) { // if water level is too low change to error state and display state change
      previousState = currentState;
      currentState = 3;
      displayChange();
      error = true;
      lcd.clear();
      running = false;
      controlFan(false);
    }
    if (minimumTemp >= currentTemp) { // if temperature is under threshold change to idle state and display state change
      previousState = currentState;
      currentState = 1;
      displayChange();
      idle = true;
      running = false;
      controlFan(false);
    }
  }

  if (error) {      // error state
    ledSelect = 3;  // red led
    display = false;  // disable temperature and humidity display
    showError = true; // enable error screen
    vent = false; // disable vent controls
    monitor = false;  // disable temperature and humidity monitoring
    waterMonitorOn(); // enable water level monitoring 
    if ((PIND & (0x01 << resetButton)) && minimumWaterLevel <= waterLevel) {  // if reset button is pressed and water level is above minimum
      previousState = currentState;
      currentState = 1;
      displayChange();  // display state change
      showError = false;  // disable error screen
      idle = true;
      error = false;
      displayTemp();  // display temperature and humidity
    }
  }
  setLed(); // sets led to correct color

  if (display) {  // if display is true 
    currentMillis = millis();
    if ((currentMillis - previousMillis) >= interval) { // one minute delay
      previousMillis = currentMillis;
      displayTemp();  // display temperature and humidity on lcd
    }
  }

  if (monitor) {  // if monitor is true
    monitorTempHumidity();  // read temperature and humidity
  }

  if (showError) {  // if showError is true
    errorMsg(); // display error message on lcd
  }
  if (vent) { // if vent is true
    while (PINL & (0x01 << ventCW)) { // if clockwise button is pressed
      myStepper.step(1);  // move vent clockwise
      my_delay(10);
    }
    while (PINL & (0x01 << ventCCW)) {  // if counter clockwise button is pressed
      myStepper.step(-1); // move vent counter clockwise
      my_delay(10);
    }
  }
}

// logic for controlling led power
void setLed() {
  // disable all leds
  PORTE &= ~(0x01 << BLUE_LED);
  PORTE &= ~(0x01 << RED_LED);
  PORTG &= ~(0x01 << YELLOW_LED);
  PORTE &= ~(0x01 << GREEN_LED);

  switch (ledSelect) {  // enable led based on value of ledSelect
    case 2:
      PORTE |= (0x01 << BLUE_LED);
      break;
    case 3:
      PORTE |= (0x01 << RED_LED);
      break;
    case 4:
      PORTG |= (0x01 << YELLOW_LED);
      break;
    case 5:
      PORTE |= (0x01 << GREEN_LED);
      break;
  }
}

// turns on water monitoring
void waterMonitorOn() {
  PORTH |= (0x01 << WATER_POWER);
  my_delay(10);
  waterLevel = adc_read(WATER_SIGNAL);
  waterMonitorOff();  // disable water monitoring
}

// turns off the water level monitor
void waterMonitorOff() {
  PORTH &= ~(0x01 << WATER_POWER);
}

// monitor for temperature and humiditys
void monitorTempHumidity() {
  int chk = DHT.read11(DHT11_PIN);
  currentTemp = DHT.temperature;
  currentHumidity = DHT.humidity;
}

// displays temperature and humidity on lcd screen
void displayTemp() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(DHT.temperature);
  lcd.print("%");
}

/*
 * UART FUNCTIONS
 */
void U0Init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}
unsigned char kbhit() {
  return *myUCSR0A & RDA;
}
unsigned char getChar() {
  return *myUDR0;
}
void putChar(unsigned char U0pdata) {
  while ((*myUCSR0A & TBE) == 0)
    ;
  *myUDR0 = U0pdata;
}

//  setup functions
void ledSetup() {
  DDRE |= (0x01 << BLUE_LED);
  DDRE |= (0x01 << RED_LED);
  DDRG |= (0x01 << YELLOW_LED);
  DDRE |= (0x01 << GREEN_LED);
}

void buttonSetup() {
  DDRD &= ~(0x01 << START_STOP);
  DDRD &= ~(0x01 << resetButton);
  DDRL &= ~(0x01 << ventCCW);
  DDRL &= ~(0x01 << ventCW);
}

void fanSetup() {
  DDRC |= (0x01 << speedPin);
  DDRC |= (0x01 << dir1);
  DDRC |= (0x01 << dir2);
}

// logic for when start/stop button is pressed
void startStop() {
  if (disabled) { // if disabled swap to idle state
    previousState = currentState;
    currentState = 1;
    idle = true;
    disabled = false;
    displayTemp();
  } else {  // switch to disabled if not already disabled
    previousState = currentState;
    currentState = 0;
    idle = false;
    disabled = true;
    running = false;
    error = false;
  }
}

void adc_init() {
  // setup the A register
  *my_ADCSRA |= 0b10000000;  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111;  // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111;  // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000;  // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111;  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000;  // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX &= 0b01111111;  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX |= 0b01000000;  // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX &= 0b11011111;  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11100000;  // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if (adc_channel_num > 7) {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0)
    ;
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void my_delay(unsigned int freq) {
  // calc period
  double period = 1.0 / double(freq);
  // 50% duty cycle
  double half_period = period / 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int)(65536 - ticks);
  // start the timer
  *myTCCR1A = 0x0;
  *myTCCR1B |= 0b00000001;
  // wait for overflow
  while ((*myTIFR1 & 0x01) == 0)
    ;  // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;  // 0b 0000 0000
  // reset TOV
  *myTIFR1 |= 0x01;
}

// controls if fan is enabled or disabled
void controlFan(bool power) {
  if (power) {
    PORTC |= (0x01 << dir1);
    PORTC &= ~(0x01 << dir2);
    analogWrite(31, 170);
  } else {
    PORTC &= ~(0x01 << dir1);
    PORTC &= ~(0x01 << dir2);
    analogWrite(31, 0);
  }
}

// displays error message on lcd screen
void errorMsg() {
  lcd.setCursor(0, 0);
  lcd.print("Water Level is");
  lcd.setCursor(0, 1);
  lcd.print("too low");
}

// displays state changes on serial monitor
void displayChange() {
  rtc.refresh();  // refresh rtc values

  // save rtc values
  int year = rtc.year();
  int month = rtc.month();
  int day = rtc.day();
  int hour = rtc.hour();
  int min = rtc.minute();
  int sec = rtc.second();

  // arrays for putChar calls
  char num[10] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
  char disabledState[8] = { 'D', 'I', 'S', 'A', 'B', 'L', 'E', 'D' };
  char idleState[4] = { 'I', 'D', 'L', 'E' };
  char runningState[7]{ 'R', 'U', 'N', 'N', 'I', 'N', 'G' };
  char errorState[5] = { 'E', 'R', 'R', 'O', 'R' };

  // display from
  putChar('F');
  putChar('r');
  putChar('o');
  putChar('m');
  putChar(' ');

  // display previous state
  switch (previousState) {
    case 0:
      for (int i = 0; i < 8; i++) {
        putChar(disabledState[i]);
      }
      break;
    case 1:
      for (int i = 0; i < 4; i++) {
        putChar(idleState[i]);
      }
      break;
    case 2:
      for (int i = 0; i < 7; i++) {
        putChar(runningState[i]);
      }
      break;
    case 3:
      for (int i = 0; i < 5; i++) {
        putChar(errorState[i]);
      }
      break;
  }

  // display to
  putChar(' ');
  putChar('t');
  putChar('o');
  putChar(' ');

  //  display current state
  switch (currentState) {
    case 0:
      for (int i = 0; i < 8; i++) {
        putChar(disabledState[i]);
      }
      break;
    case 1:
      for (int i = 0; i < 4; i++) {
        putChar(idleState[i]);
      }
      break;
    case 2:
      for (int i = 0; i < 7; i++) {
        putChar(runningState[i]);
      }
      break;
    case 3:
      for (int i = 0; i < 5; i++) {
        putChar(errorState[i]);
      }
      break;
  }

  // display at
  putChar(' ');
  putChar('a');
  putChar('t');
  putChar(' ');

  // display date mm/dd/yy
  putChar(num[(month / 10) % 10]);
  putChar(num[month % 10]);
  putChar('/');
  putChar(num[(day / 10) % 10]);
  putChar(num[day % 10]);
  putChar('/');
  putChar(num[(year / 10) % 10]);
  putChar(num[year % 10]);
  putChar(' ');

  // display time hh:mm:ss
  putChar(num[(hour / 10) % 10]);
  putChar(num[hour % 10]);
  putChar(':');
  putChar(num[(min / 10) % 10]);
  putChar(num[min % 10]);
  putChar(':');
  putChar(num[(sec / 10) % 10]);
  putChar(num[sec % 10]);
  putChar('\n');

  previousState = -1;
}