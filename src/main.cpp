//-------------------------------------------------------------------
//
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries
// by Brett Beauregard
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the Adafruit RGB/LCD Shield
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

//libraries for Encoder
#include <ByteBuffer.h>
#include <ooPinChangeInt.h>
#include <AdaEncoder.h>



// So we can save and retrieve settings
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
// modified to be pwm pin for ssr wire to 3 or + of fotek ssr-40da
#define RelayPin 13

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
// Bus is white/blue on my DS18B20
#define ONE_WIRE_BUS 3
//PWR is white/orange on DS18B20
#define ONE_WIRE_PWR 4
//gnd is white on DS18B20
#define ONE_WIRE_GND 5

// Quadrature Rotary encoder pins
// swap these if rotation has opposite result of expectation
#define ENCA_a 10
#define ENCA_b 11

// encoder pushbutton
#define ENCA_push 9
//set up variables for encoder checking
int clicks = 0;
//tell encoder library what/where encoder is
AdaEncoder encoderA = AdaEncoder('a', ENCA_a, ENCA_b);

//define pins to drive LEDs of buttons

#define LED_l A3
#define LED_r A2
#define LED_u A1
#define LED_d A0

double *adjustThis = NULL;

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double tempSetpoint;
bool doneYet = false;
double Input;
double Output;
double backCheck;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000;
unsigned long windowStartTime;
// variable for blink time
unsigned long prevMillis = 0;
int blinkInt = 500;
int ledState = HIGH;


// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember = 2;

double aTuneStep = 500;
double aTuneNoise = 1;
unsigned int aTuneLookBack = 20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// These #defines make it easy to set the backlight color
// trw- since this only works with RGB screen I need to alter to effectively have 1 or two states
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define BUTTON_SHIFT BUTTON_SELECT
 
unsigned long lastInput = 0; // last button press

byte degree[8] = // define the degree symbol
{
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
  pinMode(LED_l, OUTPUT);
  pinMode(LED_r, OUTPUT);
  pinMode(LED_u, OUTPUT);
  pinMode(LED_d, OUTPUT);

  Serial.begin(9600);
  //set pinmode for pushbutton
  pinMode(ENCA_push, INPUT_PULLUP);
  // Initialize Relay Control:

  pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(RelayPin, LOW);  // make sure it is off to start

  // Set up Ground & Power for the sensor from GPIO pins

  pinMode(ONE_WIRE_GND, OUTPUT);
  digitalWrite(ONE_WIRE_GND, LOW);

  pinMode(ONE_WIRE_PWR, OUTPUT);
  digitalWrite(ONE_WIRE_PWR, HIGH);

  // Initialize LCD DiSplay

  lcd.begin(16, 2);
  lcd.createChar(1, degree); // create degree symbol from the binary

  lcd.setBacklight(VIOLET);
  lcd.print(F("17.12.7.2|Tom's"));
  lcd.setCursor(0, 1);
  lcd.print(F("  Brewtroller"));

  // Start up the DS18B20 One Wire Temperature Sensor

  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0))
  {
    lcd.setCursor(0, 1);
    lcd.print(F("Sensor Error "));
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);

  delay(3000);  // Splash screen

  // Initialize the PID and related variables
  LoadParameters();
  myPID.SetTunings(Kp, Ki, Kd);

  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms
  TCCR2A = 0;
  TCCR2B = 1 << CS22 | 1 << CS21 | 1 << CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1 << TOIE2;
  Setpoint = 60;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect)
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
  // wait for button release before changing state
  while (ReadButtons() != 0) {}

  lcd.clear();

  switch (opState)
  {
    case OFF:
      Off();
      break;
    case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
    case TUNE_P:
      TuneP();
      break;
    case TUNE_I:
      TuneI();
      break;
    case TUNE_D:
      TuneD();
      break;
  }
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
  myPID.SetMode(MANUAL);
  lcd.setBacklight(WHITE);
  digitalWrite(RelayPin, LOW);  // make sure it is off
  lcd.print(F("OFF"));
  lcd.setCursor(0, 1);
  lcd.print(F("Temp: "));
  lcd.print(Input);
  lcd.setCursor(0, 8);
  uint8_t buttons = 0;
  digitalWrite(LED_l, LOW);
  digitalWrite(LED_r, LOW);
  digitalWrite(LED_u, LOW);
  digitalWrite(LED_d, LOW);

  while (!(buttons & (BUTTON_SHIFT)))
  {
    buttons = ReadButtons();
  }
  // Prepare to transition to the RUN state
  sensors.requestTemperatures(); // Start an asynchronous temperature reading

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  windowStartTime = millis();
  opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// NOW encoder to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************

void Run()
{
  // set up the LCD's number of rows and columns:
  lcd.print(F("Sp: "));
  lcd.print(Setpoint);
  lcd.write(1);
  lcd.print(F("F : "));
  

  SaveParameters();
  myPID.SetTunings(Kp, Ki, Kd);

  uint8_t buttons = 0;
  while (true)
  {
    setBacklight();  // set backlight based on state
    // future led button control
    digitalWrite(LED_l, HIGH); // Set temp
    digitalWrite(LED_r, HIGH); // Ki tune
    digitalWrite(LED_u, HIGH); // Kp tune
    digitalWrite(LED_d, HIGH); // Kd tune
    // BUTTON_LEFT = Set point, BUTTON_RIGHT= Ki Tune, BUTTON_UP = Kp tune, BUTTON_DOWN = Kd tune
    buttons = ReadButtons();



    /* previous conditional
      if ((buttons & BUTTON_SHIFT)
       && (buttons & BUTTON_RIGHT) */
    if ((buttons & BUTTON_SHIFT)
        && abs(Input - Setpoint) < 0.5)  // Should be at steady-state
    {
      StartAutoTune();
    }
    //Altered else if to indicate system wasn't in a a state for editing
    else if (abs(Input - Setpoint) > 0.5
             && (buttons & BUTTON_SHIFT))
    {
      lcd.clear();
      lcd.print(F("ERR: Not at SS"));
      delay (3000);
      opState = OFF;
      return;
    }
    //original else if below
    //else if ((buttons & BUTTON_RIGHT)
    //{
    // opState = SETP;
    //  return;
    //}
    //else if (buttons & BUTTON_LEFT)
    //{
    //  opState = OFF;
    //  return;
    //}
    else if (buttons & BUTTON_LEFT)
    {
      opState = SETP;
      return;
    }
    else if (buttons & BUTTON_RIGHT)
    {
      opState = TUNE_I;
      return;
    }
    else if (buttons & BUTTON_UP)
    {
      opState = TUNE_P;
      return;
    }
    else if (buttons & BUTTON_DOWN)
    {
      opState = TUNE_D;
      return;
    }
    DoControl();
    //update backCheck to be equal to any random encoder crap.
    backCheck = encoderA.query();
    lcd.setCursor(0, 1);
    lcd.print(Input);
    lcd.write(1);
    lcd.print(F("F : "));

    float pct = map(Output, 0, WindowSize, 0, 1000);
    lcd.setCursor(10, 1);
    lcd.print(F("      "));
    lcd.setCursor(10, 1);
    lcd.print(pct / 10);
    //lcd.print(Output);
    lcd.print("%");

    lcd.setCursor(15, 0);
    if (tuning)
    {
      lcd.print("T");
    }
    else
    {
      lcd.print(" ");
    }

    /* serial print function to log, not used
      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)
      {
      Serial.print(Input);
      Serial.print(",");
      Serial.println(Output);
      } */

    delay(100);
  }
}

void Tune_Sp()
{
  //print out what we're doing on the LCD
  lcd.setBacklight(GREEN);
  lcd.print(F("Set Temp.:"));
  uint8_t buttons = 0;
  //ideally I want this to collect current value of encoderA 
  //(to "clear' clicks from outside of this loop) but only do it once at start or before Tune_Sp() is invoked
  // I think I need to set a boolean that will be set to false after value is gathered. Effectively a 
  // 'run once' command.  But I've nto been able to make that work properly.  
 backCheck = encoderA.query();
  while (true)
  {
    
    buttons = ReadButtons();
    // blink button to indicate what we're changing
    digitalWrite (LED_l, ledState);
    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= blinkInt)
    {
      prevMillis = currentMillis;
      if (ledState == LOW)
      {
        ledState = HIGH;
      }
      else
      {
        ledState = LOW;
      }
      digitalWrite (LED_l, ledState);
    }

    //encoder code below
    //point to tempSetpoint as a buffer to keep controller from increasing output until verified. 
    adjustThis = &tempSetpoint;
    //gather info from encoderA.query
    // new encoder value - old encoder value gets a differential, if greater than 0 ++ if <0 --
    // it is flawed actually, if result is 0  would still increment I'm afraid.
    clicks = encoderA.query() - backCheck;
    //if clicks not equal to bachcheck adjust our buffered setpoint
    if (clicks != backCheck)
    {
      *adjustThis += (clicks > backCheck) ? -1.0 : 1.0;
    } 
    //othewise listen for button to change machine state. 
    // current mode is for left button, so another press will take us to run as cancel and keep previous value. 
    if (buttons & BUTTON_LEFT)
    {
      opState = RUN;
      return;
    }
    // adding additional code for new buttons
    if (buttons & BUTTON_RIGHT)
    {
      tempSetpoint = Setpoint;
      opState = TUNE_I;
      return;
    }
    if (buttons & BUTTON_DOWN)
    {
      tempSetpoint = Setpoint;
      opState = TUNE_D;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      tempSetpoint = Setpoint; 
      opState = TUNE_P;
      return;
    }

    if (digitalRead(ENCA_push) == LOW || buttons & BUTTON_LEFT)
    //if encoder button is down, run this
    { 
      //first, adjust setpoint to new value if less than 213 this keeps controller from potentially going into a runaway condiion
      if (tempSetpoint < 213)
      {
      Setpoint = tempSetpoint;
      }
      else //short redlight to indicate we set temp too high
      {
        lcd.setBacklight(RED);
        lcd.setCursor(0,0);
        lcd.print("TOO HIGH!");
        lcd.setCursor(0,1);
        lcd.print("CrazyPants!!!");
        Setpoint = 213;
        delay(3000);
      }
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    //print adjustThis as variable isn't set until button down
    lcd.print(tempSetpoint);
    lcd.print(" ");
    DoControl();
    
  }
  
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneP()
{
  unsigned long currentMillis = millis();
  lcd.setBacklight(RED);
  lcd.print(F("Set Kp"));

  uint8_t buttons = 0;
  while (true)
  {
    buttons = ReadButtons();
    
          // blink button code
    digitalWrite (LED_u, ledState);
    currentMillis = millis();
    if (currentMillis - prevMillis >= blinkInt)
    {
      prevMillis = currentMillis;
      if (ledState == LOW)
      {
        ledState = HIGH;
      }
      else
      {
        ledState = LOW;
      }
      digitalWrite (LED_u, ledState);
    }

    if (buttons & BUTTON_LEFT)
    {
      opState = SETP;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = TUNE_I;
      return;
    }
    if (buttons & BUTTON_DOWN)
    {
      opState = TUNE_D;
    }
    // adding code to return back to run if Kp
    if (buttons & BUTTON_UP)
    {
      opState = RUN;
    }
    // original code for button adjusting
    /*       if (buttons & BUTTON_UP)
           {
             Kp += increment;
             delay(200);
          }
          if (buttons & BUTTON_DOWN)
          {
             Kp -= increment;
             delay(200);
          }
    */
    // new code for encoder
    //make adjustThis equal to current Kp
    adjustThis = &Kp; 
    //encoder modifies adjustThis
    clicks = encoderA.query();
    if (clicks != 0)
    {
      *adjustThis += (clicks < 0) ? -1.0 : 1.0;
    }

    if (digitalRead(ENCA_push) == LOW) // return to RUN  if encoder button down
    { 
      //make Kp equal to new adjustThis value
      //Kp = adjustThis;
      //write new Kp to eeprom if different
      if (Kp != EEPROM_readDouble(KpAddress))
        {
          EEPROM_writeDouble(KpAddress, Kp);
        }
        //return to run state
      opState = RUN;
      return;
    }
    //if not down, display this stuff
    lcd.setCursor(0, 1);
    //showing adjustThis variable instead of Kp as until button down, Kp unchanged
    lcd.print(Kp);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
  lcd.setBacklight(YELLOW);
  lcd.print(F("Set Ki"));

  uint8_t buttons = 0;
  while (true)
  {
    buttons = ReadButtons();
    //code for blinking button active
    digitalWrite (LED_r, ledState);
    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= blinkInt)
    {
      prevMillis = currentMillis;
      if (ledState == LOW)
      {
        ledState = HIGH;
      }
      else
      {
        ledState = LOW;
      }
      digitalWrite (LED_r, ledState);
    }
    /*code for button adjustment
          float increment = 0.01;
          <-- if (buttons & BUTTON_SHIFT)
          {
            increment *= 10;
          }
    */
    if (buttons & BUTTON_LEFT)
    {
      opState = SETP;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = RUN;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      opState = TUNE_P;
      return;
    }
    if (buttons & BUTTON_DOWN)
    {
      opState = TUNE_D;
      return;
    }
    /*
          if (buttons & BUTTON_UP)
          {
             Ki += increment;
             delay(200);
          }
          if (buttons & BUTTON_DOWN)
          {
             Ki -= increment;
             delay(200);
          }
    */
    // New code for encoder adjustment
    //set adjustThis to equal Ki
    adjustThis = &Ki;
    clicks = encoderA.query();
    if (clicks != 0)
    {
      *adjustThis += (clicks < 0) ? -0.01 : 0.01;
    }
    if (digitalRead(ENCA_push) == LOW)  // return to RUN after  encoder button
    { 
      //first set Ki to our new value
      //Ki = adjustThis;
      //then write it to eeprom setting if it changed
      if (Ki != EEPROM_readDouble(KiAddress))
        {
          EEPROM_writeDouble(KiAddress, Ki);
        }
      //now return to Run state
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    //while we're adjusting, print out adjustThis
    lcd.print(Ki);
    lcd.print(" ");
    //and run a control loop so things keep moving
    DoControl();
  }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneD()
{
  lcd.setBacklight(BLUE);
  lcd.print(F("Set Kd"));

  uint8_t buttons = 0;
  while (true)
  {
    buttons = ReadButtons();
    //code for blinking button active
    digitalWrite (LED_d, ledState);
    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= blinkInt)
    {
      prevMillis = currentMillis;
      if (ledState == LOW)
      {
        ledState = HIGH;
      }
      else
      {
        ledState = LOW;
      }
      digitalWrite (LED_d, ledState);
    }



      /*      This is old adjustment code
            float increment = 0.01;
         if (buttons & BUTTON_SHIFT)
         {
           increment *= 10;
         } */

    if (buttons & BUTTON_LEFT)
    {
      opState = SETP;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = TUNE_I;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      opState = TUNE_P;
      return;
    }
    if (buttons & BUTTON_DOWN)
    {
      opState = RUN;
      return;
    }


    /*{
       Kd += increment;
       delay(200);
      }
      if (buttons & BUTTON_DOWN)
      {
       Kd -= increment;
       delay(200);
      }
    */
    //new encoder adjusting
    //first set adjustThis to Kd
    adjustThis = &Kd;
    clicks = encoderA.query();
    if (clicks != 0)
    {
      *adjustThis += (clicks < 0) ? -0.01 : 0.01;
    }
    if (digitalRead(ENCA_push) == LOW)  // return to RUN after encoder push
    { 
      //set Kd to the new value
      //Kd = adjustThis;
      //if Kd different from eeprom, replace with new value and write to eeprom
      if (Kd != EEPROM_readDouble(KdAddress))
        {
          EEPROM_writeDouble(KdAddress, Kd);
        }
      //now return to our run condition
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    //printing adjustThis as it is a stand in for Kd at this point
    lcd.print(Kd);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempF(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }

  if (tuning) // run the auto-tuner
  {
    if (aTune.Runtime()) // returns 'true' when done
    {
      FinishAutoTune();
    }
  }
  else // Execute control algorithm
  {
    myPID.Compute();
  }

  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output;
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if ((onTime > 100) && (onTime > (now - windowStartTime)))
  {
    digitalWrite(RelayPin, HIGH);
  }
  else
  {
    digitalWrite(RelayPin, LOW);
  }
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
  if (tuning)
  {
    lcd.setBacklight(VIOLET); // Tuning Mode
  }
  else if (abs(Input - Setpoint) > 5.0)
  {
    lcd.setBacklight(RED);  // High Alarm - off by more than 5 degree
  }
  else if (abs(Input - Setpoint) > 1.0)
  {
    lcd.setBacklight(YELLOW);  // Low Alarm - off by more than 1 degrees
  }
  else
  {
    lcd.setBacklight(GREEN);  // We're on target!
  }
}

// ************************************************
// Start the Auto-Tuning cycle b
// ************************************************

void StartAutoTune()
{
  // REmember the mode we were in
  ATuneModeRemember = myPID.GetMode();

  // set up the auto-tune parameters
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
  tuning = false;

  // Extract the auto-tune calculated parameters
  Kp = aTune.GetKp();
  Ki = aTune.GetKi();
  Kd = aTune.GetKd();

  // Re-tune the PID and revert to normal control mode
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(ATuneModeRemember);

  // Persist any changed parameters to EEPROM
  SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  uint8_t buttons = lcd.readButtons();
  if (buttons != 0)
  {
    lastInput = millis();
  }
  return buttons;
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  if (Setpoint != EEPROM_readDouble(SpAddress))
  {
    EEPROM_writeDouble(SpAddress, Setpoint);
  }
  if (Kp != EEPROM_readDouble(KpAddress))
  {
    EEPROM_writeDouble(KpAddress, Kp);
  }
  if (Ki != EEPROM_readDouble(KiAddress))
  {
    EEPROM_writeDouble(KiAddress, Ki);
  }
  if (Kd != EEPROM_readDouble(KdAddress))
  {
    EEPROM_writeDouble(KdAddress, Kd);
  }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
  Setpoint = EEPROM_readDouble(SpAddress);
  Kp = EEPROM_readDouble(KpAddress);
  Ki = EEPROM_readDouble(KiAddress);
  Kd = EEPROM_readDouble(KdAddress);

  // Use defaults if EEPROM values are invalid
  if (isnan(Setpoint))
  {
    Setpoint = 100;
  }
  if (isnan(Kp))
  {
    Kp = 850;
  }
  if (isnan(Ki))
  {
    Ki = 0.5;
  }
  if (isnan(Kd))
  {
    Kd = 0.1;
  }
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address++, *p++);
  }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(address++);
  }
  return value;
}

