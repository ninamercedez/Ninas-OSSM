// Header for DefaultValues for Stepchain
void stepchain(int _preptime = 0, int _amount = 1, int _posIn = 0, int _posOut = 0, int _accIn = 0, int _accOut = 0, int _velIn = 0,  int _velOut = 0,  int _pauseIn = 0, int _pauseOut = 0);

//#############################################################################
#define SECRET_SSID "..."
#define SECRET_PASS "..."

#define pinPul 16
#define pinDir 17
#define pinEnb 21
#define stepspermm 20
#define dist_max 150
#define vel_min 1
#define vel_max 600
#define acc_min 100
#define acc_max 5000
const bool reverseStepper = false;
const bool speedwithsliders = true;

//#############################################################################

#include <Arduino.h>
#include <ESPUI.h>
#include <WiFi.h>
#include <AVRStepperPins.h>
#include <common.h>
#include <FastAccelStepper.h>
#include <PoorManFloat.h>
#include <RampCalculator.h>
#include <RampGenerator.h>
#include <StepperISR.h>

//#############################################################################

// Global Variables - Stepper
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
int velocity;
int acceleration;

// Global Variables - Wifi
const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

// Global Variables - GUI
uint16_t BPMLabelId;
uint16_t StatusLabelId;
float bpm;
unsigned long timestamp_stpchain_start;
unsigned long timestamp_stpchain_stop;
unsigned long millispased;

uint16_t button1;

uint16_t tab1;
uint16_t tab2;
uint16_t tab3;
uint16_t tab4;
uint16_t tab5;

uint16_t posInLabelId;
uint16_t posOutLabelId;

uint16_t speedThrustInLabelId;
uint16_t speedThrustOutLabelId;
uint16_t accThrustInLabelId;
uint16_t accThrustOutLabelId;

uint16_t timesPauseInLabelId;
uint16_t timesPauseOutLabelId;


// Global Variables - Positions
int posIn = 50;
int posOut = 0;

// Global Variables - Speed
int speedThrustIn = 150;
int speedThrustOut = 150;

// Global Variables - Acc
int accThrustIn = 5000;
int accThrustOut = 5000;

// Global Variables - Times
int timesPauseIn = 0;
int timesPauseOut = 0;

// Global Variables - Flags
bool flag_statuson = false;
bool flag_torque = false;

// Global Variables - Stepchain
bool stepchain_busy = false;
bool waitactive = false;
int stp = 0;
int secondscount = 0;
int amountdone = 1;
int targetpos;
int waittime;
unsigned long timestamp1;
unsigned long timestamp2;

//#############################################################################
//ESP Callbacks - Main
void button_start(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      // Serial.println("Start");
      break;

    case B_UP:
      start();
      // Serial.println("Started");
      break;
  }
}

void button_stop(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      // Serial.println("Start");
      stopp();
      break;

    case B_UP:
      // Serial.println("Started");
      break;
  }
}

void switch_torque(Control *sender, int value) {
  switch (value) {
  case S_ACTIVE:
    Serial.print("Active:");
    enable_torque();
    break;

  case S_INACTIVE:
    Serial.print("Inactive");
    disable_torque();
    break;
  }

  Serial.print(" ");
  Serial.println(sender->id);
}

//#############################################################################
//ESP Callbacks - Settings - Position
void numberCall_posIn( Control* sender, int type ) {
  Serial.print("posIn: "); Serial.println( sender->value );
  posIn = (sender->value).toInt();
}

void numberCall_posOut( Control* sender, int type ) {
  Serial.print("posOut: "); Serial.println( sender->value );
  posOut = (sender->value).toInt();
}

//#############################################################################
//ESP Callbacks - Settings - Speed
void numberCall_speedThrustIn( Control* sender, int type ) {
  Serial.print("speedThrustIn: "); Serial.println( sender->value );
  if (speedwithsliders){
    speedThrustIn = map(sender->value.toInt(), 0, 100, vel_min, vel_max);
  }
  else {
    speedThrustIn = (sender->value).toInt();
  }
}

void numberCall_speedThrustOut( Control* sender, int type ) {
  Serial.print("speedThrustOut: "); Serial.println( sender->value );
  if (speedwithsliders){
    speedThrustOut = map(sender->value.toInt(), 0, 100, vel_min, vel_max);
  }
  else {
    speedThrustOut = (sender->value).toInt();
  }
}

//#############################################################################
//ESP Callbacks - Settings - Acceleration
void numberCall_accThrustIn( Control* sender, int type ) {
  Serial.print("accThrustIn: "); Serial.println( sender->value );
  accThrustIn = (sender->value).toInt();
}

void numberCall_accThrustOut( Control* sender, int type ) {
  Serial.print("accThrustOut: "); Serial.println( sender->value );
  accThrustOut = (sender->value).toInt();
}

//#############################################################################
//ESP Callbacks - Settings - Times
void numberCall_timesPauseOut( Control* sender, int type ) {
  Serial.print("timesPauseOut: "); Serial.println( sender->value );
  timesPauseOut = (sender->value).toInt();
}

void numberCall_timesPauseIn( Control* sender, int type ) {
  Serial.print("timesPauseIn: "); Serial.println( sender->value );
  timesPauseIn = (sender->value).toInt();
}

//#############################################################################
//Functions for this Sketch
void start() {
  flag_statuson = true;
}

void stopp() {
  flag_statuson = false;
  ESPUI.print(BPMLabelId, "-");
}

void disable_torque(){
  digitalWrite(pinEnb, HIGH);
  flag_statuson = false;
  flag_torque = false;
  stepper->setCurrentPosition(0);
  stepchain_busy = false;
  stp = 0;
}

void enable_torque(){
  digitalWrite(pinEnb, LOW);
  flag_statuson = false;
  flag_torque = true;
  stepper->setCurrentPosition(0);
  stepchain_busy = false;
  stp = 0;
}

int calc_maxPossSpeed(int _acc, int _posin, int _posout){
  int dist = _posout - _posin;
  int maxSpeed = sqrt(_acc * dist);
  return maxSpeed;
}

void stepchain(int _preptime, int _amount, int _posIn, int _posOut, int _accIn, int _accOut, int _velIn,  int _velOut,  int _pauseIn, int _pauseOut) {

  if (stepchain_busy && flag_statuson) {
    switch (stp) {

      case 0: // Preparetime
        if (waitactive) {
          timestamp2 = millis();
          if ((timestamp2 - timestamp1) >= 1000 ) {
            timestamp1 = millis();
            secondscount++;
            if (_preptime > 0) {
              //ESPUI.print(TimerLabelId, "Get Ready: " + String(_preptime - secondscount) + " Seconds");
            }
            if (secondscount >= _preptime) {
              waitactive = false;
              if (_amount > 1) {
                //ESPUI.print(TimerLabelId, String(_amount));
              }
              stp = 1;
            }
          }
        }
        else {
          secondscount = 0;
          timestamp1 = millis();
          if (_preptime > 0) {
            //ESPUI.print(TimerLabelId, "Get Ready: " + String(_preptime) + " Seconds");
            waitactive = true;
          }
          else {
            stp = 1;
          }
        }
        break;

      case 1: //Write Parameters for in
        Serial.print("Stepchain: "); Serial.println(stp);

        timestamp_stpchain_stop = millis();
        millispased = (timestamp_stpchain_stop - timestamp_stpchain_start);
        if (millispased < 1){
          millispased = 1;
        }
        bpm = 60000.0/millispased;      
        ESPUI.print(BPMLabelId, String(bpm));
        timestamp_stpchain_start = millis();      
        
        targetpos = _posIn;
        stepper->setSpeedInHz(_velIn);
        stepper->setAcceleration(_accIn);
        stepper->applySpeedAcceleration();
        stp = 2;
        break;

      case 2: //Execute movement for in
        Serial.print("Stepchain: "); Serial.println(stp);
        stepper->moveTo(targetpos);
        stp = 3;
        break;

      case 3: // Check if in pos
        if (stepper->getCurrentPosition() == targetpos) {
          Serial.print("Stepchain: "); Serial.println(stp);
          stp = 4;
        }
        break;

      case 4: // Wait inside
        Serial.print("Stepchain: "); Serial.println(stp);
        if (waitactive) {
          timestamp2 = millis();
          if ((timestamp2 - timestamp1) >= 1000 ) {
            timestamp1 = millis();
            secondscount++;
            if (_pauseIn > 0) {
              //ESPUI.print(TimerLabelId, String(_pauseIn - secondscount) + " Seconds");
            }
            if (secondscount >= _pauseIn) {
              waitactive = false;
              stp = 5;
            }
          }
        }
        else {
          secondscount = 0;
          timestamp1 = millis();
          if (_pauseIn > 0) {
            //ESPUI.print(TimerLabelId, String(_pauseIn) + " Seconds");
            waitactive = true;
          }
          else {
            stp = 5;
          }
        }
        break;

      case 5: //Write Parameters for out
        Serial.print("Stepchain: "); Serial.println(stp);
        targetpos = _posOut;
        stepper->setSpeedInHz(_velOut);
        stepper->setAcceleration(_accOut);
        stepper->applySpeedAcceleration();
        stp = 6;
        break;

      case 6: //Execute movement for out
        Serial.print("Stepchain: "); Serial.println(stp);
        stepper->moveTo(targetpos);
        stp = 7;
        break;

      case 7: // Check if in pos
        if (stepper->getCurrentPosition() == targetpos) {
          Serial.print("Stepchain: "); Serial.println(stp);
          stp = 8;
        }
        break;

      case 8: // Wait outside
        Serial.print("Stepchain: "); Serial.println(stp);
        if (waitactive) {
          timestamp2 = millis();
          if ((timestamp2 - timestamp1) >= 1000 ) {
            timestamp1 = millis();
            secondscount++;
            if (_pauseOut > 0) {
              //ESPUI.print(TimerLabelId, String(_pauseOut - secondscount) + " Seconds");
            }
            if (secondscount >= _pauseOut) {
              waitactive = false;
              stp = 9;
            }
          }
        }
        else {
          secondscount = 0;
          timestamp1 = millis();
          if (_pauseOut > 0) {
            //ESPUI.print(TimerLabelId, String(_pauseOut) + " Seconds");
            waitactive = true;
          }
          else {
            stp = 9;
          }
        }
        break;

      case 9: // Rest flags
        amountdone++;
        if (amountdone >= _amount) {
          Serial.print("Stepchain: "); Serial.println(stp);
          Serial.println("Stepchain finished");
          stepchain_busy = false;
          amountdone = 0;
          stp = 0;
        }
        else {
          Serial.print("Stepchain: "); Serial.println(stp);
          Serial.print(_amount - amountdone); Serial.println(" to go. --> Stepchain returning");
          stp = 1;
        }
        break;
    }
  }
}

//#############################################################################
// Basic Loops
void setup(void) {
  //just for debugging
  Serial.begin(115200);
  //
  pinMode(pinEnb, OUTPUT);
  digitalWrite(pinEnb, LOW);


  //#############################################################################
  //----------- fastaccelstepper stuff
  engine.init(); // houston, we have ingnition!
  stepper = engine.stepperConnectToPin(pinPul); // tells Fastaccelstepper what pins to tickle for step/pulse, dir, and enable
  if (stepper) {
    stepper->setDirectionPin(pinDir);
    stepper->setEnablePin(pinEnb);
    stepper->setAutoEnable(false); // if set true, Fastaccelstepper disables the motor if it's not moving which means it won't push back
    stepper->setSpeedInHz(vel_min);
    stepper->setAcceleration(50);
  }

  //#############################################################################
  //Begin the Wifi
  WiFi.begin(ssid, password);
  delay(2000);
  Serial.println(WiFi.localIP());
  //#############################################################################
  // ESPUI stuff
  ESPUI.setVerbosity(Verbosity::Quiet);

  tab1 = ESPUI.addControl( ControlType::Tab, "Status", "Status" );
  tab2 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Position" );
  tab3 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Speed" );
  tab4 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Acceleration" );
  tab5 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Times" );

  // shown above all tabs
  BPMLabelId = ESPUI.label("BPM:", ControlColor::Sunflower, "-");
  ESPUI.button("Start", &button_start, ControlColor::Sunflower, "Start");
  ESPUI.button("Stop", &button_stop, ControlColor::Sunflower, "Stop");
  
  // tab 1
  StatusLabelId = ESPUI.addControl( ControlType::Label, "Status", "-", ControlColor::Alizarin, tab1 );
  ESPUI.addControl( ControlType::Switcher, "Torque", "", ControlColor::Alizarin, tab1, &switch_torque );

  // tab 2
  posInLabelId = ESPUI.addControl( ControlType::Number, "Position In", String(posIn), ControlColor::Alizarin, tab2, &numberCall_posIn );
  ESPUI.addControl( ControlType::Min, "Position In", String(0), ControlColor::None, posInLabelId );
  ESPUI.addControl( ControlType::Max, "Position In", String(dist_max), ControlColor::None, posInLabelId );

  posOutLabelId = ESPUI.addControl( ControlType::Number, "Position Out", String(posOut), ControlColor::Alizarin, tab2, &numberCall_posOut );
  ESPUI.addControl( ControlType::Min, "Position Out", String(0), ControlColor::None, posOutLabelId );
  ESPUI.addControl( ControlType::Max, "Position Out", String(dist_max), ControlColor::None, posOutLabelId );

  // tab 3
  if (speedwithsliders) {
    ESPUI.addControl( ControlType::Slider, "Speed Thrust In", "10", ControlColor::Alizarin, tab3, &numberCall_speedThrustIn );
    ESPUI.addControl( ControlType::Slider, "Speed Thrust Out", "10", ControlColor::Alizarin, tab3, &numberCall_speedThrustOut );
  }
  else{
    speedThrustInLabelId = ESPUI.addControl( ControlType::Number, "Speed Thrust In", String(speedThrustIn), ControlColor::Alizarin, tab3, &numberCall_speedThrustIn );
    ESPUI.addControl( ControlType::Min, "Speed Thrust In", String(vel_min), ControlColor::None, speedThrustInLabelId );
    ESPUI.addControl( ControlType::Max, "Speed Thrust In", String(vel_max), ControlColor::None, speedThrustInLabelId );
  
    speedThrustOutLabelId = ESPUI.addControl( ControlType::Number, "Speed Thrust Out", String(speedThrustOut), ControlColor::Alizarin, tab3, &numberCall_speedThrustOut );
    ESPUI.addControl( ControlType::Min, "Speed Thrust Out", String(vel_min), ControlColor::None, speedThrustOutLabelId );
    ESPUI.addControl( ControlType::Max, "Speed Thrust Out", String(vel_max), ControlColor::None, speedThrustOutLabelId );
  }
  // tab 4
  accThrustInLabelId = ESPUI.addControl( ControlType::Number, "Acceleration Thrust In", String(accThrustIn), ControlColor::Alizarin, tab4, &numberCall_accThrustIn );
  ESPUI.addControl( ControlType::Min, "Acceleration Thrust In", String(acc_min), ControlColor::None, accThrustInLabelId );
  ESPUI.addControl( ControlType::Max, "Acceleration Thrust In", String(acc_max), ControlColor::None, accThrustInLabelId );

  accThrustOutLabelId = ESPUI.addControl( ControlType::Number, "Acceleration Thrust Out", String(accThrustOut), ControlColor::Alizarin, tab4, &numberCall_accThrustOut );
  ESPUI.addControl( ControlType::Min, "Acceleration Thrust Out", String(acc_min), ControlColor::None, accThrustOutLabelId );
  ESPUI.addControl( ControlType::Max, "Acceleration Thrust Out", String(acc_max), ControlColor::None, accThrustOutLabelId );
  
  // tab 5
  timesPauseInLabelId = ESPUI.addControl( ControlType::Number, "Wait Inside", String(timesPauseIn), ControlColor::Alizarin, tab5, &numberCall_timesPauseIn );
  ESPUI.addControl( ControlType::Min, "Wait Inside", String(0), ControlColor::None, timesPauseInLabelId );
  ESPUI.addControl( ControlType::Max, "Wait Inside", String(30), ControlColor::None, timesPauseInLabelId );

  timesPauseOutLabelId = ESPUI.addControl( ControlType::Number, "Wait Outside", String(timesPauseOut), ControlColor::Alizarin, tab5, &numberCall_timesPauseOut );
  ESPUI.addControl( ControlType::Min, "Wait Outside", String(0), ControlColor::None, timesPauseOutLabelId );
  ESPUI.addControl( ControlType::Max, "Wait Outside", String(30), ControlColor::None, timesPauseOutLabelId );

  // I dont think its a good idea to enable this...
  //ESPUI.sliderContinuous = true;
  ESPUI.jsonInitialDocumentSize = 16000; // Default is 8000. Thats not enough for this many widgeds
  ESPUI.begin("OSSM");
}

void loop(void) {
  if (flag_statuson && flag_torque) {
    stepchain_busy = true;
    stepchain(0, 1, -posIn * stepspermm, -posOut * stepspermm, accThrustIn * stepspermm, accThrustOut * stepspermm, speedThrustIn * stepspermm, speedThrustOut * stepspermm, timesPauseIn, timesPauseOut);
  }
}
