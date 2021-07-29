// Header for DefaultValues for Stepchain
void stepchain(int _preptime = 0, int _amount = 1, int _posIn = 0, int _posOut = 0, int _accIn = 0, int _accOut = 0, int _velIn = 0,  int _velOut = 0,  int _pauseIn = 0, int _pauseOut = 0);

//#############################################################################
#define SECRET_SSID "add_your_ssid"
#define SECRET_PASS "add_your_pw"

#define pinPul 16
#define pinDir 17
#define pinEnb 21
#define stepspermm 20
#define dist_max 150
#define vel_min 1
#define vel_max 600
const bool reverseStepper = false;

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
int ProgramLabelId;
int CurrentTaskLabelId;
int TimerLabelId;
int PrevTaskLabelId;
int tasknumber = 1;

uint16_t pos1LabelId;
uint16_t pos2LabelId;
uint16_t pos3LabelId;
uint16_t pos4LabelId;
uint16_t pos5LabelId;
uint16_t pos6LabelId;

uint16_t speedHoldInMinLabelId;
uint16_t speedHoldOutMinLabelId;
uint16_t speedThrustInMinLabelId;
uint16_t speedThrustOutMinLabelId;
uint16_t speedHoldInMaxLabelId;
uint16_t speedHoldOutMaxLabelId;
uint16_t speedThrustInMaxLabelId;
uint16_t speedThrustOutMaxLabelId;

uint16_t probabilityPauseLabelId;
uint16_t probabilityHeavyLabelId;
uint16_t probabilityHoldLabelId;

// Global Variables - Positions
int posGap = 0;
int posLip = 20;
int posTip = 50;
int posEnt = 100;
int posThr = 120;
int posBde = 140;

// Global Variables - Speed
int speedHoldInMin = 200;
int speedHoldOutMin = 200;
int speedThrustInMin = 400;
int speedThrustOutMin = 400;
int speedHoldInMax = 200;
int speedHoldOutMax = 200;
int speedThrustInMax = 400;
int speedThrustOutMax = 400;

// Global Variables - Times

// Global Variables - Probability
int prob_pause = 30;
int prob_task; // =100 - prob_pause = 70
int prob_heavytask = 30;
int prob_hold = 40;

// Global Variables - Flags
bool flag_statuson = false;
bool flag_hold = false;
bool flag_thrust = false;
bool flag_paus = false;

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
String msg;

// Define the Struct 'command'
struct command {
  char taskidentifier;
  int cmd_preptime;
  int cmd_amount;
  int cmd_posin;
  int cmd_posout;
  int cmd_accin;
  int cmd_accout;
  int cmd_velin;
  int cmd_velout;
  int cmd_pausein;
  int cmd_pauseout;
  String cmd_message;
  String cmd_hint;
};

// Declare variable 'activecommand' as struct 'command'
command activecommand;
command prevcommands[5];

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

//#############################################################################
//ESP Callbacks - Settings - Position
void numberCall_posGap( Control* sender, int type ) {
  Serial.print("posGap: "); Serial.println( sender->value );
  posGap = (sender->value).toInt();
}

void numberCall_posLip( Control* sender, int type ) {
  Serial.print("posLip: "); Serial.println( sender->value );
  posLip = (sender->value).toInt();
}

void numberCall_posTip( Control* sender, int type ) {
  Serial.print("posTip: "); Serial.println( sender->value );
  posTip = (sender->value).toInt();
}

void numberCall_posEnt( Control* sender, int type ) {
  Serial.print("posEnt: "); Serial.println( sender->value );
  posEnt = (sender->value).toInt();
}

void numberCall_posThr( Control* sender, int type ) {
  Serial.print("posThr: "); Serial.println( sender->value );
  posThr = (sender->value).toInt();
}

void numberCall_posBde( Control* sender, int type ) {
  Serial.print("posBde: "); Serial.println( sender->value );
  posBde = (sender->value).toInt();
}

//#############################################################################
//ESP Callbacks - Settings - Speed
void numberCall_speedHoldInMin( Control* sender, int type ) {
  Serial.print("speedHoldInMin: "); Serial.println( sender->value );
  speedHoldInMin = (sender->value).toInt();
}

void numberCall_speedHoldOutMin( Control* sender, int type ) {
  Serial.print("speedHoldOutMin: "); Serial.println( sender->value );
  speedHoldOutMin = (sender->value).toInt();
}

void numberCall_speedThrustInMin( Control* sender, int type ) {
  Serial.print("speedThrustInMin: "); Serial.println( sender->value );
  speedThrustInMin = (sender->value).toInt();
}

void numberCall_speedThrustOutMin( Control* sender, int type ) {
  Serial.print("speedThrustOutMin: "); Serial.println( sender->value );
  speedThrustOutMin = (sender->value).toInt();
}

void numberCall_speedHoldInMax( Control* sender, int type ) {
  Serial.print("speedHoldInMax: "); Serial.println( sender->value );
  speedHoldInMax = (sender->value).toInt();
}

void numberCall_speedHoldOutMax( Control* sender, int type ) {
  Serial.print("speedHoldOutMax: "); Serial.println( sender->value );
  speedHoldOutMax = (sender->value).toInt();
}

void numberCall_speedThrustInMax( Control* sender, int type ) {
  Serial.print("speedThrustInMax: "); Serial.println( sender->value );
  speedThrustInMax = (sender->value).toInt();
}

void numberCall_speedThrustOutMax( Control* sender, int type ) {
  Serial.print("speedThrustOutMax: "); Serial.println( sender->value );
  speedThrustOutMax = (sender->value).toInt();
}

//#############################################################################
//ESP Callbacks - Settings - Times

//#############################################################################
//ESP Callbacks - Settings - Probability
void numberCall_probPause( Control* sender, int type ) {
  Serial.print("Probability - Pause: "); Serial.println( sender->value );
  prob_pause = (sender->value).toInt();
}

void numberCall_probHeavy( Control* sender, int type ) {
  Serial.print("Probability - HeavyTask: "); Serial.println( sender->value );
  prob_heavytask = (sender->value).toInt();
}

void numberCall_probHold( Control* sender, int type ) {
  Serial.print("Probability - Hold: "); Serial.println( sender->value );
  prob_hold = (sender->value).toInt();
}


//#############################################################################

struct command task_pause(){
  struct command taskcmd;
  
  taskcmd.taskidentifier = 'P';
  taskcmd.cmd_preptime = 0;
  taskcmd.cmd_amount = 1;
  taskcmd.cmd_posin = 0;
  taskcmd.cmd_posout = 0;
  taskcmd.cmd_accin = 0;
  taskcmd.cmd_accout = 0;
  taskcmd.cmd_velin = 0;
  taskcmd.cmd_velout = 0;
  taskcmd.cmd_pausein = 0;
  taskcmd.cmd_pauseout = random(5, 11);
  taskcmd.cmd_message = "Pause for " + String(taskcmd.cmd_pauseout);
  taskcmd.cmd_hint = "Catch your breath";

  return taskcmd;
}

struct command task_hold(bool _heavy){
  struct command taskcmd;
  int posin_dice;
  int posout_dice;
  String message;
  String hint;

  taskcmd.taskidentifier = 'H';
  
  if (_heavy){
    posin_dice = random(5, 7);
    posout_dice = random(1, posin_dice);
    taskcmd.cmd_preptime = 5;
  }
  else {
    posin_dice = random(1, 5);
    posout_dice = random(1, posin_dice);
    taskcmd.cmd_preptime = 0;
  }
  taskcmd.cmd_amount = 1;
  switch (posin_dice) { //cmd_posin
    case 1:
      taskcmd.cmd_posin = posGap;
      message = "Wait with your open Mouth for ";
      break;

    case 2:
      taskcmd.cmd_posin = posLip;
      message = "Lick the Tip for ";
      break;

    case 3:
      taskcmd.cmd_posin = posTip;
      message = "Hold the Tip in your Mouth for ";
      break;

    case 4:
      taskcmd.cmd_posin = posEnt;
      message = "Hold it at the Entrance of your Throat for ";
      break;

    case 5:
      taskcmd.cmd_posin = posThr;
      message = "Hold it past the Entrance of your Throat for ";
      break;

    case 6:
      taskcmd.cmd_posin = posBde;
      message = "Hold it ballsdeep for ";
      break;
  }
  taskcmd.cmd_posout = 0;
  taskcmd.cmd_accin = 5000;
  taskcmd.cmd_accout = 5000;
  taskcmd.cmd_velin = random(speedHoldInMin, speedHoldInMax);
  taskcmd.cmd_velout = random(speedHoldOutMin, speedHoldOutMax);
  taskcmd.cmd_pausein = random(5, 16);
  taskcmd.cmd_pauseout = 0;
  taskcmd.cmd_message = message + String(taskcmd.cmd_pausein) + " Seconds" ;
  taskcmd.cmd_hint = "No Air";
        
  return taskcmd;
}

struct command task_thrust(bool _heavy){
  struct command taskcmd;
  int posin_dice;
  int posout_dice;
  String str_posin;
  String str_posout;
  String message;
  String hint;

  taskcmd.taskidentifier = 'T';
  if (_heavy){
    posin_dice = random(5, 7);
    posout_dice = random(1, posin_dice);
    taskcmd.cmd_preptime = 5;
  }
  else {
    posin_dice = random(3, 5);
    posout_dice = random(1, posin_dice);
    taskcmd.cmd_preptime = 0;
  } 
  taskcmd.cmd_amount = random(5, 16);
  switch (posin_dice) { //cmd_posin
    case 1:
      taskcmd.cmd_posin = posGap;
      str_posin = "";
      break;

    case 2:
      taskcmd.cmd_posin = posLip;
      str_posin = "to your Lips";
      break;

    case 3:
      taskcmd.cmd_posin = posTip;
      str_posin = "to the Tip";
      break;

    case 4:
      taskcmd.cmd_posin = posEnt;
      str_posin = "to the Entrace of your Throat";
      break;

    case 5:
      taskcmd.cmd_posin = posThr;
      str_posin = "to the Back of your Throat";
      break;

    case 6:
      taskcmd.cmd_posin = posBde;
      str_posin = "Ballsdeep in your Throat";
      break;
  }
  switch (posout_dice) { //cmd_posout
    case 1:
      taskcmd.cmd_posout = posGap;
      str_posout = "outside your Mouth ";
      break;

    case 2:
      taskcmd.cmd_posout = posLip;
      str_posout = "your Lips ";
      break;

    case 3:
      taskcmd.cmd_posout = posTip;
      str_posout = "the Tip ";
      break;

    case 4:
      taskcmd.cmd_posout = posEnt;
      str_posout = "the Entrace of your Throat ";
      break;

    case 5:
      taskcmd.cmd_posout = posThr;
      str_posout = "the Back of your Throat ";
      break;

    case 6:
      taskcmd.cmd_posout = posBde;
      str_posout = "";
      break;
  }
  taskcmd.cmd_accin = 10000;
  taskcmd.cmd_accout = 10000;
  taskcmd.cmd_velin = random(speedThrustInMin, speedThrustInMax);
  taskcmd.cmd_velout = random(speedThrustOutMin, speedThrustOutMax);
  taskcmd.cmd_pausein = 0;
  taskcmd.cmd_pauseout = 0;
  taskcmd.cmd_message = "Suck it from " + str_posout + str_posin + " for " + String(taskcmd.cmd_amount) + " Times" ;
  taskcmd.cmd_hint = "In - Out";

  return taskcmd;
}

bool alea(int _probability) {
  if (random(0, 100) < _probability) {
    return true;
  }
  else {
    return false;
  }
}

struct command randomizeTask() {
  struct command meincmd;
  bool nxtTaskIsPause = alea(prob_pause) && (prevcommands[0].taskidentifier != 'P');
  bool nxtTaskIsHeavy = alea(prob_heavytask);
  bool nxtTaskIsHold = alea(prob_hold);

  if (nxtTaskIsPause) { //Pause
    meincmd = task_pause();
  }
  else {  // Task
    if (nxtTaskIsHold){ // Hold
      meincmd = task_hold(nxtTaskIsHeavy);
    }
    else{ // Thrust
      meincmd = task_thrust(nxtTaskIsHeavy);
    }
  }

  if (reverseStepper) {
    meincmd.cmd_posin = -meincmd.cmd_posin;
    meincmd.cmd_posout = -meincmd.cmd_posin;
  }

  return meincmd;
}

void printTask() { //Just for Debugging
  Serial.print(String(activecommand.taskidentifier) + ' ');
  Serial.print(String(activecommand.cmd_preptime) + ' ');
  Serial.print(String(activecommand.cmd_amount) + ' ');
  Serial.print(String(activecommand.cmd_posin) + ' ');
  Serial.print(String(activecommand.cmd_posout) + ' ');
  Serial.print(String(activecommand.cmd_accin) + ' ');
  Serial.print(String(activecommand.cmd_accout) + ' ');
  Serial.print(String(activecommand.cmd_velin) + ' ');
  Serial.print(String(activecommand.cmd_velout) + ' ');
  Serial.print(String(activecommand.cmd_pausein) + ' ');
  Serial.print(String(activecommand.cmd_pauseout) + ' ');
  Serial.print(activecommand.cmd_message + " ");
  Serial.println(activecommand.cmd_hint);
}

void taskHandler() {
  switch (activecommand.taskidentifier) {

    case 'P':
      Serial.println("Cmd is P");
      flag_paus = true;
      break;

    case 'H':
      Serial.println("Cmd is H");
      flag_hold = true;;
      break;

    case 'T':
      Serial.println("Cmd is T");
      flag_thrust = true;
      break;

    case 'F':
      Serial.println("Cmd is F");
      //
      break;

    default:
      Serial.println("No Cmd");
      stopp();
      break;
  }
  msg = activecommand.cmd_message;
  ESPUI.print(CurrentTaskLabelId, msg);
}

void start() {
  digitalWrite(pinEnb, LOW);
  flag_statuson = true;
  tasknumber = 1;
  ESPUI.print(ProgramLabelId, String(tasknumber));
  activecommand = randomizeTask();
  printTask();
  taskHandler();
}

void stopp() {
  flag_statuson = false;
  flag_hold = false;
  flag_thrust = false;
  flag_paus = false;
  digitalWrite(pinEnb, HIGH);
  stepper->setCurrentPosition(0);
  stepchain_busy = false;
  stp = 0;
  ESPUI.print(CurrentTaskLabelId, "- - -");
}

void nextTask() {

  prevcommands[4] = prevcommands[3];
  prevcommands[3] = prevcommands[2];
  prevcommands[2] = prevcommands[1];
  prevcommands[1] = prevcommands[0];
  prevcommands[0] = activecommand;
  Serial.print("Previous Commands: ");
  Serial.print(String(prevcommands[0].taskidentifier));
  Serial.print(String(prevcommands[1].taskidentifier));
  Serial.print(String(prevcommands[2].taskidentifier));
  Serial.print(String(prevcommands[3].taskidentifier));
  Serial.println(String(prevcommands[4].taskidentifier));

  ESPUI.print(PrevTaskLabelId, String(prevcommands[0].taskidentifier) + " " + String(prevcommands[1].taskidentifier) + " " + String(prevcommands[2].taskidentifier) + " " + String(prevcommands[3].taskidentifier) + " " + String(prevcommands[4].taskidentifier));

  tasknumber++;
  ESPUI.print(ProgramLabelId, String(tasknumber));
  activecommand = randomizeTask();
  printTask();
  taskHandler();

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
              ESPUI.print(TimerLabelId, "Get Ready: " + String(_preptime - secondscount) + " Seconds");
            }
            if (secondscount >= _preptime) {
              waitactive = false;
              if (_amount > 1) {
                ESPUI.print(TimerLabelId, String(_amount));
              }
              stp = 1;
            }
          }
        }
        else {
          secondscount = 0;
          timestamp1 = millis();
          if (_preptime > 0) {
            ESPUI.print(TimerLabelId, "Get Ready: " + String(_preptime) + " Seconds");
            waitactive = true;
          }
          else {
            stp = 1;
          }
        }
        break;

      case 1: //Write Parameters for in
        Serial.print("Stepchain: "); Serial.println(stp);
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
        if (waitactive) {
          timestamp2 = millis();
          if ((timestamp2 - timestamp1) >= 1000 ) {
            timestamp1 = millis();
            secondscount++;
            if (_pauseIn > 0) {
              ESPUI.print(TimerLabelId, String(_pauseIn - secondscount) + " Seconds");
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
            ESPUI.print(TimerLabelId, String(_pauseIn) + " Seconds");
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
        if (waitactive) {
          timestamp2 = millis();
          if ((timestamp2 - timestamp1) >= 1000 ) {
            timestamp1 = millis();
            secondscount++;
            if (_pauseOut > 0) {
              ESPUI.print(TimerLabelId, String(_pauseOut - secondscount) + " Seconds");
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
            ESPUI.print(TimerLabelId, String(_pauseOut) + " Seconds");
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
          flag_hold = false;
          flag_thrust = false;
          flag_paus = false;
          stepchain_busy = false;
          amountdone = 0;
          stp = 0;
          ESPUI.print(TimerLabelId, "Next Task" );
          nextTask();
        }
        else {
          Serial.print("Stepchain: "); Serial.println(stp);
          Serial.print(_amount - amountdone); Serial.println(" to go. --> Stepchain returning");
          ESPUI.print(TimerLabelId, String(_amount - amountdone));
          stp = 1;
        }
        break;
    }
  }
}

//#############################################################################

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

  uint16_t tab1 = ESPUI.addControl( ControlType::Tab, "Main", "Main" );
  uint16_t tab2 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Position" );
  uint16_t tab3 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Speed" );
  uint16_t tab4 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Times" );
  uint16_t tab5 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Probability" );


  // shown above all tabs
  ProgramLabelId = ESPUI.addControl( ControlType::Label, "Tasknumber", "-", ControlColor::Emerald );
  CurrentTaskLabelId = ESPUI.addControl( ControlType::Label, "Task", "-", ControlColor::Emerald );
  TimerLabelId = ESPUI.addControl( ControlType::Label, "Timer", "-", ControlColor::Emerald );

  // tab 1
  ESPUI.addControl( ControlType::Button, "Start", "Start", ControlColor::Sunflower, tab1, &button_start );
  ESPUI.addControl( ControlType::Button, "Stop", "Stop", ControlColor::Sunflower, tab1, &button_stop );
  PrevTaskLabelId = ESPUI.addControl( ControlType::Label, "Prevoius Tasks", "- - - - -", ControlColor::Sunflower, tab1 );

  // tab 2
  pos1LabelId = ESPUI.addControl( ControlType::Number, "Position 1 - Not Touching", String(posGap), ControlColor::Alizarin, tab2, &numberCall_posGap );
  ESPUI.addControl( ControlType::Min, "Position 1 - Not Touching", String(0), ControlColor::None, pos1LabelId );
  ESPUI.addControl( ControlType::Max, "Position 1 - Not Touching", String(dist_max), ControlColor::None, pos1LabelId );

  pos2LabelId = ESPUI.addControl( ControlType::Number, "Position 2 - On Lips", String(posLip), ControlColor::Alizarin, tab2, &numberCall_posLip );
  ESPUI.addControl( ControlType::Min, "Position 2 - On Lips", String(0), ControlColor::None, pos2LabelId );
  ESPUI.addControl( ControlType::Max, "Position 2 - On Lips", String(dist_max), ControlColor::None, pos2LabelId );

  pos3LabelId = ESPUI.addControl( ControlType::Number, "Position 3 - Just the Tip", String(posTip), ControlColor::Alizarin, tab2, &numberCall_posTip );
  ESPUI.addControl( ControlType::Min, "Position 3 - Just the Tip", String(0), ControlColor::None, pos3LabelId );
  ESPUI.addControl( ControlType::Max, "Position 3 - Just the Tip", String(dist_max), ControlColor::None, pos3LabelId );

  pos4LabelId = ESPUI.addControl( ControlType::Number, "Position 4 - Throat Entrance", String(posEnt), ControlColor::Alizarin, tab2, &numberCall_posEnt );
  ESPUI.addControl( ControlType::Min, "Position 4 - Throat Entrance", String(0), ControlColor::None, pos4LabelId );
  ESPUI.addControl( ControlType::Max, "Position 4 - Throat Entrance", String(dist_max), ControlColor::None, pos4LabelId );

  pos5LabelId = ESPUI.addControl( ControlType::Number, "Position 5 - Past Entrance", String(posThr), ControlColor::Alizarin, tab2, &numberCall_posThr );
  ESPUI.addControl( ControlType::Min, "Position 5 - Past Entrance", String(0), ControlColor::None, pos5LabelId );
  ESPUI.addControl( ControlType::Max, "Position 5 - Past Entrance", String(dist_max), ControlColor::None, pos5LabelId );

  pos6LabelId = ESPUI.addControl( ControlType::Number, "Position 6 - Balls Deep", String(posBde), ControlColor::Alizarin, tab2, &numberCall_posBde );
  ESPUI.addControl( ControlType::Min, "Position 6 - Balls Deep", String(0), ControlColor::None, pos6LabelId );
  ESPUI.addControl( ControlType::Max, "Position 6 - Balls Deep", String(dist_max), ControlColor::None, pos6LabelId );

  // tab 3
  speedHoldInMinLabelId = ESPUI.addControl( ControlType::Number, "Speed Hold In Min", String(speedHoldInMin), ControlColor::Alizarin, tab3, &numberCall_speedHoldInMin );
  ESPUI.addControl( ControlType::Min, "Speed Hold In Min", String(vel_min), ControlColor::None, speedHoldInMinLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Hold In Min", String(vel_max), ControlColor::None, speedHoldInMinLabelId );

  speedHoldOutMinLabelId = ESPUI.addControl( ControlType::Number, "Speed Hold Out Min", String(speedHoldOutMin), ControlColor::Alizarin, tab3, &numberCall_speedHoldOutMin );
  ESPUI.addControl( ControlType::Min, "Speed Hold Out Min", String(vel_min), ControlColor::None, speedHoldOutMinLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Hold Out Min", String(vel_max), ControlColor::None, speedHoldOutMinLabelId );

  speedThrustInMinLabelId = ESPUI.addControl( ControlType::Number, "Speed Thrust In Min", String(speedThrustInMin), ControlColor::Alizarin, tab3, &numberCall_speedThrustInMin );
  ESPUI.addControl( ControlType::Min, "Speed Thrust In Min", String(vel_min), ControlColor::None, speedThrustInMinLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Thrust In Min", String(vel_max), ControlColor::None, speedThrustInMinLabelId );

  speedThrustOutMinLabelId = ESPUI.addControl( ControlType::Number, "Speed Thrust Out Min", String(speedThrustOutMin), ControlColor::Alizarin, tab3, &numberCall_speedThrustOutMin );
  ESPUI.addControl( ControlType::Min, "Speed Thrust Out Min", String(vel_min), ControlColor::None, speedThrustOutMinLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Thrust Out Min", String(vel_max), ControlColor::None, speedThrustOutMinLabelId );

  speedHoldInMaxLabelId = ESPUI.addControl( ControlType::Number, "Speed Hold In Max", String(speedHoldInMax), ControlColor::Alizarin, tab3, &numberCall_speedHoldInMax );
  ESPUI.addControl( ControlType::Min, "Speed Hold In Max", String(vel_min), ControlColor::None, speedHoldInMaxLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Hold In Max", String(vel_max), ControlColor::None, speedHoldInMaxLabelId );

  speedHoldOutMaxLabelId = ESPUI.addControl( ControlType::Number, "Speed Hold Out Max", String(speedHoldOutMax), ControlColor::Alizarin, tab3, &numberCall_speedHoldOutMax );
  ESPUI.addControl( ControlType::Min, "Speed Hold Out Max", String(vel_min), ControlColor::None, speedHoldOutMaxLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Hold Out Max", String(vel_max), ControlColor::None, speedHoldOutMaxLabelId );

  speedThrustInMaxLabelId = ESPUI.addControl( ControlType::Number, "Speed Thrust In Max", String(speedThrustInMax), ControlColor::Alizarin, tab3, &numberCall_speedThrustInMax );
  ESPUI.addControl( ControlType::Min, "Speed Thrust In Max", String(vel_min), ControlColor::None, speedThrustInMaxLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Thrust In Max", String(vel_max), ControlColor::None, speedThrustInMaxLabelId );

  speedThrustOutMaxLabelId = ESPUI.addControl( ControlType::Number, "Speed Thrust Out Max", String(speedThrustOutMax), ControlColor::Alizarin, tab3, &numberCall_speedThrustOutMax );
  ESPUI.addControl( ControlType::Min, "Speed Thrust Out Max", String(vel_min), ControlColor::None, speedThrustOutMaxLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Thrust Out Max", String(vel_max), ControlColor::None, speedThrustOutMaxLabelId );

  // tab 4

  // tab 5
  probabilityPauseLabelId = ESPUI.addControl( ControlType::Number, "Probability - Pause", String(prob_pause), ControlColor::Alizarin, tab5, &numberCall_probPause );
  ESPUI.addControl( ControlType::Min, "Probability - Pause", String(0), ControlColor::None, probabilityPauseLabelId );
  ESPUI.addControl( ControlType::Max, "Probability - Pause", String(100), ControlColor::None, probabilityPauseLabelId );

  probabilityHeavyLabelId = ESPUI.addControl( ControlType::Number, "Probability - Heavy Task", String(prob_heavytask), ControlColor::Alizarin, tab5, &numberCall_probHeavy );
  ESPUI.addControl( ControlType::Min, "Probability - Heavy Task", String(0), ControlColor::None, probabilityHeavyLabelId );
  ESPUI.addControl( ControlType::Max, "Probability - Heavy Task", String(100), ControlColor::None, probabilityHeavyLabelId );

  probabilityHoldLabelId = ESPUI.addControl( ControlType::Number, "Probability - Hold", String(prob_hold), ControlColor::Alizarin, tab5, &numberCall_probHold );
  ESPUI.addControl( ControlType::Min, "Probability - Hold", String(0), ControlColor::None, probabilityHoldLabelId );
  ESPUI.addControl( ControlType::Max, "Probability - Hold", String(100), ControlColor::None, probabilityHoldLabelId );
  


  ESPUI.begin("TaskHandler");
}

void loop(void) {
  if (flag_hold) {
    stepchain_busy = true;
    stepchain(activecommand.cmd_preptime, activecommand.cmd_amount, -activecommand.cmd_posin * stepspermm, -activecommand.cmd_posout * stepspermm, activecommand.cmd_accin, activecommand.cmd_accout, activecommand.cmd_velin * stepspermm, activecommand.cmd_velout * stepspermm, activecommand.cmd_pausein, activecommand.cmd_pauseout);
  }
  if (flag_thrust) {
    stepchain_busy = true;
    stepchain(activecommand.cmd_preptime, activecommand.cmd_amount, -activecommand.cmd_posin * stepspermm, -activecommand.cmd_posout * stepspermm, activecommand.cmd_accin, activecommand.cmd_accout, activecommand.cmd_velin * stepspermm, activecommand.cmd_velout * stepspermm, activecommand.cmd_pausein, activecommand.cmd_pauseout);
  }
  if (flag_paus) {
    stepchain_busy = true;
    stepchain(activecommand.cmd_preptime, activecommand.cmd_amount, -activecommand.cmd_posin * stepspermm, -activecommand.cmd_posout * stepspermm, activecommand.cmd_accin, activecommand.cmd_accout, activecommand.cmd_velin * stepspermm, activecommand.cmd_velout * stepspermm, activecommand.cmd_pausein, activecommand.cmd_pauseout);
  }
}
