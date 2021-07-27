void thrust(int _amount=0, int _posIn=0, int _posOut=0, int _accIn=500, int _velIn=400, int _accOut=500, int _velOut=400); //Dummy
void hold(int _posIn=0, int _posOut=0, int _accIn=0, int _velIn=0, int _accOut=0, int _velOut=0, int _pauseIn=0); //Dummy
void paus(int _time=1000); //Dummy
void stepchain(int _preptime = 0, int _amount=1, int _posIn=0, int _posOut=0, int _accIn=0, int _accOut=0, int _velIn=0,  int _velOut=0,  int _pauseIn=0, int _pauseOut=0); //Dummy

// How to PrePare the cmdfile
// TaskIdentifier | preptime | amount | posin | posout | accin | accout | velin | velout | pausein | pauseout | Message | Tipp
// 


//#############################################################################
#define SECRET_SSID "Internet"
#define SECRET_PASS "F-jAm-Dzj-0QJREl}cUE|31W(pN\"L3:\"3{\"ckwyW{b>u,x4gI[SL;.cRle+HK+%"

#define pinPul 16
#define pinDir 17
#define pinEnb 21
#define stepspermm 20
#define dist_max 150
#define dist_min 10
#define vel_min 1
#define vel_max 600

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

#include <SPIFFS.h>

//#############################################################################

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
bool reverseStepper = false;

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

int ProgramLabelId;
int NextTaskLabelId;
int CurrentTaskLabelId;
int PrevTaskLabelId;
int TimerLabelId;

int posGap = 0;
int posLip = 20;
int posTip = 50;
int posEnt = 100;
int posThr = 120;
int posBde = 140;

int velocity;
int acceleration;

int tasknumber = 1;

bool flag_disabletorque = true;
bool flag_homing = false;
bool flag_statuson = false;

bool flag_hold = false;
bool flag_thrust = false;
bool flag_paus = false;

int stp = 0;
int secondscount = 0;
bool stepchain_busy = false;

String msg;

  int amountdone = 1;
  int targetpos;
  int waittime;
  bool waitactive = false;
  unsigned long timestamp1;
  unsigned long timestamp2;
  

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

struct command activecommand;

//#############################################################################
//ESP Callbacks

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

void button_nextTask(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      // Serial.println("Next Task");
      nextTask();
      break;

    case B_UP:
      // Serial.println("Switched to Next Task");
      break;
  }
}

void button_prevTask(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      // Serial.println("Next Task");
      
      break;

    case B_UP:
      // Serial.println("Switched to Next Task");
      break;
  }
}

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

struct command randomizeTask() {
  struct command meincmd;
  String message;
  String hint;

  String str_posin;
  String str_posout;

  int taskidentifier_dice = random(1,4);
  int posin_dice = random(2,7);
  int posout_dice = random(1,6);
  if (posin_dice <= posout_dice){
    posout_dice = posin_dice - 1;
  }
  
  switch (taskidentifier_dice){
    case 1:
    meincmd.taskidentifier = 'P';
    meincmd.cmd_preptime = 0;
    meincmd.cmd_amount = 1;
    meincmd.cmd_posin = 0;
    meincmd.cmd_posout = 0;
    meincmd.cmd_accin = 0;
    meincmd.cmd_accout = 0;
    meincmd.cmd_velin = 0;
    meincmd.cmd_velout = 0;
    meincmd.cmd_pausein = 0;
    meincmd.cmd_pauseout = random(5,11);
    meincmd.cmd_message = "Pause for " + String(meincmd.cmd_pauseout);
    meincmd.cmd_hint = "Catch your breath";
    break;

    case 2:
    meincmd.taskidentifier = 'H';
    if (posin_dice >= 5){
      meincmd.cmd_preptime = 5; //random(3,7);
    }
    else{
      meincmd.cmd_preptime = 0; //random(3,7);
    }
    
    meincmd.cmd_amount = 1;
    switch (posin_dice){ //cmd_posin
      case 1:
      meincmd.cmd_posin = posGap;
      message = "Beg for Cock for ";
      break;

      case 2:
      meincmd.cmd_posin = posLip;
      message = "Lick the Tip for ";
      break;

      case 3:
      meincmd.cmd_posin = posTip;
      message = "Hold the Tip in your Mouth for ";
      break;

      case 4:
      meincmd.cmd_posin = posEnt;
      message = "Hold it at the Entrance of your Throat for ";
      break;

      case 5:
      meincmd.cmd_posin = posThr;
      message = "Hold it past the Entrance of your Throat for ";
      break;
      
      case 6:
      meincmd.cmd_posin = posBde;
      message = "Hold it ballsdeep for ";
      break;
    }
    meincmd.cmd_posout = 0;
    meincmd.cmd_accin = 5000;
    meincmd.cmd_accout = 5000;
    meincmd.cmd_velin = 200;
    meincmd.cmd_velout = 400;
    meincmd.cmd_pausein = random(5,16);
    meincmd.cmd_pauseout = 0;
    meincmd.cmd_message = message + String(meincmd.cmd_pausein) + " Seconds" ;
    meincmd.cmd_hint = "No Air";
    break;

    case 3:
    meincmd.taskidentifier = 'T';
    if (posin_dice >= 5){
      meincmd.cmd_preptime = 5; //random(3,7);
    }
    else{
      meincmd.cmd_preptime = 0; //random(3,7);
    }
    meincmd.cmd_amount = random(5,16);
    
    switch (posin_dice){ //cmd_posin
      case 1:
      meincmd.cmd_posin = posGap;  
      str_posin = "";  
      break;

      case 2:
      meincmd.cmd_posin = posLip;
      str_posin = "to your Lips";
      break;

      case 3:
      meincmd.cmd_posin = posTip;
      str_posin = "to the Tip";
      break;

      case 4:
      meincmd.cmd_posin = posEnt;
      str_posin = "to the Entrace of your Throat";
      break;

      case 5:
      meincmd.cmd_posin = posThr;
      str_posin = "to the Back of your Throat";
      break;
      
      case 6:
      meincmd.cmd_posin = posBde;
      str_posin = "Ballsdeep in your Throat";
      break;
    }

    switch (posout_dice){ //cmd_posout
      case 1:
      meincmd.cmd_posout = posGap;
      str_posout = "outside your Mouth ";
      break;

      case 2:
      meincmd.cmd_posout = posLip;
      str_posout = "your Lips ";
      break;

      case 3:
      meincmd.cmd_posout = posTip;
      str_posout = "the Tip ";
      break;

      case 4:
      meincmd.cmd_posout = posEnt;
      str_posout = "the Entrace of your Throat ";
      break;

      case 5:
      meincmd.cmd_posout = posThr;
      str_posout = "the Back of your Throat ";
      break;
      
      case 6:
      meincmd.cmd_posout = posBde;
      str_posout = "";
      break;
    }

    meincmd.cmd_accin = 10000;
    meincmd.cmd_accout = 10000;
    meincmd.cmd_velin = 350;
    meincmd.cmd_velout = 350;
    meincmd.cmd_pausein = 0;
    meincmd.cmd_pauseout = 0;
    meincmd.cmd_message = "Suck it from " + str_posout + str_posin + " for " + String(meincmd.cmd_amount) + " Times" ;
    meincmd.cmd_hint = "In - Out";
    break;
  }
  
  if (reverseStepper){
    meincmd.cmd_posin = -meincmd.cmd_posin;
    meincmd.cmd_posout = -meincmd.cmd_posin;
  }

  return meincmd;
}

void printTask(){ //Just for Debugging
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

void taskHandler(){
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
  //flag_nextTask = true;
}

void start(){
  digitalWrite(pinEnb, LOW);
  flag_statuson = true;
  tasknumber = 1;
  ESPUI.print(ProgramLabelId, String(tasknumber));
  activecommand = randomizeTask();
  printTask();
  taskHandler();
}

void stopp(){
  flag_statuson = false;
  flag_hold = false;
  flag_thrust = false;
  flag_paus = false;
  digitalWrite(pinEnb, HIGH);
  stepper->setCurrentPosition(0);
  stepchain_busy = false;
  stp = 0;
  ESPUI.print(CurrentTaskLabelId,"- - -");
}

void nextTask(){
  tasknumber++;
  ESPUI.print(ProgramLabelId, String(tasknumber));
  activecommand = randomizeTask();
  printTask();
  taskHandler();
}

void stepchain(int _preptime, int _amount, int _posIn, int _posOut, int _accIn, int _accOut, int _velIn,  int _velOut,  int _pauseIn, int _pauseOut){
  
  if (stepchain_busy && flag_statuson) {
    switch (stp) {
      
      case 0: // Preparetime    
        if (waitactive){
        timestamp2 = millis();
          if((timestamp2 - timestamp1)>= 1000 ){ 
            timestamp1 = millis();   
            secondscount++;
            if (_preptime > 0){ESPUI.print(TimerLabelId, "Get Ready: " + String(_preptime - secondscount) + " Seconds");}
            if(secondscount >= _preptime){
              waitactive = false;
              if (_amount > 1){ESPUI.print(TimerLabelId,String(_amount));} 
              stp = 1;
            }
          }
        } 
        else{
          secondscount = 0;   
          timestamp1 = millis();  
          if (_preptime > 0){
            ESPUI.print(TimerLabelId,"Get Ready: " + String(_preptime) + " Seconds");
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
        if (waitactive){
        timestamp2 = millis();
          if((timestamp2 - timestamp1)>= 1000 ){ 
            timestamp1 = millis();   
            secondscount++;
            if (_pauseIn > 0){ESPUI.print(TimerLabelId, String(_pauseIn - secondscount) + " Seconds");}
            if(secondscount >= _pauseIn){
              waitactive = false;
              stp = 5;
            }
          }
        } 
        else{
          secondscount = 0;   
          timestamp1 = millis();  
          if (_pauseIn > 0){
            ESPUI.print(TimerLabelId,String(_pauseIn) + " Seconds");
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
        if (waitactive){
        timestamp2 = millis();
          if((timestamp2 - timestamp1)>= 1000 ){ 
            timestamp1 = millis();   
            secondscount++;
            if (_pauseOut > 0){ESPUI.print(TimerLabelId, String(_pauseOut - secondscount) + " Seconds");}
            if(secondscount >= _pauseOut){
              waitactive = false;
              stp = 9;
            }
          }
        } 
        else{
          secondscount = 0;   
          timestamp1 = millis();  
          if (_pauseOut > 0){
            ESPUI.print(TimerLabelId,String(_pauseOut) + " Seconds");
            waitactive = true;
            }  
            else {
              stp = 9;
            }
        }    
        break;
        
      case 9: // Rest flags
        amountdone++;
        if (amountdone >= _amount){
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
        else{  
          Serial.print("Stepchain: "); Serial.println(stp); 
          Serial.print(_amount - amountdone); Serial.println(" to go. --> Stepchain returning");  
          ESPUI.print(TimerLabelId,String(_amount - amountdone)); 
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
  //ESPUI stuff
  ESPUI.setVerbosity(Verbosity::Quiet);

  ESPUI.button("Start", &button_start, ControlColor::Sunflower, "Go");
  ESPUI.button("Stop", &button_stop, ControlColor::None, "Stop");
  ProgramLabelId = ESPUI.label("Program", ControlColor::Emerald, "-");

  CurrentTaskLabelId = ESPUI.label("Current Task", ControlColor::Emerald, "-");
  TimerLabelId = ESPUI.label("Timer", ControlColor::Emerald, "-");

  ESPUI.number("Position 1 - Not Touching", &numberCall_posGap, ControlColor::Alizarin, 0, 0, dist_max); // Gap
  ESPUI.number("Position 2 - On Lips", &numberCall_posLip, ControlColor::Alizarin, 20, 0, dist_max); // Lip
  ESPUI.number("Position 3 - Just the Tip", &numberCall_posTip, ControlColor::Alizarin, 50, 0, dist_max); // Tip
  ESPUI.number("Position 4 - Throat Entrance", &numberCall_posEnt, ControlColor::Alizarin, 100, 0, dist_max); // Entrance
  ESPUI.number("Position 5 - Past Entrance", &numberCall_posThr, ControlColor::Alizarin, 120, 0, dist_max); // Down
  ESPUI.number("Position 6 - Balls Deep", &numberCall_posBde, ControlColor::Alizarin, 140, 0, dist_max); // Total
  
  //ESPUI.slider("Position", &sliderpos, ControlColor::Alizarin, 0);

  ESPUI.begin("TaskHandler");
}

void loop(void) {
  if (flag_hold){
    stepchain_busy = true;
    stepchain(activecommand.cmd_preptime, activecommand.cmd_amount, -activecommand.cmd_posin * stepspermm, -activecommand.cmd_posout * stepspermm, activecommand.cmd_accin, activecommand.cmd_accout, activecommand.cmd_velin * stepspermm, activecommand.cmd_velout * stepspermm, activecommand.cmd_pausein, activecommand.cmd_pauseout);
  }
  if (flag_thrust){
    stepchain_busy = true;
    stepchain(activecommand.cmd_preptime, activecommand.cmd_amount, -activecommand.cmd_posin * stepspermm, -activecommand.cmd_posout * stepspermm, activecommand.cmd_accin, activecommand.cmd_accout, activecommand.cmd_velin * stepspermm, activecommand.cmd_velout * stepspermm, activecommand.cmd_pausein, activecommand.cmd_pauseout);
  }
  if (flag_paus){
    stepchain_busy = true;
    stepchain(activecommand.cmd_preptime, activecommand.cmd_amount, -activecommand.cmd_posin * stepspermm, -activecommand.cmd_posout * stepspermm, activecommand.cmd_accin, activecommand.cmd_accout, activecommand.cmd_velin * stepspermm, activecommand.cmd_velout * stepspermm, activecommand.cmd_pausein, activecommand.cmd_pauseout);
  }    
}
