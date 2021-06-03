

//#############################################################################
//
#define SECRET_SSID "YOUR_SSID "
#define SECRET_PASS "YOUR_PASS"

#define pinPul 16     //My values. Guess you need to adjust
#define pinDir 17
#define pinEnb 21
#define stepspermm 20
#define dist_max 150
#define dist_min 1
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

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

int statusLabelId;
int bpmLabelId;

int workpos = 0;
int new_workpos = 0;
int temp_targetpos = 0;
int velocity = 0;
bool flag_statuson = false;
bool flag_newworkposavailable = false;
bool flag_homing = false;
bool flag_disabletorque = true;
bool flag_manual_pos = false;
bool flag_manual_neg = false;

unsigned long starttime = 2;
unsigned long endtime = 1;
int bpm = 0;
String s_bpm;

void setup(void) {


  pinMode(pinEnb, OUTPUT);
  digitalWrite(pinEnb, LOW);

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

  ESPUI.setVerbosity(Verbosity::Quiet);
  // Serial.begin(115200);
  WiFi.begin(ssid, password);
  // Serial.print("\n\nTry to connect to existing network");

  statusLabelId = ESPUI.label("Status", ControlColor::Alizarin, "Stop");
  bpmLabelId = ESPUI.label("BPM", ControlColor::Alizarin, "None");
  ESPUI.switcher("OFF / ON", &switchonoff, ControlColor::Alizarin, false);
  ESPUI.switcher("Disable Torque", &switchdisable, ControlColor::Alizarin, true);
  ESPUI.slider("Speed", &sliderspeed, ControlColor::Alizarin, 0);
  ESPUI.slider("Distance", &sliderdistance, ControlColor::Alizarin, 0);
  ESPUI.button("Move Manual", &manual_pos, ControlColor::Alizarin, "Forward");
  ESPUI.button("Move Manual", &manual_neg, ControlColor::Alizarin, "Backward");
  ESPUI.button("Homing", &buttonhome, ControlColor::Alizarin, "Home");

  ESPUI.begin("OSSM");
}

void loop(void) {

  if (flag_statuson) {
    cmd_run();
  }

  if (flag_homing) {
    cmd_homing();
  }

  if (flag_manual_pos && !flag_statuson) {
    cmd_manual(0);
  }

  if (flag_manual_neg && !flag_statuson) {
    cmd_manual(1);
  }

}

void cmd_run() {
  if (stepper->getCurrentPosition() >= 0) {
    if (flag_newworkposavailable) {
      workpos = new_workpos;
      flag_newworkposavailable = false;
    }

    starttime = millis();
    if (starttime - endtime > 200) {
      bpm = 60000 / (starttime - endtime);
      s_bpm = String(bpm);
      //Serial.print(starttime);
      //Serial.print("int: "); Serial.println(bpm);
      //Serial.print ("string: "); Serial.println(s_bpm);
      ESPUI.print(bpmLabelId, s_bpm);
      endtime = starttime;
    }


    stepper->moveTo(workpos);
  }

  if (stepper->getCurrentPosition() <= workpos) {
    stepper->moveTo(0);
  }
}

void cmd_homing() {
  if (stepper->getCurrentPosition() != 0) {
    stepper->setSpeedInHz(20 * stepspermm);
    stepper->applySpeedAcceleration(); // Apply speed and acceleration
    stepper->moveTo(0);
  }
  else {
    stepper->setSpeedInHz(velocity * stepspermm);
    stepper->setAcceleration(velocity * stepspermm * 10);
    stepper->applySpeedAcceleration(); // Apply speed and acceleration
    flag_homing = false;
    ESPUI.print(statusLabelId, "Ready");
  }
  /*
    stepper->moveTo()(0);
    stepper->setSpeedInHz(20 * stepspermm);
    stepper->setAcceleration(20 * stepspermm * 6);
    // stepper.runToNewPosition(0);
    stepper->setSpeedInHz(velocity * stepspermm);
    stepper->setAcceleration(velocity * stepspermm * 10);
    flag_homing = false;
    ESPUI.print(statusLabelId, "Ready");
  */
}

void cmd_manual(int _dir) {

  if (_dir == 0) {
    stepper->moveTo(stepper->getCurrentPosition() - 10);

  }

  if (_dir == 1) {
    stepper->moveTo(stepper->getCurrentPosition() + 10);

  }
}

void sliderspeed(Control *sender, int type) {
  velocity = map(sender->value.toInt(), 0, 100, vel_min, vel_max);
  stepper->setSpeedInHz(velocity * stepspermm);
  stepper->setAcceleration(velocity * stepspermm * 10);
  stepper->applySpeedAcceleration(); // Apply speed and acceleration
  // Serial.print("Speed-Slider Value: "); // Serial.println(sender->value);
  // Serial.print("Mapped Speed Value: "); // Serial.println(velocity);
}

void sliderdistance(Control *sender, int type) {
  flag_newworkposavailable = true;
  new_workpos = -map(sender->value.toInt(), 0, 100, dist_min * stepspermm, dist_max * stepspermm);
  // Serial.print("Distance-Slider Value: "); // Serial.println(sender->value);
  // Serial.print("Mapped Distance Value: "); // Serial.println(-new_workpos);
}

void buttonhome(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      flag_statuson = false;
      // Serial.println("Start Homing");
      ESPUI.print(statusLabelId, "Start Homing");
      break;

    case B_UP:
      flag_homing = true;
      // Serial.println("Homing...");
      ESPUI.print(statusLabelId, "Homing...");
      break;
  }
}

void manual_pos(Control *sender, int type) {
  switch (type) {
    case B_DOWN:

      //      temp_targetpos = stepper.targetPosition(); // no equivalent in Fastaccelstepper I think. hope this works without this line
      // Serial.print("Store Target to ");// Serial.println(temp_targetpos);
      flag_manual_pos = true;
      // Serial.println("Manual Forward");
      ESPUI.print(statusLabelId, "Manual Forward");
      break;

    case B_UP:
      flag_manual_pos = false;
      //      stepper->moveTo()(temp_targetpos);
      // Serial.print("Re-Store Target to ");// Serial.println(temp_targetpos);
      // Serial.println("Stop");
      ESPUI.print(statusLabelId, "STOP");
      break;
  }
}

void manual_neg(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      if (stepper->getCurrentPosition() <= 0) {
        temp_targetpos = 0;
      }
      // temp_targetpos = stepper.targetPosition();
      // Serial.print("Store Target to ");// Serial.println(temp_targetpos);
      flag_manual_neg = true;
      // Serial.println("Manual Backward");
      ESPUI.print(statusLabelId, "Manual Backward");
      break;

    case B_UP:
      flag_manual_neg = false;
      //     stepper->moveTo(temp_targetpos);
      // Serial.print("Re-Store Target to ");// Serial.println(temp_targetpos);
      // Serial.println("Stop");
      ESPUI.print(statusLabelId, "STOP");
      break;
  }
}

void switchonoff(Control *sender, int value) {
  switch (value) {
    case S_ACTIVE:
      flag_statuson = true;
      ESPUI.print(statusLabelId, "Go");
      // Serial.println("ON");
      break;

    case S_INACTIVE:
      flag_statuson = false;
      ESPUI.print(statusLabelId, "Stop");
      // Serial.println("OFF");
      break;
  }
}

void switchdisable(Control *sender, int value) {
  switch (value) {
    case S_ACTIVE:
      flag_statuson = false;
      flag_disabletorque = true;
      digitalWrite(pinEnb, HIGH);
      ESPUI.print(statusLabelId, "Torque disabeled");
      // Serial.println("Outputs disabled");
      break;

    case S_INACTIVE:
      flag_disabletorque = false;
      digitalWrite(pinEnb, LOW);
      stepper->setCurrentPosition(0);
      ESPUI.print(statusLabelId, "Torque enabeled");
      // Serial.println("Outputs enabeled");
      break;
  }
}
