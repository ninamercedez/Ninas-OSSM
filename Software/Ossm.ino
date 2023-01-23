// Header for DefaultValues for Stepchain
void stepchain(int _preptime = 0, int _amount = 1, int _posIn = 0, int _posOut = 0, int _accIn = 0, int _accOut = 0, int _velIn = 0,  int _velOut = 0,  int _pauseIn = 0, int _pauseOut = 0);

//#############################################################################

int pinPul = 16;
int pinDir = 17;
int pinEnb = 21;

int stepspermm = 20;
int dist_max = 150;
int vel_min = 1;
int vel_max = 600;
int acc_min = 1000;
int acc_max = 10000;
bool reverseStepper = false;

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
#include <EEPROM.h>
#include <DNSServer.h>

uint16_t DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer;
//#############################################################################

String ssid;
String password;

// Global Variables - Stepper
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;



bool stepchainverbose = false;
int velocity;
int acceleration;

bool flag_dec = false;


// Global Variables - GUI
uint16_t BPMLabelId;
uint16_t StatusLabelId;
float bpm;
unsigned long timestamp_stpchain_start;
unsigned long timestamp_stpchain_stop;
unsigned long airwatchtime1 = 0;
unsigned long airwatchtime2 = 0;
unsigned long airwatchtime3 = 0;
unsigned long airwatchtime4 = 0;
unsigned long millispased;
int ProgramLabelId;
int CurrentTaskLabelId;
int TimerLabelId;
int PrevTaskLabelId;
int tasknumber = 1;

uint16_t selectStartPage;
int selectedOpt = 0;
uint16_t textinput_ssid;
uint16_t textinput_pw;

uint16_t tab1;
uint16_t tab2;
uint16_t tab3;
uint16_t tab4;
uint16_t tab5;
uint16_t tab6;

uint16_t textInput1;
uint16_t textInput2;
uint16_t eepromwritebutton;
uint16_t eepromerasebutton;

uint16_t posInSliderLabelId;
uint16_t posOutSliderLabelId;

uint16_t strokeLengthLabelId;
uint16_t strokeOffsetLabelId;

uint16_t speedThrustInSliderLabelId;
uint16_t speedThrustOutSliderLabelId;
uint16_t syncSpeedLabelId;

uint16_t accThrustInLabelId;
uint16_t accThrustOutLabelId;

uint16_t timesPauseInLabelId;
uint16_t timesPauseOutLabelId;

uint16_t posLabelId;

uint16_t speedHoldInLabelId;
uint16_t speedHoldOutLabelId;
uint16_t speedThrustInLabelId;
uint16_t speedThrustOutLabelId;

uint16_t timesPreptimeLabelId;
uint16_t timesPauseMinLabelId;
uint16_t timesHoldMinLabelId;
uint16_t timesThrustMinLabelId;
uint16_t timesPauseMaxLabelId;
uint16_t timesHoldMaxLabelId;
uint16_t timesThrustMaxLabelId;

uint16_t probabilityPauseLabelId;
uint16_t probabilityMfLabelId;
uint16_t probabilityTfLabelId;
uint16_t probabilitySfLabelId;
uint16_t probabilityHoldLabelId;
uint16_t probabilityGainLabelId;

uint16_t posGapLabelId;
uint16_t posLipLabelId;
uint16_t posTipLabelId;
uint16_t posEntLabelId;
uint16_t posThrLabelId;
uint16_t posBdeLabelId;

uint16_t timeairwatchtriggerLabelId;
uint16_t timeairwatchresetLabelId;

// Global Variables - Positions
int posIn = 50;
int posOut = 0;

int strokeLength = 50;
int strokeOffset = 0;

int posGap = 0;
int posLip = 20;
int posTip = 60;
int posEnt = 100;
int posThr = 120;
int posBde = 150;

// Global Variables - Speed
int speedThrustIn = 150;
int speedThrustOut = 150;

int speedHoldInMin = 60;
int speedHoldOutMin = 300;
int speedThrustInMin = 100;
int speedThrustOutMin = 100;
int speedHoldInMax = 120;
int speedHoldOutMax = 450;
int speedThrustInMax = 400;
int speedThrustOutMax = 400;

// Global Variables - Acc
int accThrustIn = 10000;
int accThrustOut = 10000;

// Global Variables - Times
int timesPauseIn = 0;
int timesPauseOut = 0;

int timesPreptime = 5;
int timesPauseMin = 5;
int timesHoldMin = 5;
int timesThrustMin = 10;
int timesPauseMax = 15;
int timesHoldMax = 10;
int timesThrustMax = 20;

// Global Variables - Probability
int prob_pause = 0;
int prob_task; // =100 - prob_pause = 70
int prob_mf = 100;
int prob_tf = 0;
int prob_sf = 0;
int prob_hold = 0;
int gain = 1;

// Global Variables - Airwatch
int airwatch_resettime = 3000;
int airwatch_triggertime = 20000;
bool enable_airwatch = true;
bool flag_airwatch = false;

// Global Variables - Flags
bool flag_statuson = false;
bool flag_torque = false;
bool flag_syncspeed = false;
bool flag_speedwithsliders = false;
bool flag_positionwithsliders = false;
bool flag_gain = false;

bool flag_hold = false;
bool flag_tf = false;
bool flag_paus = false;
bool flag_mf = false;
bool flag_sf = false;
bool lastTaskPause = false;
bool flag_task = false;



// Global Variables - Stepchain
bool stepchain_busy = false;
bool waitactive = false;
int stp = 0;
int state=0;
int secondscount = 0;
int amountdone = 1;
int targetpos;
int offsetpos = 0;
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

void connecttowifi() {
  //Begin the Wifi
  ssid = EEPROM.readString(0);
  password = EEPROM.readString(128);

  WiFi.begin(ssid.c_str(), password.c_str());
  delay(2000);
  Serial.println(WiFi.localIP());
  {
    uint8_t timeout = 10;

    // Wait for connection, 5s timeout
    do {
      delay(500);
      Serial.print(".");
      timeout--;
    } while (timeout && WiFi.status() != WL_CONNECTED);

    // not connected -> create hotspot
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("\n\nCreating hotspot");
      const char* apssid = "OSSM";
      const char* appassword = "nowiamossm";
      WiFi.mode(WIFI_AP);
      delay(100);

      WiFi.softAP(apssid, appassword);

      timeout = 5;

      do
      {
        delay(500);
        Serial.print(".");
        timeout--;
      } while (timeout);
    }
  }

  dnsServer.start(DNS_PORT, "*", apIP);

  Serial.println("\n\nWiFi parameters:");
  Serial.print("Mode: ");
  Serial.println(WiFi.getMode() == WIFI_AP ? "Station" : "Client");
  Serial.print("IP address: ");
  Serial.println(WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP());
}


//#############################################################################
//Initial GUI
void loadStartPage() {
  //tab7 = ESPUI.addControl( ControlType::Tab, "Options", "Options" );
  //tab7 = ESPUI.addControl( ControlType::Tab, "EEPROM", "Configuration" );

  selectStartPage = ESPUI.addControl(ControlType::Select, "Select:", "", ControlColor::Alizarin, Control::noParent, &select_loadStartPage);
  ESPUI.setPanelWide(selectStartPage, true);
  ESPUI.addControl(ControlType::Option, "Select Option ...", "Default", ControlColor::Alizarin, selectStartPage);
  ESPUI.addControl(ControlType::Option, "OSSM - Normal Mode", "Opt1", ControlColor::Alizarin, selectStartPage);
  ESPUI.addControl(ControlType::Option, "OSSM - Emma Mode", "Opt2", ControlColor::Alizarin, selectStartPage);
  ESPUI.addControl(ControlType::Option, "NTT", "Opt3", ControlColor::Alizarin, selectStartPage);
  ESPUI.separator(" ");
  textInput1 = ESPUI.addControl(ControlType::Text, "SSID", "...", ControlColor::Alizarin, Control::noParent, &callback_input_ssid);
  textInput2 = ESPUI.addControl(ControlType::Text, "PW", "...", ControlColor::Alizarin, Control::noParent, &callback_input_pw);
  eepromwritebutton = ESPUI.addControl(ControlType::Button, "Write to EEPROM", "Write", ControlColor::Wetasphalt, Control::noParent, &buttonWriteEEPROM);
  eepromerasebutton = ESPUI.addControl(ControlType::Button, "Erase EEPROM", "Erase", ControlColor::Wetasphalt, Control::noParent, &buttonEraseEEPROM);
  ESPUI.setPanelWide(textInput1, true);
  ESPUI.setPanelWide(textInput2, true);
  ESPUI.setPanelWide(eepromwritebutton, true);
  ESPUI.setPanelWide(eepromerasebutton, true);
}

void loadOpt1() {
  Serial.println("Loading Option 1");
  selectedOpt = 1;

  ESPUI.removeControl(textInput1, false);
  ESPUI.removeControl(textInput2, false);
  ESPUI.removeControl(eepromwritebutton, false);
  ESPUI.removeControl(eepromerasebutton, true);

  tab1 = ESPUI.addControl( ControlType::Tab, "Status", "Status" );
  tab2 = ESPUI.addControl( ControlType::Tab, "Settings", "Position" );
  tab3 = ESPUI.addControl( ControlType::Tab, "Settings", "Speed" );
  tab4 = ESPUI.addControl( ControlType::Tab, "Settings", "Acceleration" );
  tab5 = ESPUI.addControl( ControlType::Tab, "Settings", "Times" );
  tab6 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings" );

  // shown above all tabs
  uint16_t ReseButtonLabelId = ESPUI.button("Reset", &button_reset, ControlColor::Dark, "Reset");
  ESPUI.setPanelWide(ReseButtonLabelId, true);
  ESPUI.separator("");

  BPMLabelId = ESPUI.label("BPM:", ControlColor::Sunflower, "-");
  ESPUI.button("Start", &button_start, ControlColor::Sunflower, "Start");
  ESPUI.button("Stop", &button_stop, ControlColor::Sunflower, "Stop");

  // tab 1
  StatusLabelId = ESPUI.addControl( ControlType::Label, "Status", "-", ControlColor::Alizarin, tab1 );
  ESPUI.addControl( ControlType::Switcher, "Torque", "", ControlColor::Alizarin, tab1, &switch_torque );

  // tab 2
  posInSliderLabelId = ESPUI.addControl( ControlType::Slider, "Position In", String(posIn), ControlColor::Alizarin, tab2, &sliderCall_posIn );
  ESPUI.addControl( ControlType::Min, "Position In", String(0), ControlColor::None, posInSliderLabelId );
  ESPUI.addControl( ControlType::Max, "Position In", String(dist_max), ControlColor::None, posInSliderLabelId );

  posOutSliderLabelId = ESPUI.addControl( ControlType::Slider, "Position Out", String(posOut), ControlColor::Alizarin, tab2, &sliderCall_posOut );
  ESPUI.addControl( ControlType::Min, "Position Out", String(0), ControlColor::None, posOutSliderLabelId );
  ESPUI.addControl( ControlType::Max, "Position Out", String(dist_max), ControlColor::None, posOutSliderLabelId );

  // tab 3
  speedThrustInSliderLabelId = ESPUI.addControl( ControlType::Slider, "Speed Thrust In", String(speedThrustIn), ControlColor::Alizarin, tab3, &sliderCall_speedThrustIn );
  ESPUI.addControl( ControlType::Min, "Speed Thrust In", String(vel_min), ControlColor::None, speedThrustInSliderLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Thrust In", String(vel_max), ControlColor::None, speedThrustInSliderLabelId );

  speedThrustOutSliderLabelId = ESPUI.addControl( ControlType::Slider, "Speed Thrust Out", String(speedThrustOut), ControlColor::Alizarin, tab3, &sliderCall_speedThrustOut );
  ESPUI.addControl( ControlType::Min, "Speed Thrust Out", String(vel_min), ControlColor::None, speedThrustOutSliderLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Thrust Out", String(vel_max), ControlColor::None, speedThrustOutSliderLabelId );

  syncSpeedLabelId = ESPUI.addControl( ControlType::Switcher, "Sync Speed", "", ControlColor::Alizarin, tab3, &switch_syncspeed );

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

  // tab 6
  //ESPUI.addControl( ControlType::Switcher, "Sliders for Speed", "", ControlColor::Alizarin, tab6, &switch_slidersspeed );
  //ESPUI.addControl( ControlType::Switcher, "Sliders for Position", "", ControlColor::Alizarin, tab6, &switch_slidersposition );

  ESPUI.jsonReload();
}

void loadOpt2() {
  Serial.println("Loading Option 2");
  selectedOpt = 2;

  ESPUI.removeControl(textInput1, false);
  ESPUI.removeControl(textInput2, false);
  ESPUI.removeControl(eepromwritebutton, false);
  ESPUI.removeControl(eepromerasebutton, true);

  tab1 = ESPUI.addControl( ControlType::Tab, "Status", "Status" );
  tab2 = ESPUI.addControl( ControlType::Tab, "Settings", "Position" );
  tab3 = ESPUI.addControl( ControlType::Tab, "Settings", "Speed" );
  tab4 = ESPUI.addControl( ControlType::Tab, "Settings", "Acceleration" );
  tab5 = ESPUI.addControl( ControlType::Tab, "Settings", "Times" );
  tab6 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings" );

  // shown above all tabs
  uint16_t ReseButtonLabelId = ESPUI.button("Reset", &button_reset, ControlColor::Dark, "Reset");
  ESPUI.setPanelWide(ReseButtonLabelId, true);
  ESPUI.separator("");

  BPMLabelId = ESPUI.label("BPM:", ControlColor::Sunflower, "-");
  ESPUI.button("Start", &button_start, ControlColor::Sunflower, "Start");
  ESPUI.button("Stop", &button_stop, ControlColor::Sunflower, "Stop");

  // tab 1
  StatusLabelId = ESPUI.addControl( ControlType::Label, "Status", "-", ControlColor::Alizarin, tab1 );
  ESPUI.addControl( ControlType::Switcher, "Torque", "", ControlColor::Alizarin, tab1, &switch_torque );

  // tab 2
  strokeLengthLabelId = ESPUI.addControl( ControlType::Slider, "Stroke Length", String(strokeLength), ControlColor::Alizarin, tab2, &sliderCall_strokeLength );
  ESPUI.addControl( ControlType::Min, "", String(0), ControlColor::None, strokeLengthLabelId );
  ESPUI.addControl( ControlType::Max, "", String(dist_max), ControlColor::None, strokeLengthLabelId );

  strokeOffsetLabelId = ESPUI.addControl( ControlType::Slider, "Stroke Offset", String(strokeOffset), ControlColor::Alizarin, tab2, &sliderCall_strokeOffset );
  ESPUI.addControl( ControlType::Min, "", String(0), ControlColor::None, strokeOffsetLabelId );
  ESPUI.addControl( ControlType::Max, "", String(dist_max), ControlColor::None, strokeOffsetLabelId );

  // tab 3
  speedThrustInSliderLabelId = ESPUI.addControl( ControlType::Slider, "Speed Thrust In", String(speedThrustIn), ControlColor::Alizarin, tab3, &sliderCall_speedThrustIn );
  ESPUI.addControl( ControlType::Min, "Speed Thrust In", String(vel_min), ControlColor::None, speedThrustInSliderLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Thrust In", String(vel_max), ControlColor::None, speedThrustInSliderLabelId );

  speedThrustOutSliderLabelId = ESPUI.addControl( ControlType::Slider, "Speed Thrust Out", String(speedThrustOut), ControlColor::Alizarin, tab3, &sliderCall_speedThrustOut );
  ESPUI.addControl( ControlType::Min, "Speed Thrust Out", String(vel_min), ControlColor::None, speedThrustOutSliderLabelId );
  ESPUI.addControl( ControlType::Max, "Speed Thrust Out", String(vel_max), ControlColor::None, speedThrustOutSliderLabelId );

  syncSpeedLabelId = ESPUI.addControl( ControlType::Switcher, "Sync Speed", "", ControlColor::Alizarin, tab3, &switch_syncspeed );

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

  // tab 6
  //ESPUI.addControl( ControlType::Switcher, "Sliders for Speed", "", ControlColor::Alizarin, tab6, &switch_slidersspeed );
  //ESPUI.addControl( ControlType::Switcher, "Sliders for Position", "", ControlColor::Alizarin, tab6, &switch_slidersposition );

  ESPUI.jsonReload();
}

void loadOpt3() {
  Serial.println("Loading Option 3");
  selectedOpt = 3;

  ESPUI.removeControl(textInput1, false);
  ESPUI.removeControl(textInput2, false);
  ESPUI.removeControl(eepromwritebutton, false);
  ESPUI.removeControl(eepromerasebutton, true);

  tab1 = ESPUI.addControl( ControlType::Tab, "Main", "Main" );
  tab2 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Position" );
  tab3 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Speed" );
  tab4 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Times" );
  tab5 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Probability" );
  tab6 = ESPUI.addControl( ControlType::Tab, "Settings", "Settings - Airwatch" );

  // shown above all tabs
  uint16_t ReseButtonLabelId = ESPUI.button("Reset", &button_reset, ControlColor::Dark, "Reset");
  ESPUI.setPanelWide(ReseButtonLabelId, true);
  ESPUI.separator("");

  // shown above all tabs
  ProgramLabelId = ESPUI.addControl( ControlType::Label, "Tasknumber", "-", ControlColor::Emerald );
  CurrentTaskLabelId = ESPUI.addControl( ControlType::Label, "Task", "-", ControlColor::Emerald );
  TimerLabelId = ESPUI.addControl( ControlType::Label, "Timer", "-", ControlColor::Emerald );
  ESPUI.setPanelWide(ProgramLabelId, true);
  ESPUI.setPanelWide(CurrentTaskLabelId, true);
  ESPUI.setPanelWide(TimerLabelId, true);

  // tab 1
  ESPUI.addControl( ControlType::Button, "Start", "Start", ControlColor::Sunflower, tab1, &button_start );
  ESPUI.addControl( ControlType::Button, "Stop", "Stop", ControlColor::Sunflower, tab1, &button_stop );
  PrevTaskLabelId = ESPUI.addControl( ControlType::Label, "Prevoius Tasks", "- - - - -", ControlColor::Sunflower, tab1 );

  // tab 2
  posGapLabelId = ESPUI.addControl( ControlType::Slider, "Positions", String(posGap), ControlColor::Alizarin, tab2, &numberCall_posGap );

  posLipLabelId = ESPUI.addControl( ControlType::Slider, "Position 2 - On Lips", String(posLip), ControlColor::Alizarin, posGapLabelId, &numberCall_posLip );
  posTipLabelId = ESPUI.addControl( ControlType::Slider, "Position 3 - Just the Tip", String(posTip), ControlColor::Alizarin, posGapLabelId, &numberCall_posTip );
  posEntLabelId = ESPUI.addControl( ControlType::Slider, "Position 4 - Throat Entrance", String(posEnt), ControlColor::Alizarin, posGapLabelId, &numberCall_posEnt );
  posThrLabelId = ESPUI.addControl( ControlType::Slider, "Position 5 - Past Entrance", String(posThr), ControlColor::Alizarin, posGapLabelId, &numberCall_posThr );
  posBdeLabelId = ESPUI.addControl( ControlType::Slider, "Position 6 - Balls Deep", String(posBde), ControlColor::Alizarin, posGapLabelId, &numberCall_posBde );

  ESPUI.addControl( ControlType::Min, "", String(0), ControlColor::None, posGapLabelId );
  ESPUI.addControl( ControlType::Max, "", String(dist_max), ControlColor::None, posGapLabelId );
  ESPUI.addControl( ControlType::Max, "", String(dist_max), ControlColor::None, posLipLabelId );
  ESPUI.addControl( ControlType::Max, "", String(dist_max), ControlColor::None, posTipLabelId );
  ESPUI.addControl( ControlType::Max, "", String(dist_max), ControlColor::None, posEntLabelId );
  ESPUI.addControl( ControlType::Max, "", String(dist_max), ControlColor::None, posThrLabelId );
  ESPUI.addControl( ControlType::Max, "", String(dist_max), ControlColor::None, posBdeLabelId );

  ESPUI.setPanelWide(posGapLabelId, true);

  // tab 3
  ESPUI.addControl(ControlType::Separator, "Hold", "", ControlColor::None, tab3);
  speedHoldInLabelId = ESPUI.addControl( ControlType::Slider, "In", String(speedHoldInMin), ControlColor::Alizarin, tab3, &sliderCall_speedHoldInMin );
  ESPUI.addControl( ControlType::Slider, "", String(speedHoldInMax), ControlColor::Alizarin, speedHoldInLabelId, &sliderCall_speedHoldInMax );
  ESPUI.addControl( ControlType::Min, "", String(vel_min), ControlColor::None, speedHoldInLabelId );
  ESPUI.addControl( ControlType::Max, "", String(vel_max), ControlColor::None, speedHoldInLabelId );

  speedHoldOutLabelId = ESPUI.addControl( ControlType::Slider, "Out", String(speedHoldOutMin), ControlColor::Alizarin, tab3, &sliderCall_speedHoldOutMin );
  ESPUI.addControl( ControlType::Slider, "", String(speedHoldOutMax), ControlColor::Alizarin, speedHoldOutLabelId, &sliderCall_speedHoldOutMax );
  ESPUI.addControl( ControlType::Min, "", String(vel_min), ControlColor::None, speedHoldOutLabelId );
  ESPUI.addControl( ControlType::Max, "", String(vel_max), ControlColor::None, speedHoldOutLabelId );


  ESPUI.addControl(ControlType::Separator, "Thrust", "", ControlColor::None, tab3);
  speedThrustInLabelId = ESPUI.addControl( ControlType::Slider, "In", String(speedThrustInMin), ControlColor::Alizarin, tab3, &sliderCall_speedThrustInMin );
  ESPUI.addControl( ControlType::Slider, "", String(speedThrustInMax), ControlColor::Alizarin, speedThrustInLabelId, &sliderCall_speedThrustInMax );
  ESPUI.addControl( ControlType::Min, "", String(vel_min), ControlColor::None, speedThrustInLabelId );
  ESPUI.addControl( ControlType::Max, "", String(vel_max), ControlColor::None, speedThrustInLabelId );

  speedThrustOutLabelId = ESPUI.addControl( ControlType::Slider, "Out", String(speedThrustOutMin), ControlColor::Alizarin, tab3, &sliderCall_speedThrustOutMin );
  ESPUI.addControl( ControlType::Slider, "", String(speedThrustOutMax), ControlColor::Alizarin, speedThrustOutLabelId, &sliderCall_speedThrustOutMax );
  ESPUI.addControl( ControlType::Min, "", String(vel_min), ControlColor::None, speedThrustOutLabelId );
  ESPUI.addControl( ControlType::Max, "", String(vel_max), ControlColor::None, speedThrustOutLabelId );

  // tab 4
  ESPUI.addControl(ControlType::Separator, "Prep", "", ControlColor::None, tab4);
  timesPreptimeLabelId = ESPUI.addControl( ControlType::Number, "Prep Time", String(timesPreptime), ControlColor::Alizarin, tab4, &numberCall_timesPreptime );
  ESPUI.addControl( ControlType::Min, "Prep Time", String(0), ControlColor::None, timesPreptimeLabelId );
  ESPUI.addControl( ControlType::Max, "Prep Time", String(10), ControlColor::None, timesPreptimeLabelId );
  ESPUI.setPanelWide(timesPreptimeLabelId, true);

  ESPUI.addControl(ControlType::Separator, "Pause", "", ControlColor::None, tab4);
  timesPauseMinLabelId = ESPUI.addControl( ControlType::Number, "Pausetime Min", String(timesPauseMin), ControlColor::Alizarin, tab4, &numberCall_timesPauseMin );
  ESPUI.addControl( ControlType::Min, "Pausetime Min", String(0), ControlColor::None, timesPauseMinLabelId );
  ESPUI.addControl( ControlType::Max, "Pausetime Min", String(30), ControlColor::None, timesPauseMinLabelId );
  ESPUI.setPanelWide(timesPauseMinLabelId, true);

  timesPauseMaxLabelId = ESPUI.addControl( ControlType::Number, "Pausetime Max", String(timesPauseMax), ControlColor::Alizarin, tab4, &numberCall_timesPauseMax );
  ESPUI.addControl( ControlType::Min, "Pausetime Max", String(0), ControlColor::None, timesPauseMaxLabelId );
  ESPUI.addControl( ControlType::Max, "Pausetime Max", String(30), ControlColor::None, timesPauseMaxLabelId );
  ESPUI.setPanelWide(timesPauseMaxLabelId, true);

  ESPUI.addControl(ControlType::Separator, "Hold", "", ControlColor::None, tab4);
  timesHoldMinLabelId = ESPUI.addControl( ControlType::Number, "Hold Time Min", String(timesHoldMin), ControlColor::Alizarin, tab4, &numberCall_timesHoldMin );
  ESPUI.addControl( ControlType::Min, "Hold Time Min", String(1), ControlColor::None, timesHoldMinLabelId );
  ESPUI.addControl( ControlType::Max, "Hold Time Min", String(30), ControlColor::None, timesHoldMinLabelId );
  ESPUI.setPanelWide(timesHoldMinLabelId, true);

  timesHoldMaxLabelId = ESPUI.addControl( ControlType::Number, "Hold Time Max", String(timesHoldMax), ControlColor::Alizarin, tab4, &numberCall_timesHoldMax );
  ESPUI.addControl( ControlType::Min, "Hold Time Max", String(1), ControlColor::None, timesHoldMaxLabelId );
  ESPUI.addControl( ControlType::Max, "Hold Time Max", String(30), ControlColor::None, timesHoldMaxLabelId );
  ESPUI.setPanelWide(timesHoldMaxLabelId, true);

  ESPUI.addControl(ControlType::Separator, "Thrusts", "", ControlColor::None, tab4);
  timesThrustMinLabelId = ESPUI.addControl( ControlType::Number, "Thrust Amount Min", String(timesThrustMin), ControlColor::Alizarin, tab4, &numberCall_timesThrustMin );
  ESPUI.addControl( ControlType::Min, "Thrust Amount Min", String(1), ControlColor::None, timesThrustMinLabelId );
  ESPUI.addControl( ControlType::Max, "Thrust Amount Min", String(50), ControlColor::None, timesThrustMinLabelId );
  ESPUI.setPanelWide(timesThrustMinLabelId, true);

  timesThrustMaxLabelId = ESPUI.addControl( ControlType::Number, "Thrust Amount Max", String(timesThrustMax), ControlColor::Alizarin, tab4, &numberCall_timesThrustMax );
  ESPUI.addControl( ControlType::Min, "Thrust Amount Max", String(1), ControlColor::None, timesThrustMaxLabelId );
  ESPUI.addControl( ControlType::Max, "Thrust Amount Max", String(50), ControlColor::None, timesThrustMaxLabelId );
  ESPUI.setPanelWide(timesThrustMaxLabelId, true);

  // tab 5
  probabilityPauseLabelId = ESPUI.addControl( ControlType::Label, "Pause", String(prob_pause), ControlColor::Alizarin, tab5);
  probabilityMfLabelId = ESPUI.addControl( ControlType::Slider, "Mouth", String(prob_mf), ControlColor::Alizarin, tab5, &sliderCall_probMf );
  probabilityTfLabelId = ESPUI.addControl( ControlType::Slider, "Throat", String(prob_tf), ControlColor::Alizarin, tab5, &sliderCall_probTf );
  probabilitySfLabelId = ESPUI.addControl( ControlType::Slider, "Skull", String(prob_sf), ControlColor::Alizarin, tab5, &sliderCall_probSf );
  probabilityHoldLabelId = ESPUI.addControl( ControlType::Slider, "Hold", String(prob_hold), ControlColor::Alizarin, tab5, &sliderCall_probHold );
  probabilityGainLabelId = ESPUI.addControl( ControlType::Slider, "Gain", String(prob_mf), ControlColor::Alizarin, tab5, &sliderCall_gain );
  ESPUI.setPanelWide(probabilityPauseLabelId, true);
  ESPUI.setPanelWide(probabilityMfLabelId, true);
  ESPUI.setPanelWide(probabilityTfLabelId, true);
  ESPUI.setPanelWide(probabilitySfLabelId, true);
  ESPUI.setPanelWide(probabilityHoldLabelId, true);
  ESPUI.setPanelWide(probabilityGainLabelId, true);

  // tab 6
  ESPUI.addControl( ControlType::Switcher, "Airwatch", "1", ControlColor::Alizarin, tab6, &switch_airwatch );
  timeairwatchtriggerLabelId = ESPUI.addControl( ControlType::Number, "Trigger Time", String(airwatch_triggertime), ControlColor::Alizarin, tab6, &numberCall_triggertime );
  timeairwatchresetLabelId = ESPUI.addControl( ControlType::Number, "Reset Time", String(airwatch_resettime), ControlColor::Alizarin, tab6, &numberCall_resettime );
  

  //Refresh the whole page
  ESPUI.jsonReload();
}

//#############################################################################
//ESP Callbacks - StartPage
void select_loadStartPage(Control* sender, int value) {
  Serial.print("Select: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);

  if ((sender->value) == "Default") {
    Serial.println("Default");
  }
  if ((sender->value) == "Opt1") {
    Serial.println("Option1");
    loadOpt1();
  }
  if ((sender->value) == "Opt2") {
    Serial.println("Option2");
    loadOpt2();
  }
  if ((sender->value) == "Opt3") {
    Serial.println("Option3");
    loadOpt3();
  }
  ESPUI.updateVisibility(selectStartPage, false);
}

void callback_input_ssid(Control* sender, int type) {
  Serial.print("Text: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
  ssid = (sender->value);
}

void callback_input_pw(Control* sender, int type) {
  Serial.print("Text: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
  password = (sender->value);
}

void buttonWriteEEPROM(Control* sender, int type) {
  switch (type)
  {
    case B_DOWN:
      //Serial.println("Button DOWN");

      break;

    case B_UP:
      Serial.println("Write to EEPROM");
      EEPROM.writeString(0, ssid);
      EEPROM.writeString(128, password);
      delay(500);
      EEPROM.commit();
      delay(1000);
      ESP.restart();
      break;
  }
}

void buttonEraseEEPROM(Control* sender, int type) {
  switch (type)
  {
    case B_DOWN:
      Serial.println("Button DOWN");
      break;

    case B_UP:
      Serial.println("Button UP");
      for (int i = 0; i < 512; i++) {
        EEPROM.write(i, 0);
        Serial.print("Byte: ");
        Serial.print(i);
        Serial.println(" erased!");
      }
      EEPROM.commit();
      delay(500);
      break;
  }
}


//#############################################################################
//ESP Callbacks - Main
void button_reset(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      // Serial.println("Start");
      break;

    case B_UP:
      ESP.restart();
      // Serial.println("Started");
      break;
  }
}

void button_start(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      // Serial.println("Start");
      break;

    case B_UP:
      if (selectedOpt == 3) {
        startOpt3();
      } else {
        start();
      }

      // Serial.println("Started");
      break;
  }
}

void button_stop(Control *sender, int type) {
  switch (type) {
    case B_DOWN:
      // Serial.println("Start");
      if (selectedOpt == 3) {
        stoppOpt3();
      } else {
        stopp();
      }

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
void sliderCall_posIn( Control* sender, int type ) {
  Serial.print("posIn: "); Serial.println( sender->value );
  posIn = (sender->value).toInt();
}

void sliderCall_posOut( Control* sender, int type ) {
  Serial.print("posOut: "); Serial.println( sender->value );
  posOut = (sender->value).toInt();
  if (posOut >= posIn) {
    posOut = posIn - 1;
    ESPUI.updateSlider(posOutSliderLabelId, posOut);
    Serial.print("Error - new posOut: "); Serial.println(posOut);
  }
}

void sliderCall_strokeLength( Control* sender, int type ) {
  Serial.print("strokeLength: "); Serial.println( sender->value );
  strokeLength = (sender->value).toInt();
  posIn = posOut + strokeLength;
  if (posIn > dist_max) {
    posIn = dist_max;
    strokeLength = dist_max - strokeOffset;
    ESPUI.updateSlider(strokeLengthLabelId, strokeLength);
    Serial.print("Error - new strokeLength: "); Serial.println(strokeLength);
  }
  Serial.print("posIn: "); Serial.println(posIn);
  Serial.print("posOut: "); Serial.println(posOut);
}

void sliderCall_strokeOffset( Control* sender, int type ) {
  Serial.print("strokeOffset: "); Serial.println( sender->value );
  strokeOffset = (sender->value).toInt();
  posOut = strokeOffset;
  posIn = posOut + strokeLength;
  if (posIn > dist_max) {
    posIn = dist_max;
    posOut = dist_max - strokeLength;
    strokeOffset = dist_max - strokeLength;
    ESPUI.updateSlider(strokeOffsetLabelId, strokeOffset);
    Serial.print("Error - new strokeOffset: "); Serial.println(strokeOffset);
  }
  Serial.print("posIn: "); Serial.println(posIn);
  Serial.print("posOut: "); Serial.println(posOut);
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
//ESP Callbacks - Settings - Speed
void sliderCall_speedThrustIn( Control* sender, int type ) {
  Serial.print("speedThrustIn: "); Serial.println( sender->value );
  speedThrustIn = (sender->value).toInt();
}

void sliderCall_speedThrustOut( Control* sender, int type ) {
  Serial.print("speedThrustOut: "); Serial.println( sender->value );
  speedThrustOut = (sender->value).toInt();
}

void switch_syncspeed(Control *sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("Active:");
      flag_syncspeed = true;
      ESPUI.updateVisibility(speedThrustOutSliderLabelId, false);
      break;

    case S_INACTIVE:
      Serial.print("Inactive");
      flag_syncspeed = false;
      ESPUI.updateVisibility(speedThrustOutSliderLabelId, true);
      speedThrustOut = speedThrustIn;
      ESPUI.updateSlider(speedThrustOutSliderLabelId, speedThrustOut);
      break;
  }

  Serial.print(" ");
  Serial.println(sender->id);
}

void sliderCall_speedHoldInMin( Control* sender, int type ) {
  Serial.print("speedHoldInMin: "); Serial.println( sender->value );
  speedHoldInMin = (sender->value).toInt();
  if (speedHoldInMin > speedHoldInMax) {
    speedHoldInMin = speedHoldInMax - 1;
    ESPUI.updateSlider(speedHoldInLabelId, speedHoldInMin);
  }
}

void sliderCall_speedHoldInMax( Control* sender, int type ) {
  Serial.print("speedHoldInMax: "); Serial.println( sender->value );
  speedHoldInMax = (sender->value).toInt();
  if (speedHoldInMax < speedHoldInMin) {
    speedHoldInMin = speedHoldInMax - 1;
    ESPUI.updateSlider(speedHoldInLabelId, speedHoldInMin);
  }
}

void sliderCall_speedHoldOutMin( Control* sender, int type ) {
  Serial.print("speedHoldOutMin: "); Serial.println( sender->value );
  speedHoldOutMin = (sender->value).toInt();
}

void sliderCall_speedHoldOutMax( Control* sender, int type ) {
  Serial.print("speedHoldOutMax: "); Serial.println( sender->value );
  speedHoldOutMax = (sender->value).toInt();
}

void sliderCall_speedThrustInMin( Control* sender, int type ) {
  Serial.print("speedThrustInMin: "); Serial.println( sender->value );
  speedThrustInMin = (sender->value).toInt();
}

void sliderCall_speedThrustInMax( Control* sender, int type ) {
  Serial.print("speedThrustInMax: "); Serial.println( sender->value );
  speedThrustInMax = (sender->value).toInt();
}

void sliderCall_speedThrustOutMin( Control* sender, int type ) {
  Serial.print("speedThrustOutMin: "); Serial.println( sender->value );
  speedThrustOutMin = (sender->value).toInt();
}

void sliderCall_speedThrustOutMax( Control* sender, int type ) {
  Serial.print("speedThrustOutMax: "); Serial.println( sender->value );
  speedThrustOutMax = (sender->value).toInt();
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

void numberCall_timesPreptime( Control* sender, int type ) {
  Serial.print("Preptime: "); Serial.println( sender->value );
  timesPreptime = (sender->value).toInt();
}

void numberCall_timesPauseMin( Control* sender, int type ) {
  Serial.print("timesPauseMin: "); Serial.println( sender->value );
  timesPauseMin = (sender->value).toInt();
}

void numberCall_timesHoldMin( Control* sender, int type ) {
  Serial.print("timesHoldMin: "); Serial.println( sender->value );
  timesHoldMin = (sender->value).toInt();
}

void numberCall_timesThrustMin( Control* sender, int type ) {
  Serial.print("timesThrustMin: "); Serial.println( sender->value );
  timesThrustMin = (sender->value).toInt();
}

void numberCall_timesPauseMax( Control* sender, int type ) {
  Serial.print("timesPauseMax: "); Serial.println( sender->value );
  timesPauseMax = (sender->value).toInt();
}

void numberCall_timesHoldMax( Control* sender, int type ) {
  Serial.print("timesHoldMax: "); Serial.println( sender->value );
  timesHoldMax = (sender->value).toInt();
}

void numberCall_timesThrustMax( Control* sender, int type ) {
  Serial.print("timesThrustMax: "); Serial.println( sender->value );
  timesThrustMax = (sender->value).toInt();
}



//#############################################################################
//ESP Callbacks - Settings - Probability
void sliderCall_probMf( Control* sender, int type ) {
  Serial.print("Probability - Pause: "); Serial.println( sender->value );
  prob_mf = (sender->value).toInt();
  if ((prob_mf + prob_tf + prob_sf + prob_hold) <= 100){
    prob_pause = 100 - (prob_mf + prob_tf + prob_sf + prob_hold);
    ESPUI.updateLabel(probabilityPauseLabelId, String(prob_pause));
  } 
  else {
    prob_pause = 0;
    prob_mf = 100 -(prob_tf + prob_sf + prob_hold);  
    ESPUI.updateSlider(probabilityMfLabelId, prob_mf);
    ESPUI.updateLabel(probabilityPauseLabelId, String(prob_pause));
  }
}

void sliderCall_probTf( Control* sender, int type ) {
  Serial.print("Probability - HeavyTask: "); Serial.println( sender->value );
  prob_tf = (sender->value).toInt();
  if ((prob_mf + prob_tf + prob_sf + prob_hold) <= 100){
    prob_pause = 100 - (prob_mf + prob_tf + prob_sf + prob_hold);
    ESPUI.updateLabel(probabilityPauseLabelId, String(prob_pause));
  } 
  else {
    prob_pause = 0;
    prob_tf = 100 -(prob_mf + prob_sf + prob_hold);  
    ESPUI.updateSlider(probabilityTfLabelId, prob_tf);
    ESPUI.updateLabel(probabilityPauseLabelId, String(prob_pause));
  }
}

void sliderCall_probSf( Control* sender, int type ) {
  Serial.print("Probability - Hold: "); Serial.println( sender->value );
  prob_sf = (sender->value).toInt();
  if ((prob_mf + prob_tf + prob_sf + prob_hold) <= 100){
    prob_pause = 100 - (prob_mf + prob_tf + prob_sf + prob_hold);
    ESPUI.updateLabel(probabilityPauseLabelId, String(prob_pause));
  } 
  else {
    prob_pause = 0;
    prob_sf = 100 -(prob_mf + prob_tf + prob_hold);  
    ESPUI.updateSlider(probabilitySfLabelId, prob_sf);
    ESPUI.updateLabel(probabilityPauseLabelId, String(prob_pause));
  }
}

void sliderCall_probHold( Control* sender, int type ) {
  Serial.print("Probability - Hold: "); Serial.println( sender->value );
  prob_hold = (sender->value).toInt();
  if ((prob_mf + prob_tf + prob_sf + prob_hold) <= 100){
    prob_pause = 100 - (prob_mf + prob_tf + prob_sf + prob_hold);
    ESPUI.updateLabel(probabilityPauseLabelId, String(prob_pause));
  } 
  else {
    prob_pause = 0;
    prob_hold = 100 -(prob_mf + prob_tf + prob_sf);  
    ESPUI.updateSlider(probabilityHoldLabelId, prob_hold);
    ESPUI.updateLabel(probabilityPauseLabelId, String(prob_pause));
  }
}

void sliderCall_gain( Control* sender, int type ) {
  Serial.print("Gain: "); Serial.println( sender->value );
  gain = (sender->value).toInt();
}

//#############################################################################
//ESP Callbacks - Settings - Airwatch
void switch_airwatch(Control *sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("Active:");
      enable_airwatch = true;
      break;

    case S_INACTIVE:
      Serial.print("Inactive");
      enable_airwatch = false;
      break;
  }

  Serial.print(" ");
  Serial.println(sender->id);
}

void numberCall_triggertime( Control* sender, int type ) {
  Serial.print("Airwatch Trigger Time: "); Serial.println( sender->value );
  airwatch_triggertime = (sender->value).toInt();
}

void numberCall_resettime( Control* sender, int type ) {
  Serial.print("Airwatch Reset Time: "); Serial.println( sender->value );
  airwatch_resettime = (sender->value).toInt();
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

void disable_torque() {
  digitalWrite(pinEnb, HIGH);
  flag_statuson = false;
  flag_torque = false;
  stepper->setCurrentPosition(0);
  stepchain_busy = false;
  stp = 0;
}

void enable_torque() {
  digitalWrite(pinEnb, LOW);
  flag_statuson = false;
  flag_torque = true;
  stepper->setCurrentPosition(0);
  stepchain_busy = false;
  stp = 0;
}

int calc_maxPossSpeed(int _acc, int _posin, int _posout) {
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
        if (stepchainverbose) {
          Serial.print("Stepchain: ");
          Serial.println(stp);
        }

        timestamp_stpchain_stop = millis();
        millispased = (timestamp_stpchain_stop - timestamp_stpchain_start);
        if (millispased < 1) {
          millispased = 1;
        }
        bpm = 60000.0 / millispased;
        ESPUI.print(BPMLabelId, String(bpm));
        timestamp_stpchain_start = millis();

        targetpos = _posIn;
        stepper->setSpeedInHz(_velIn);
        stepper->setAcceleration(_accIn);
        stepper->applySpeedAcceleration();
        stp = 2;
        break;

      case 2: //Execute movement for in
        if (stepchainverbose) {
          Serial.print("Stepchain: ");
          Serial.println(stp);
        }
        stepper->moveTo(targetpos);
        stp = 3;
        break;

      case 3: // Check if in pos
        //#####################################################
        //Test for Different Deceleration

        Serial.println(-(stepper->getCurrentSpeedInMilliHz()));
        /*
          if ((-(stepper->getCurrentSpeedInMilliHz()) >= (_velIn * 999)) && (!flag_dec)) {
          Serial.println("New Deceleration");
          stepper->setAcceleration(_accOut);
          stepper->applySpeedAcceleration();
          flag_dec = true;
          }*/

        if ((stepper->rampState() & RAMP_STATE_MASK) == RAMP_STATE_COAST) {
          Serial.println("Condition from gin66 is TRUE");
          stepper->setAcceleration(_accOut);
          stepper->applySpeedAcceleration();
          flag_dec = true;
        }

        //#####################################################

        if (stepper->getCurrentPosition() == targetpos) {
          if (stepchainverbose) {
            Serial.print("Stepchain: ");
            Serial.println(stp);
          }
          stp = 4;
        }
        break;

      case 4: // Wait inside
        if (stepchainverbose) {
          Serial.print("Stepchain: ");
          Serial.println(stp);
        }
        flag_dec = false;
        if (waitactive) {
          timestamp2 = millis();
          if ((timestamp2 - timestamp1) >= 100 ) {
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
        if (stepchainverbose) {
          Serial.print("Stepchain: ");
          Serial.println(stp);
        }
        targetpos = _posOut;
        if (flag_syncspeed) {
          stepper->setSpeedInHz(_velIn);
        }
        else {
          stepper->setSpeedInHz(_velOut);
        }
        stepper->setAcceleration(_accOut);
        stepper->applySpeedAcceleration();
        stp = 6;
        break;

      case 6: //Execute movement for out
        if (stepchainverbose) {
          Serial.print("Stepchain: ");
          Serial.println(stp);
        }
        stepper->moveTo(targetpos);
        stp = 7;
        break;

      case 7: // Check if in pos
        if (stepper->getCurrentPosition() == targetpos) {
          if (stepchainverbose) {
            Serial.print("Stepchain: ");
            Serial.println(stp);
          }
          stp = 8;
        }
        break;

      case 8: // Wait outside
        if (stepchainverbose) {
          Serial.print("Stepchain: ");
          Serial.println(stp);
        }
        if (waitactive) {
          timestamp2 = millis();
          if ((timestamp2 - timestamp1) >= 100 ) {
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
          if (stepchainverbose) {
            Serial.print("Stepchain: ");
            Serial.println(stp);
          }
          if (stepchainverbose) {
            Serial.println("Stepchain finished");
          }
          stepchain_busy = false;
          amountdone = 0;
          stp = 0;
        }
        else {
          if (stepchainverbose) {
            Serial.print("Stepchain: ");
            Serial.println(stp);
          }
          if (stepchainverbose) {
            Serial.print(_amount - amountdone);
            Serial.println(" to go. --> Stepchain returning");
          }
          stp = 1;
        }
        break;
    }
  }
}

void stepchainOpt3(int _preptime, int _amount, int _posIn, int _posOut, int _accIn, int _accOut, int _velIn,  int _velOut,  int _pauseIn, int _pauseOut) {

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
          flag_tf = false;
          flag_paus = false;
          flag_mf = false;
          flag_sf = false;
          flag_task = false;
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

void airwatch(int _maxtime, int _resettime){
  switch(state){
    //Not armed
    case 0:
    if(-(stepper->getCurrentPosition()) > posEnt*stepspermm){
      state=1;
    }
    break;

    //Arming
    case 1:
    airwatchtime1 = millis();
    airwatchtime2 = millis();
    Serial.println("Airwatch arming");
    state=2;
    break;

    //Armed
    case 2:
    if(-(stepper->getCurrentPosition()) < posEnt*stepspermm){
      state=3;
    }
    airwatchtime2 = millis();
    if((airwatchtime2 - airwatchtime1) >= _maxtime){
      state=5;
    }
    break;

    //Disarming
    case 3:
    airwatchtime3 = millis();
    airwatchtime4 = millis();
    Serial.println("Airwatch dis-arming");
    state=4;
    break;

    //Disarmed
    case 4:
    if(-(stepper->getCurrentPosition()) > posEnt*stepspermm){
      state=2;
    }
    airwatchtime4 = millis();
    if((airwatchtime4 - airwatchtime3) >= _resettime){
      state=0;
    }
    break;
    
    //Triggering
    case 5:
    Serial.println("Airwatch Triggered");
    state=6;
    break;

    //Triggered
    case 6:
    Serial.println("Autoreset Debugging");

    Serial.print("Stepchain: "); Serial.println(stp);
    Serial.println("Stepchain abborted");
    flag_hold = false;
    flag_tf = false;
    flag_paus = false;
    flag_mf = false;
    flag_sf = false;
    flag_task = false;
    stepchain_busy = false;
    amountdone = 0;
    stp = 0;
    ESPUI.print(TimerLabelId, "Airwatch" );
    flag_airwatch = true;

    state=0;
    
    nextTask();   
  }
}

struct command task_pause() {
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
  taskcmd.cmd_pauseout = random(timesPauseMin, timesPauseMax + 1);
  taskcmd.cmd_message = "Pause for " + String(taskcmd.cmd_pauseout);
  taskcmd.cmd_hint = "Catch your breath";

  return taskcmd;
}

struct command task_mf() {
  struct command taskcmd;
  int posin_dice;
  int posout_dice;
  String str_posin;
  String str_posout;
  String message;
  String hint;

  taskcmd.taskidentifier = 'M';
  posin_dice = random(3, 5);
  posout_dice = random(1, posin_dice);
  taskcmd.cmd_preptime = 0;
  taskcmd.cmd_amount = random(timesThrustMin, timesThrustMax + 1);
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
  taskcmd.cmd_accin = 8000;
  taskcmd.cmd_accout = 8000;
  taskcmd.cmd_velin = random(speedThrustInMin, speedThrustInMax + 1);
  taskcmd.cmd_velout = random(speedThrustOutMin, speedThrustOutMax + 1);
  taskcmd.cmd_pausein = 0;
  taskcmd.cmd_pauseout = 0;
  taskcmd.cmd_message = "Suck it from " + str_posout + str_posin + " for " + String(taskcmd.cmd_amount) + " Times" ;
  taskcmd.cmd_hint = "In - Out";

  return taskcmd;
}

struct command task_tf() {
  struct command taskcmd;
  int posin_dice;
  int posout_dice;
  String str_posin;
  String str_posout;
  String message;
  String hint;

  taskcmd.taskidentifier = 'T';
  posin_dice = random(6, 7);
  posout_dice = random(5, posin_dice);
  taskcmd.cmd_preptime = 0;
  taskcmd.cmd_amount = random(timesThrustMin, timesThrustMax + 1);
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
      str_posin = "Ballsdeep";
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
  taskcmd.cmd_accin = 8000;
  taskcmd.cmd_accout = 8000;
  taskcmd.cmd_velin = random(speedThrustInMin, speedThrustInMax + 1);
  taskcmd.cmd_velout = random(speedThrustOutMin, speedThrustOutMax + 1);
  taskcmd.cmd_pausein = 0;
  taskcmd.cmd_pauseout = 0;
  taskcmd.cmd_message = "Throatfuck from " + str_posout + str_posin + " for " + String(taskcmd.cmd_amount) + " Times" ;
  taskcmd.cmd_hint = "In - Out";

  return taskcmd;
}

struct command task_sf() {
  struct command taskcmd;
  int posin_dice;
  int posout_dice;
  String str_posin;
  String str_posout;
  String message;
  String hint;

  taskcmd.taskidentifier = 'S';
  posin_dice = random(5, 7);
  posout_dice = random(1, 4);
  taskcmd.cmd_preptime = 0;
  taskcmd.cmd_amount = random(timesThrustMin, timesThrustMax + 1);
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
      str_posin = "Ballsdeep";
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
  taskcmd.cmd_accin = 8000;
  taskcmd.cmd_accout = 8000;
  taskcmd.cmd_velin = random(speedThrustInMin, speedThrustInMax + 1);
  taskcmd.cmd_velout = random(speedThrustOutMin, speedThrustOutMax + 1);
  taskcmd.cmd_pausein = 0;
  taskcmd.cmd_pauseout = 0;
  taskcmd.cmd_message = "Facefuck from " + str_posout + str_posin + " for " + String(taskcmd.cmd_amount) + " Times" ;
  taskcmd.cmd_hint = "In - Out";

  return taskcmd;
}

struct command task_hold() {
  struct command taskcmd;
  int posin_dice;
  int posout_dice;
  String message;
  String hint;

  taskcmd.taskidentifier = 'H';
  taskcmd.cmd_preptime = timesPreptime;
  posin_dice = random(5, 7);
  posout_dice = random(1, posin_dice);
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
      message = "Hold it in your Throat for ";
      break;

    case 6:
      taskcmd.cmd_posin = posBde;
      message = "Hold it ballsdeep for ";
      break;
  }

  taskcmd.cmd_posout = 0;
  taskcmd.cmd_accin = 5000;
  taskcmd.cmd_accout = 5000;
  taskcmd.cmd_velin = random(speedHoldInMin, speedHoldInMax + 1);
  taskcmd.cmd_velout = random(speedHoldOutMin, speedHoldOutMax + 1);
  taskcmd.cmd_pausein = random(timesHoldMin, timesHoldMax + 1);
  taskcmd.cmd_pauseout = 0;
  taskcmd.cmd_message = message + String(taskcmd.cmd_pausein) + " Seconds" ;
  taskcmd.cmd_hint = "No Air";

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

bool inRange(int val, int minimum, int maximum){
  return ((minimum <= val) && (val < maximum));
}

struct command randomizeTask() {
  struct command meincmd;
  int prob_pause_storage;
  
  if(flag_airwatch){
    //Force Pause after Airwatch Trigger
    Serial.println("Forcing Pause");
    prob_pause_storage = prob_pause;
    prob_pause = 100;
  }
  
    int ran = random(0, 100);

  
  if(inRange(ran, 0, prob_pause)){
    meincmd = task_pause();
    Serial.println("pause");
  }
  if(inRange(ran, prob_pause, prob_pause + prob_mf)){
    meincmd = task_mf();
    Serial.println("mf");
  }
  if(inRange(ran, prob_pause + prob_mf, prob_pause + prob_mf + prob_tf)){
    meincmd = task_tf();
    Serial.println("tf");
  }
  if(inRange(ran, prob_pause + prob_mf + prob_tf, prob_pause + prob_mf + prob_tf + prob_sf)){
    meincmd = task_sf();
    Serial.println("sf");
  }
  if(inRange(ran, prob_pause + prob_mf + prob_tf + prob_sf, 100)){
    meincmd = task_hold();
    Serial.println("hold");
  } 
  if (reverseStepper) {
    meincmd.cmd_posin = -meincmd.cmd_posin;
    meincmd.cmd_posout = -meincmd.cmd_posin;
  }

  if(flag_airwatch){
    prob_pause = prob_pause_storage;
    flag_airwatch = false;
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

    case 'M':
      Serial.println("Cmd is M");
      flag_task = true;
      break;

    case 'T':
      Serial.println("Cmd is T");
      flag_task = true;
      break;

    case 'S':
      Serial.println("Cmd is S");
      flag_task = true;
      break;

    case 'H':
      Serial.println("Cmd is H");
      flag_task = true;
      break;

    default:
      Serial.println("No Cmd");
      stopp();
      break;
  }
  msg = activecommand.cmd_message;
  ESPUI.print(CurrentTaskLabelId, msg);
}

void startOpt3() {
  digitalWrite(pinEnb, LOW);
  flag_statuson = true;
  //tasknumber = 1;
  ESPUI.print(ProgramLabelId, String(tasknumber));
  activecommand = randomizeTask();
  printTask();
  taskHandler();
}

void stoppOpt3() {
  flag_statuson = false;
  flag_hold = false;
  flag_task = false;
  flag_paus = false;
  digitalWrite(pinEnb, HIGH);
  stepper->setCurrentPosition(0);
  stepchain_busy = false;
  stp = 0;
  ESPUI.print(CurrentTaskLabelId, "- - -");
  //ESPUI.print(ProgramLabelId, "- - -");
  ESPUI.print(TimerLabelId, "- - -");
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

  //#############################################################################
  // Basic Loops
  void setup(void) {
    //just for debugging
    Serial.begin(115200);
    //
    pinMode(pinEnb, OUTPUT);
    digitalWrite(pinEnb, LOW);

    EEPROM.begin(512);

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
    connecttowifi();


    //#############################################################################
    // ESPUI stuff
    ESPUI.setVerbosity(Verbosity::Quiet);
    loadStartPage();
    ESPUI.jsonInitialDocumentSize = 16000; // Default is 8000. Thats not enough for this many widgeds
    ESPUI.begin("OSSM");
  }

  void loop(void) {
    if (selectedOpt == 3) {
      if(enable_airwatch){
        airwatch(airwatch_triggertime,airwatch_resettime);
      }
      if (flag_task) {
        stepchain_busy = true;
        stepchainOpt3(activecommand.cmd_preptime, activecommand.cmd_amount, -activecommand.cmd_posin * stepspermm, -activecommand.cmd_posout * stepspermm, activecommand.cmd_accin * stepspermm, activecommand.cmd_accout * stepspermm, activecommand.cmd_velin * stepspermm, activecommand.cmd_velout * stepspermm, activecommand.cmd_pausein, activecommand.cmd_pauseout);
      }
      if (flag_paus) {
        stepchain_busy = true;
        stepchainOpt3(activecommand.cmd_preptime, activecommand.cmd_amount, -activecommand.cmd_posin * stepspermm, -activecommand.cmd_posout * stepspermm, activecommand.cmd_accin, activecommand.cmd_accout, activecommand.cmd_velin * stepspermm, activecommand.cmd_velout * stepspermm, activecommand.cmd_pausein, activecommand.cmd_pauseout);
        lastTaskPause = true;
      }
    } else {
      if (flag_statuson && flag_torque) {
        stepchain_busy = true;
        stepchain(0, 1, -posIn * stepspermm, -posOut * stepspermm, accThrustIn * stepspermm, accThrustOut * stepspermm, speedThrustIn * stepspermm, speedThrustOut * stepspermm, timesPauseIn, timesPauseOut);
      }
    }
 }
