#include <Arduino.h>          // Basic Needs
#include <ArduinoJson.h>      // Needed for the Bubble APP
#include <ESP_FlexyStepper.h> // Current Motion Control
#include <Wire.h>             // Used for i2c connections (Remote OLED Screen)

#include "FastLED.h"          // Used for the LED on the Reference Board (or any other pixel LEDS you may add)
#include "OSSM_Config.h"      // START HERE FOR Configuration
#include "OSSM_PinDef.h"      // This is where you set pins specific for your board

///////////////////////////////////////////
////
////  RTOS Settings
////
///////////////////////////////////////////

#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define INCLUDE_uxTaskGetStackHighWaterMark 1


///////////////////////////////////////////
////
////  Includes for Xtoys BLE Integration
////
///////////////////////////////////////////

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLE2904.h"
#include <Preferences.h>
#include <list>               // Is filled with the Cached Commands from Xtoys

///////////////////////////////////////////
////
////  To Debug or not to Debug
////
///////////////////////////////////////////

// Uncomment the following line if you wish to print DEBUG info
#define DEBUG

#ifdef DEBUG
#define LogDebug(...) Serial.println(__VA_ARGS__)
#define LogDebugFormatted(...) Serial.printf(__VA_ARGS__)
#else
#define LogDebug(...) ((void)0)
#define LogDebugFormatted(...) ((void)0)
#endif

// Homing
volatile bool g_has_not_homed = true;
bool REMOTE_ATTACHED = false;

// create the stepper motor object
ESP_FlexyStepper stepper;

// Current command state
volatile float strokePercentage = 0;
volatile float speedPercentage = 0;
volatile float deceleration = 0;
volatile int targetPosition;
volatile int targetDuration;
volatile int targetStepperPosition = 0;
volatile int remainingCommandTime = 0;
volatile float accelspeed = 0;

// Create tasks for checking pot input or web server control, and task to handle
// planning the motion profile (this task is high level only and does not pulse
// the stepper!)
TaskHandle_t wifiTask = nullptr;
TaskHandle_t getInputTask = nullptr;
TaskHandle_t motionTask = nullptr;
TaskHandle_t estopTask = nullptr;
TaskHandle_t oledTask = nullptr;
TaskHandle_t bleTask = nullptr;
TaskHandle_t blemTask = nullptr;

#define BRIGHTNESS 170
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define LED_PIN 25
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Declarations
// TODO: Document functions
void setLedRainbow(CRGB leds[]);
void bleConnectionTask(void *pvParameters);
void blemotionTask(void *pvParameters);

bool stopSwitchTriggered = 0;

///////////////////////////////////////////
////
////
//// Xtoys Integration BLE Management
////
////
////////////////////////////////////////////

const char* FIRMWARE_VERSION = "v1.1";

#define SERVICE_UUID                "e556ec25-6a2d-436f-a43d-82eab88dcefd"
#define CONTROL_CHARACTERISTIC_UUID "c4bee434-ae8f-4e67-a741-0607141f185b"
#define SETTINGS_CHARACTERISTIC_UUID "fe9a02ab-2713-40ef-a677-1716f2c03bad"

// WRITE
// T-Code messages in the format:
// ex. L199I100 = move linear actuator to the 99% position over 100ms
// ex. L10I100 = move linear actuator to the 0% position over 100ms
// DSTOP = stop
// DENABLE = enable motor (non-standard T-Code command)
// DDISABLE = disable motor (non-standard T-Code command)

// WRITE
// Preferences in the format:
// minSpeed:200 = set minimum speed of half-stroke to 200ms (used by XToys client)
// maxSpeed:2000 = set maximum speed of half-stroke to 2000ms (used by XToys client)
// READ
// Returns all preference values in the format:
// minSpeed:200,maxSpeed:2000,maxOut:0,maxIn:1000

#define NUM_NONE 0
#define NUM_CHANNEL 1     // Channel from Xtoys T-Code
#define NUM_PERCENT 2     // Percent of Position from Xtoys T-Code
#define NUM_DURATION 3    // Duration to Position from Xtoys T-Code in ms
#define NUM_VALUE 4

#define DEFAULT_MAX_SPEED 50        // Max Sending Resolution in ms should not be changed right now
#define DEFAULT_MIN_SPEED 1000      // Min Sending Resolution in ms should not be changed right now


// Global Variables - Bluetooth Configuration
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *controlCharacteristic;
BLECharacteristic *settingsCharacteristic;
BLEService *infoService;
BLECharacteristic *softwareVersionCharacteristic;

// Preferences
Preferences preferences;
int maxInPosition;
int maxOutPosition;
int maxSpeed;
int minSpeed;
int changetime = 100; // Ms wich the system is slowing down when change of code is dected for safety


// Other
bool deviceConnected = false;
unsigned long previousMillis = 0; 
unsigned long tcodeMillis = 0; 
std::list<std::string> pendingCommands = {};

// Create Voids for Xtoys
void updateSettingsCharacteristic();
void processCommand(std::string msg);
void moveTo(int targetPosition, int targetDuration);

// Read actions and numeric values from T-Code command
void processCommand(std::string msg) {
  
  char command = NULL;
  int channel = 0;
  int targetAmount = 0;
  int targetDuration = 0;
  int numBeingRead = NUM_NONE;

  for (char c : msg) {
    switch (c) {
      case 'l':
      case 'L':
        command = 'L';
        numBeingRead = NUM_CHANNEL;
        break;
      case 'i':
      case 'I':
        numBeingRead = NUM_DURATION;
        break;
      case 'D':
      case 'd':
        command = 'D';
        numBeingRead = NUM_CHANNEL;
        break;
      case 'v':
      case 'V':
        numBeingRead = NUM_VALUE;
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        int num = c - '0'; // convert from char to numeric int
        switch (numBeingRead) {
          case NUM_CHANNEL:
            channel = num;
            numBeingRead = NUM_PERCENT;
            break;
          case NUM_PERCENT:
            targetAmount = targetAmount * 10 + num;
            break;
          case NUM_DURATION:
            targetDuration = targetDuration * 10 + num;
            break;
        }
        break;
    }
  }
  // if amount was less than 5 digits increase to 5 digits
  // t-code message is a value to the right of the decimal place so we need a consistent length to work with in moveTo command
  // ex L99 means 0.99000 and L10010 means 0.10010
  if (command == 'L' && channel == 1) {
    moveTo(targetAmount, targetDuration);
    //} 
  } else if (command == 'D') { // not handling currently
  } else {
    Serial.print("Invalid command: ");
    Serial.println(msg.c_str());
  }

}

// Dedicated MoveCommand for Xtoys for Position based movement
void moveTo(int targetPosition, int targetDuration){
        stepper.releaseEmergencyStop();
        unsigned long currentMillis = millis();                                       // Get Time 
        float targetspeed;
        float currentStepperPosition = stepper.getCurrentPositionInMillimeters();      // Get Current Position from Stepper 
        float targetxStepperPosition = map(targetPosition, 0, 100, (maxStrokeLengthMm -(strokeZeroOffsetmm * 0.5)), 0.0); // Calculate Target positon Mulitply for Calculation Speed. 
        float travelInMM = targetxStepperPosition -currentStepperPosition; // Get Travel Distance to Target Position

        if(currentMillis - previousMillis <= changetime){
          targetspeed = (abs(travelInMM) / targetDuration) * xtoySpeedScaling *0.2;
        } else {
          targetspeed = (abs(travelInMM) / targetDuration) * xtoySpeedScaling;  // Calculate Target speed from Travel Distance with xtoySpeedScaling 
        }
        targetspeed = constrain(targetspeed, 0, maxSpeedMmPerSecond);
        LogDebugFormatted("Targetspeed %ld \n", static_cast<long int>(targetspeed));
        LogDebugFormatted("TargetxStepperPosition %ld \n", static_cast<long int>(targetxStepperPosition));
        // Security if something went wrong and were out of target range kill all.
        if(targetxStepperPosition < (maxStrokeLengthMm + (strokeZeroOffsetmm * 0.5)) || targetxStepperPosition >= 0.0){
        stepper.setSpeedInMillimetersPerSecond(targetspeed);                          // Sets speed to Stepper 
        stepper.setAccelerationInMillimetersPerSecondPerSecond(xtoyAccelartion);      // Sets Accelartion
        stepper.setTargetPositionInMillimeters(targetxStepperPosition);               //Sets Position 

        } else {
          stepper.emergencyStop();
          vTaskSuspend(blemTask);
          LogDebugFormatted("Position out of Safety %ld \n", static_cast<long int>(targetxStepperPosition));
        }
}

// Received request to update a setting
class SettingsCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string msg = characteristic->getValue();

    LogDebug("");
    Serial.println(msg.c_str());

    std::size_t pos = msg.find(':');
    std::string settingKey = msg.substr(0, pos);
    std::string settingValue = msg.substr(pos + 1, msg.length());

    if (settingKey == "maxSpeed") {
      maxSpeed = atoi(settingValue.c_str());
      preferences.putInt("maxSpeed", maxSpeed);
    }
    if (settingKey == "minSpeed") {
      minSpeed = atoi(settingValue.c_str());
      preferences.putInt("minSpeed", minSpeed);
    }
    Serial.print("Setting pref ");
    Serial.print(settingKey.c_str());
    Serial.print(" to ");
    Serial.println(settingValue.c_str());
    updateSettingsCharacteristic();
  }
};

// Received T-Code command
class ControlCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string msg = characteristic->getValue();

    LogDebug("Received command: ");
    Serial.println(msg.c_str());
    tcodeMillis = millis() - tcodeMillis;
    LogDebugFormatted("comand time:  %ld \n", static_cast<long int>(tcodeMillis));
    tcodeMillis = millis();

    // check for messages that might need to be immediately handled
    if (msg == "DSTOP") { // stop request
      LogDebug("STOP");
      pendingCommands.clear();
      return;
    } else if (msg == "DENABLE") { // Does nothing anymore not needed
      return;
    } else if (msg == "DDISABLE") { // disable stepper motor
      LogDebug("DISABLE");
      pendingCommands.clear();
      return;
    }
    if (msg.front() == 'D') { // device status message (technically the code isn't handling any T-Code 'D' messages currently
      return;
    }
    if (msg.back() == 'C') { // movement command includes a clear existing commands flag, clear queue start slow movement counter
      pendingCommands.clear();
      previousMillis = millis();  
      pendingCommands.push_back(msg);
      return;
    }
    // probably a normal movement command, store it to be run after other movement commands are finished
    if (pendingCommands.size() < 100) {
      pendingCommands.push_back(msg);
      LogDebugFormatted("# of pending commands:  %ld \n", static_cast<long int>(pendingCommands.size()));
    } else {
      LogDebug("Too many commands in queue. Dropping: ");
      Serial.println(msg.c_str());
    }
  }
};

// Client connected to OSSM over BLE
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServerm, esp_ble_gatts_cb_param_t *param) {
    deviceConnected = true;
    Serial.println("BLE Connected");
     esp_ble_conn_update_params_t conn_params = {0};  
     memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
     /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
     conn_params.latency = 0;  
     conn_params.max_int = 0x12;    // max_int = 0x48*1.25ms  
     conn_params.min_int = 0x12;    // min_int = 0x24*1.25ms   
     conn_params.timeout = 800;     // timeout = *10ms  
	  //start sent the update connection parameters to the peer device.
	  esp_ble_gap_update_conn_params(&conn_params);
    vTaskResume(blemTask);
    moveTo(0, 500);             // Pulls it to 0 Position Fully out for Xtoys
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Disconnected");
    pServer->startAdvertising();
    vTaskSuspend(blemTask);
  }
};


void updateSettingsCharacteristic() {
  String settingsInfo = String("maxIn:") + maxInPosition + ",maxOut:" + maxOutPosition + ",maxSpeed:" + maxSpeed + ",minSpeed:" + minSpeed;
  settingsCharacteristic->setValue(settingsInfo.c_str());
}
///////////////////////////////////////////
////
////
////  VOID SETUP -- Here's where it's hiding
////
////
///////////////////////////////////////////

void setup()
{
    Serial.begin(115200);
    LogDebug("\n Starting");
    delay(200);

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(150);
    setLedRainbow(leds);
    FastLED.show();
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

    preferences.begin("OSSM", false);
    maxSpeed = preferences.getInt("maxSpeed", DEFAULT_MAX_SPEED);
    minSpeed = preferences.getInt("minSpeed", DEFAULT_MIN_SPEED);

    float stepsPerMm = motorStepPerRevolution / (pulleyToothCount * beltPitchMm); // GT2 belt has 2mm tooth pitch
    stepper.setStepsPerMillimeter(stepsPerMm);
    // initialize the speed and acceleration rates for the stepper motor. These
    // will be overwritten by user controls. 100 values are placeholders
    stepper.setSpeedInStepsPerSecond(200);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(100);
    stepper.setDecelerationInStepsPerSecondPerSecond(100000);
    stepper.setLimitSwitchActive(0);

    // Start the stepper instance as a service in the "background" as a separate
    // task and the OS of the ESP will take care of invoking the processMovement()
    // task regularly on core 1 so you can do whatever you want on core 0
    stepper.startAsService(); // Kinky Makers - we have modified this function
                              // from default library to run on core 1 and suggest
                              // you don't run anything else on that core.
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);

    if (g_has_not_homed == true)
    {
        LogDebug("OSSM will now home");
        stepper.setSpeedInMillimetersPerSecond(20);
        stepper.moveToHomeInMillimeters(1, 30, 300, LIMIT_SWITCH_PIN);
        LogDebug("OSSM has homed, will now move out to max length");
        stepper.setSpeedInMillimetersPerSecond(20);
        stepper.moveToPositionInMillimeters((-1 * maxStrokeLengthMm) - strokeZeroOffsetmm);
        LogDebug("OSSM has moved out, will now set new home?");
        stepper.setCurrentPositionAsHomeAndStop();
        LogDebug("OSSM should now be home and happy");
        g_has_not_homed = false;
    }

    //start the BLE connection after homing for clean homing when reconnecting
    xTaskCreatePinnedToCore(blemotionTask,      /* Task function. */
                            "blemotionTask",    /* name of task. */
                            8000,               /* Stack size of task */
                            NULL,               /* parameter of the task */
                            1,                  /* priority of the task */
                            &blemTask,          /* Task handle to keep track of created task */
                            0);                 /* pin task to core 0 */
    vTaskSuspend(blemTask);                     //Suspend Task after Creation for free CPU & RAM
    delay(100);
    
    xTaskCreatePinnedToCore(bleConnectionTask,   /* Task function. */
                            "bleConnectionTask", /* name of task. */
                            8000,                /* Stack size of task */
                            NULL,                 /* parameter of the task */
                            1,                    /* priority of the task */
                            &bleTask,            /* Task handle to keep track of created task */
                            0);                   /* pin task to core 0 */
    delay(100);

}

///////////////////////////////////////////
////
////
////   VOID LOOP - Hides here
////
////
///////////////////////////////////////////

void loop()
{
  delay(200);
}

///////////////////////////////////////////
////
////
////  freeRTOS multitasking
////
////
///////////////////////////////////////////

void bleConnectionTask(void *pvParameters){
  UBaseType_t uxHighWaterMark;

  LogDebug("Initializing BLE Server...");
  BLEDevice::init("OSSM");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  infoService = pServer->createService(BLEUUID((uint16_t) 0x180a));
  BLE2904* softwareVersionDescriptor = new BLE2904();
  softwareVersionDescriptor->setFormat(BLE2904::FORMAT_UINT8);
  softwareVersionDescriptor->setNamespace(1);
  softwareVersionDescriptor->setUnit(0x27ad);

  softwareVersionCharacteristic = infoService->createCharacteristic((uint16_t) 0x2a28, BLECharacteristic::PROPERTY_READ);
  softwareVersionCharacteristic->addDescriptor(softwareVersionDescriptor);
  softwareVersionCharacteristic->addDescriptor(new BLE2902());
  softwareVersionCharacteristic->setValue(FIRMWARE_VERSION);
  infoService->start();
  
  pService = pServer->createService(SERVICE_UUID);
  controlCharacteristic = pService->createCharacteristic(
                                         CONTROL_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  controlCharacteristic->addDescriptor(new BLE2902());
  controlCharacteristic->setValue("");
  controlCharacteristic->setCallbacks(new ControlCharacteristicCallback());
  
  settingsCharacteristic = pService->createCharacteristic(
                                         SETTINGS_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  settingsCharacteristic->addDescriptor(new BLE2902());
  settingsCharacteristic->setValue("");
  settingsCharacteristic->setCallbacks(new SettingsCharacteristicCallback());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  updateSettingsCharacteristic();
  // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  // LogDebugFormatted("Ble Free Stack size %ld \n", static_cast<long int>(uxHighWaterMark));
  vTaskDelete(NULL);
}

// BLE Motion task reads the Command list and starts the processing Cycle
void blemotionTask(void *pvParameters)
{
  UBaseType_t uxHighWaterMark;
  float target = 0.0; 
    for (;;) // tasks should loop forever and not return - or will throw error in
             // OS
    {

        while ( abs(stepper.getDistanceToTargetSigned()) > (target * 0.10) )
        {
            vTaskDelay(5); // wait for motion to complete
        }
        
        stepper.setDecelerationInMillimetersPerSecondPerSecond(xtoyDeaccelartion);
        vTaskDelay(1);
        if (pendingCommands.size() > 0) { 
        std::string command = pendingCommands.front();
        processCommand(command);
        target = abs(stepper.getDistanceToTargetSigned());
        pendingCommands.pop_front();
        }  
        vTaskDelay(1);
        //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        //LogDebugFormatted("Blemotion HighMark in %ld \n", static_cast<long int>(uxHighWaterMark));
    }    
}

void setLedRainbow(CRGB leds[])
{
    // int power = 250;

    for (int hueShift = 0; hueShift < 350; hueShift++)
    {
        int gHue = hueShift % 255;
        fill_rainbow(leds, NUM_LEDS, gHue, 25);
        FastLED.show();
        delay(4);
    }
}
