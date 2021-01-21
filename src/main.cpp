#include <Arduino.h>
#include <Joystick.h>
#include <PluggableUSB.h>
#include <HID.h>
#include <Wire.h>

//button pins in order for reference
const int buttonPins[12] = {A2,1,4,5,6,7,8,9,10,11,12,13};

//construct a joystick with 16 buttons and an x-axis
Joystick_ Joystick(0x03, 0x04, 16, 0, true, false, false, false, false, false, false, false, false, false, false);

//defines
#define ENC_A 0 //needs to be on zero to use it as an interrupt pin
#define ENC_B A1

//init arrays for holding the states of each button
int currentState [] = {0,0,0,0,0,0,0,0,0,0,0,0};
int prevState [] =    {0,0,0,0,0,0,0,0,0,0,0,0};

//init variables for encoder; must be volatile to use them in ISR for encoder reads
volatile int encCount = 0;
volatile int aState;
volatile int bState;
volatile int aLastState;
volatile int lastEncCount = 0;

//globals
int lightState[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
bool infinitasMode = false;
int continuousCounts = 0;
bool hidLightMode = false;
bool needsUpdate = false;

//constants
const int TT_SENSITIVITY = 2;

//function prototypes
void requestEvent();
void encoderUpdate(); //ISR
bool anyButtonPressed();

/**
 * BEGIN HID LIGHTING STUFF
 */
typedef struct {
  uint8_t brightness;
} SingleLED;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} RGBLed;

#define NUMBER_OF_SINGLE 10
#define NUMBER_OF_RGB 0

void light_update(SingleLED* single_leds, RGBLed* rgb_leds) {
  if (hidLightMode) {
    for(int i = 0; i < NUMBER_OF_SINGLE; i++) {
        if (single_leds[i].brightness > 128)
          lightState[i] = 1;
        else
          lightState[i] = 0;
    }
    for(int i = 0; i < NUMBER_OF_RGB; i++) {
      //there are no RGBs
    }
    needsUpdate = true;
  }
}
/**
 * END HID LIGHTING STUFF
 */

void setup() {
  //initialize button pins
  for (int pin = 0; pin < 12; pin++) {
    pinMode(buttonPins[pin], INPUT_PULLUP);
  }

  //initialize encoder inputs
  pinMode(ENC_A, INPUT_PULLUP);  //has to be INPUT_PULLUP because the encoder is NPN sinking (does not generate high for a pulse, rather pulls up to ground)
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderUpdate, CHANGE); 
  pinMode(A1, INPUT_PULLUP);

  //start the joystick output and set the x axis range
  Joystick.begin();
  Joystick.setXAxisRange(0,1199);

  //priming read for encoder
  aLastState = !digitalRead(ENC_A);

  //check which mode to boot controller in
  if (!digitalRead(A2)) //if key 1 is held on startup, use joystick x axis for turntable instead of emulating buttons
    infinitasMode = true;

  if (!digitalRead(3)) //if key 2 is held on startup, use HID light data
    hidLightMode = true;

  while (anyButtonPressed()) {
    delay(1); //do nothing
  }

  Wire.begin(8); //start I2C as slave on channel 8
  Wire.setClock(400000);
  Wire.onRequest(requestEvent);
}

void loop() {
  /**
   * BUTTON LOGIC
   */  
  //getting states of buttons and writing to currentState
  for (int i = 0; i < 12; i++) {
    if (digitalRead(buttonPins[i]) == LOW) {
      currentState[i] = 1;
      if (!hidLightMode)
        lightState[i] = 1;
    }
    else {
      currentState[i] = 0;
      if (!hidLightMode)
        lightState[i] = 0;
    }
  }

  //writing those states to joystick if pressed (while ignoring the mode button)
  for (int i = 0; i < 11; i++) {
    if (currentState[i])
      Joystick.setButton(i,1);
    else
      Joystick.setButton(i,0);
  }

  //finally, set prevState = currentState
  for (int i = 0; i < 12; i++) {
    prevState[i] = currentState[i];
  }

  /**
   * TURNTABLE LOGIC
   */ 
  if (infinitasMode) {  //if in infinitas (turntable as joystick axis mode) 
    //bound the encoder count from 0-1199 so it wraps around if it exceeds either value
    if (encCount > 1199)
      encCount = 0;
    if (encCount < 0)
      encCount = 1199;

    //send the output to the Joystick's x-axis  
    Joystick.setXAxis(encCount);
  }
  else {  //if in button mode (turntable as joystick buttons mode)
    if (encCount != lastEncCount) {
      if (encCount > lastEncCount) {
        if (continuousCounts < 0)
          continuousCounts = 0;
        continuousCounts++;
      }
      if (encCount < lastEncCount) {
        if (continuousCounts > 0)
          continuousCounts = 0;
        continuousCounts--;
      }
    }
    else {
      continuousCounts = 0;
    }

    if (continuousCounts > TT_SENSITIVITY) {
      Joystick.setButton(12,1);
    }
    else if (continuousCounts < -TT_SENSITIVITY) {
      Joystick.setButton(13,1);
    }
    else {
      Joystick.setButton(12,0);
      Joystick.setButton(13,0);
    }
    lastEncCount = encCount;
  }
}

void requestEvent() {
  if (hidLightMode) {
    if (needsUpdate) {
      for (int i = 0; i < 12; i++) {
        Wire.write(lightState[i]);
      }
      Wire.write(aState);
      Wire.write(bState);
    }
  }
  else {
    for (int i = 0; i < 12; i++) {
        Wire.write(lightState[i]);
      }
      Wire.write(aState);
      Wire.write(bState);
  }
}

void encoderUpdate() {
  //read the state of the encoder's a output
  aState = !digitalRead(ENC_A); //because it's INPUT_PULLUP, a pulse occurs when ENC_A goes LOW
  bState = !digitalRead(ENC_B);

  //if the current state of the a output is different from the previous state, a pulse has occurred
  if (aState != aLastState) {
    if (bState != aState) {
      encCount++;
    }
    else {
      encCount--;
    }
  }

  //set the current a state as the previous a state
  aLastState = aState;
}

bool anyButtonPressed() {
  for (int i = 0; i < 12; i++) {
    if (!digitalRead(buttonPins[i]))
      return true;
  }
  return false;
}


/**
 * HID LIGHTING THINGS
 */
#define NUMBER_OF_LIGHTS (NUMBER_OF_SINGLE + NUMBER_OF_RGB*3)
#if NUMBER_OF_LIGHTS > 63
  #error You must have less than 64 lights
#endif

union {
  struct {
    SingleLED singles[NUMBER_OF_SINGLE];
    RGBLed rgb[NUMBER_OF_RGB];
  } leds;
  uint8_t raw[NUMBER_OF_LIGHTS];
} led_data;

static const uint8_t PROGMEM _hidReportLEDs[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x00,                    // USAGE (Undefined)
    0xa1, 0x01,                    // COLLECTION (Application)
    // Globals
    0x95, NUMBER_OF_LIGHTS,        //   REPORT_COUNT
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x05, 0x0a,                    //   USAGE_PAGE (Ordinals)
    // Locals
    0x19, 0x01,                    //   USAGE_MINIMUM (Instance 1)
    0x29, NUMBER_OF_LIGHTS,        //   USAGE_MAXIMUM (Instance n)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    // BTools needs at least 1 input to work properly
    0x19, 0x01,                    //   USAGE_MINIMUM (Instance 1)
    0x29, 0x01,                    //   USAGE_MAXIMUM (Instance 1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0xc0                           // END_COLLECTION
};

// This is almost entirely copied from NicoHood's wonderful RawHID example
// Trimmed to the bare minimum
// https://github.com/NicoHood/HID/blob/master/src/SingleReport/RawHID.cpp
class HIDLED_ : public PluggableUSBModule {

  uint8_t epType[1];

  public:
    HIDLED_(void) : PluggableUSBModule(1, 1, epType) {
      epType[0] = EP_TYPE_INTERRUPT_IN;
      PluggableUSB().plug(this);
    }

    int getInterface(uint8_t* interfaceCount) {
      *interfaceCount += 1; // uses 1
      HIDDescriptor hidInterface = {
        D_INTERFACE(pluggedInterface, 1, USB_DEVICE_CLASS_HUMAN_INTERFACE, HID_SUBCLASS_NONE, HID_PROTOCOL_NONE),
        D_HIDREPORT(sizeof(_hidReportLEDs)),
        D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint), USB_ENDPOINT_TYPE_INTERRUPT, USB_EP_SIZE, 16)
      };
      return USB_SendControl(0, &hidInterface, sizeof(hidInterface));
    }

    int getDescriptor(USBSetup& setup)
    {
      // Check if this is a HID Class Descriptor request
      if (setup.bmRequestType != REQUEST_DEVICETOHOST_STANDARD_INTERFACE) { return 0; }
      if (setup.wValueH != HID_REPORT_DESCRIPTOR_TYPE) { return 0; }

      // In a HID Class Descriptor wIndex contains the interface number
      if (setup.wIndex != pluggedInterface) { return 0; }

      return USB_SendControl(TRANSFER_PGM, _hidReportLEDs, sizeof(_hidReportLEDs));
    }

    bool setup(USBSetup& setup)
    {
      if (pluggedInterface != setup.wIndex) {
        return false;
      }

      uint8_t request = setup.bRequest;
      uint8_t requestType = setup.bmRequestType;

      if (requestType == REQUEST_DEVICETOHOST_CLASS_INTERFACE)
      {
        return true;
      }

      if (requestType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE) {
        if (request == HID_SET_REPORT) {
          if(setup.wValueH == HID_REPORT_TYPE_OUTPUT && setup.wLength == NUMBER_OF_LIGHTS){
            USB_RecvControl(led_data.raw, NUMBER_OF_LIGHTS);
            light_update(led_data.leds.singles, led_data.leds.rgb);
            return true;
          }
        }
      }

      return false;
    }
};

HIDLED_ HIDLeds;