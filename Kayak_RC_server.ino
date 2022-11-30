/*
  Kayak_RC_server.ino    (WAS SimpleEspNowConnectionServer)

 */

#include <SPI.h>
#include "SimpleEspNowConnection.h"
#include "shared_yak_messaging.h"
#include "debug_serial_print.h"
#include "MCP41xxx.h"
#include <Adafruit_NeoPixel.h>



// Default SPI Config
//   MOSI: 18
//   MISO: 19
//   SCK: 5
//   SS: 33
//
// DEFAULT I2 CONFIG:
//   SDA: 23
//   SCL: 22


#define CLIENT_ADDRESS      "7CDFA194CA86"
#define RELAY_FORWARD_PIN   14
#define RELAY_REVERSE_PIN   32
#define BOARD_LED           13
#define NEOPIXEL_PIN        27

// setting PWM properties
const int PWMfreq       = 20000;
const int PWMchannel    = 0;
const int PWMresolution = 8;


typedef struct {
    uint32_t color0;
    uint32_t color1;
    uint32_t durationMillis;
    uint8_t flashes;
    uint8_t current_color;    // always 0 (color0) or 1 (color1)
    uint32_t nextChangeMillis;
} PixelFlashEvent_t;

PixelFlashEvent_t pixelAction;
#define PIXEL_TRIG_NOW 1
// Neopixel colors
#define NeoBlack    0x000000
#define NeoWhite    0xFFFFFF
#define NeoRed      0xFF0000
#define NeoGreen    0x00FF00
#define NeoDimGreen 0x000F00
#define NeoMinGreen 0x000100
#define NeoBlue     0x0000FF
#define NeoYellow   0xFFFF00
#define NeoMagenta  0xFF00FF
#define NeoPurple   0x800080
#define NeoDarkPurple 0x2c0222
#define NeoOrange   0xFF8C00
#define NeoLime     0x00FF00
#define NeoNavyBlue 0x000080
#define NeoGray     0x696969
#define NeoSilver   0xC0C0C0
#define NeoBrown    0x8B4513

void flash_pixel(uint32_t color0, uint8_t flashes=1, uint32_t duration=500, uint32_t color1=NeoBlack);



// DEFAULTS:
// #define RPWM 3 // define pin 3 for RPWM pin (output)
// #define R_EN 4 // define pin 2 for R_EN pin (input)
// #define R_IS 5 // define pin 5 for R_IS pin (output)
//
// #define LPWM 6 // define pin 6 for LPWM pin (output)
// #define L_EN 7 // define pin 7 for L_EN pin (input)
// #define L_IS 8 // define pin 8 for L_IS pin (output)
// #define CW 1 //do not change
// #define CCW 0 //do not change
// #define debug 1 //change to 0 to hide serial monitor debugging infornmation or set to 1 to view

#define FWD_PWM_PIN     21 // (FWD) define pin 3 for RPWM pin (input)   ESP --> Motor
#define FWD_EN_PIN      15 // CHECK WIRE 17 // (FWD) define pin 2 for R_EN pin (input)   ESP --> Motor
#define FWD_ALARM_PIN   39 // (FWD) define pin 5 for R_IS pin (output)  OPT

#define REV_PWM_PIN     12 // (REV) define pin 6 for LPWM pin (input)   ESP --> Motor
#define REV_EN_PIN      13 // (REV) define pin 7 for L_EN pin (input)   ESP --> Motor
#define REV_ALARM_PIN   36 // CHECK WIRE A5-IO4, Reconcile (REV) define pin 8 for L_IS pin (output)  OPT

/* OUT... was Robojax Crap
#define CW 1 //do not change
#define CCW 0 //do not change
#define debug 1 //change to 0 to hide serial monitor debugging infornmation or set to 1 to view

uint8_t motorDirection=CW;  // must only be CW  (Forward?) or CCW (Reverse?)
*/

GEAR_t lastGear=NEUTRAL;
uint8_t lastSpeed=0;
uint8_t lastPWMdutyCycle=0;
static uint32_t tick50=0; // , tick128=0, tick10000=0, tick30000=0;


SimpleEspNowConnection simpleEspConnection(SimpleEspNowRole::SERVER);
Adafruit_NeoPixel pixel(1 /*NUMPIXELS*/, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

//MCP41xxx(/*int CS*/ SS, /*int CLK*/ SCK, /*int MOSI*/ MOSI);

/* OUT... was Robojax Crap
 * RobojaxBTS7960 motor(FWD_EN_PIN, FWD_PWM_PIN, FWD_ALARM_PIN, REV_EN_PIN, REV_PWM_PIN, REV_ALARM_PIN, debug);
*/

//MCP41xxx* pot;



// Notes:
//    For Debug, THIS the initial WeMos D1 Mini Pro SERVER is AC:0B:FB:DD:41:00   (This code)
//    Client: Adafruit Feather ES:32-S2 TFT  mac address is 7C:DF:A1:95:0C:6E

yakMessage_t yakMessage = { YAK,        // msgType
                            0,          // msgID  <future> to track messages lost or ensure/check ordering
                            PING,       // action
                            NEUTRAL,    // gear
                            0           // speed (0-100)
                          };   // A default message


uint32_t    serverMsgID=0;

typedef struct struct_message {
  char type;
  char a[32];
  int b;
  float c;
  bool e;
}
struct_message;

String inputString;
String clientAddress = CLIENT_ADDRESS;

bool sendBigMessage() {
    char bigMessage[] = "\nThere was once a woman\n";

    return (simpleEspConnection.sendMessage(bigMessage, clientAddress));
}

bool sendStructMessage() {
    struct_message myData;

    myData.type = '#'; // just to mark first byte. It's on you how to distinguish between struct and text message
    sprintf(myData.a, "Greetings from %s", simpleEspConnection.myAddress.c_str());
    myData.b = random(1, 20);
    myData.c = (float) random(1, 100000) / (float) 10000;
    myData.e = false;

    return (simpleEspConnection.sendMessage((uint8_t * ) & myData, sizeof(myData), clientAddress));
}

void OnSendError(uint8_t * ad) {
    debug_pln("SENDING TO '" + simpleEspConnection.macToStr(ad) + "' WAS NOT POSSIBLE!");
    flash_pixel(NeoOrange, 3, 200);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)
}

void OnMessage(uint8_t * ad, const uint8_t * message, size_t len)
{

    if ((MSG_t) message[0] == YAK) {    // cast to pointer to this message type?
        yakMessage_t yakMessage;
        memcpy( &yakMessage, message, len);
        debug_printf("yakMessage: msgType (val):%X. msgID:%d, action:%s, gear:%s, speed:%d\n",
                      yakMessage.msgType, yakMessage.msgID, ACTION_t_v[yakMessage.action],
                      GEAR_t_v[yakMessage.gear], yakMessage.speed);

        if (yakMessage.action == MOTOR_CONTROL) {
            if (lastGear != yakMessage.gear) {
                setDirection(yakMessage.gear);
                debug_printf("\nSHIFTED!\n");

                //debug_printf("\nNow lastSpeed: %d, yakMessage.speed: %d  (Will we get there?)\n", lastSpeed, yakMessage.speed);
            }

            if (lastSpeed != yakMessage.speed) {
                if ( yakMessage.speed < 4 ) {
                    yakMessage.speed = 0;
                } else if ( yakMessage.speed > 97 ) {
                    yakMessage.speed = 100;
                }

                BTS7960SpeedRampTo(yakMessage.speed);
                lastSpeed = yakMessage.speed;
            }
        }
    } else {
        Serial.printf("MESSAGE:[%d]%s from %s\n", len, (char * ) message, simpleEspConnection.macToStr(ad).c_str());
    }
}

void OnPaired(uint8_t * ga, String ad) {
    debug_pln("EspNowConnection : Client '" + ad + "' paired! ");
    simpleEspConnection.endPairing();

    flash_pixel(NeoPurple, 1, 400);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)
    clientAddress = ad;
}


void OnConnected(uint8_t * ga, String ad) {
    //debug_pln("EspNowConnection : Client '" + ad + "' connected! ");

    yakCommsMessage_t commsMsg = { COMMS,          // MSG_t msgType
                                   serverMsgID++,  // uint32_t msgID
                                   CONNECTED,      // YAK_COMMS_t commState
                                   0,              // uint32_t rssi
                                 };

    simpleEspConnection.sendMessage((uint8_t * ) &commsMsg, sizeof(yakCommsMessage_t));

   flash_pixel(NeoGreen, 3, 200);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)

}

/*
        yakMessage: msgType (val):A1. msgID:98, action:MOTOR_CONTROL, gear:NEUTRAL, speed:46

        typedef enum gear { NEUTRAL, FORWARD, REVERSE } GEAR_t;
        typedef enum { YAK=0xA1, COMMS=0xB1 } MSG_t;
        typedef enum { PING, MOTOR_CONTROL, BATTERY_CHECK, BATTERY_MESSAGING, NEW_CLIENT_MAC, BLUETOOTH_PAIR } ACTION_t;
 */


void BTS7960Setup()
{
    // configure PWM functionality
    ledcSetup(PWMchannel, PWMfreq, PWMresolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(FWD_PWM_PIN, PWMchannel);
    ledcAttachPin(REV_PWM_PIN, PWMchannel);
}


void BTS7960Gear(GEAR_t gear)
{
    switch(gear) {
    case FORWARD:
        digitalWrite(REV_EN_PIN, LOW);
        digitalWrite(FWD_EN_PIN, HIGH);
        break;
    case REVERSE:
        digitalWrite(FWD_EN_PIN, LOW);
        digitalWrite(REV_EN_PIN, HIGH);
        break;
    case NEUTRAL:
        digitalWrite(FWD_EN_PIN, LOW);
        digitalWrite(REV_EN_PIN, LOW);
        break;
    // Future   case BRAKE:
    // Future       digitalWrite(FWD_EN_PIN, LOW);
    // Future       digitalWrite(REV_EN_PIN, LOW);
    // Future       break;
    }
}

void BTS7960Speed(uint8_t dutyCycle)
{
    ledcWrite(PWMchannel, dutyCycle);
    lastPWMdutyCycle = dutyCycle;
}

void BTS7960SpeedRampTo(uint8_t request)
{
    byte dutyCycle = map(request, 0, 100, 0, 0xFF);

    if (lastPWMdutyCycle > dutyCycle) {
        debug_p("Ramp DOWN: ");
        for (uint8_t x=lastPWMdutyCycle; x > dutyCycle; x--){
            //debug_printf("%d ", x);
            debug_p(String(x) + " ");
            ledcWrite(PWMchannel, x);
            //delay(10);
        }
    } else if (lastPWMdutyCycle < dutyCycle) {
        debug_p("Ramp UP: ");
        for (uint8_t x=lastPWMdutyCycle; x < dutyCycle; x++){
            //debug_printf("%d ", x);
            debug_p(String(x) + " ");
            ledcWrite(PWMchannel, x);
            //delay(10);
        }
    }
    debug_pln("");
    lastPWMdutyCycle = dutyCycle;
}


void setDirection(GEAR_t gear)
{
     if (gear != lastGear) {
        // Ramp down speed
        debug_p("Gear change ramp: ");
        BTS7960SpeedRampTo(0);
        lastSpeed = 0;
        BTS7960Gear(NEUTRAL);
        lastGear = NEUTRAL;
        debug_printf(" -> NEUTRAL%s", gear==NEUTRAL?"\n" :"");
     }

    switch(gear) {
    case FORWARD:
        debug_pln(" -> FORWARD");
        BTS7960Gear(FORWARD);
        //digitalWrite(RELAY_REVERSE_PIN, LOW);
        //delay(50);
        //digitalWrite(RELAY_FORWARD_PIN, HIGH);
        //delay(50);
        break;

    case REVERSE:
        debug_pln(" -> REVERSE");
        BTS7960Gear(FORWARD);
        //digitalWrite(RELAY_FORWARD_PIN, LOW);
        //delay(50);
        //digitalWrite(RELAY_REVERSE_PIN, HIGH);
        //delay(50);
        break;
    }
}

void flash_pixel(uint32_t color0, uint8_t flashes, uint32_t duration, uint32_t color1)
{
    pixelAction = {color0, color1, duration, flashes, NeoBlack, PIXEL_TRIG_NOW};
}


//    // Define the MCP41100 OP command bits (only one POT)
//    // Note: command byte format xxCCxxPP, CC command, PP pot number (01 if selected)
//    //                   xxCCxxPP
//    #define MCP_NOP    0b00000000
//    #define MCP_WRITE  0b00010001
//    #define MCP_SHTDWN 0b00100001
//
//    #define WAIT_DELAY 1000
//
//    void SPIWrite(uint8_t cmd, uint8_t data, uint8_t ssPin)
//    // SPI write the command and data to the MCP IC connected to the ssPin
//    {
//      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
//      digitalWrite(ssPin, LOW); // SS pin low to select chip
//      SPI.transfer(cmd);        // Send command code
//      SPI.transfer(data);       // Send associated value
//      digitalWrite(ssPin, HIGH);// SS pin high to de-select chip
//      SPI.endTransaction();
//    }
//
//    void setup()
//    {
//      pinMode (SS, OUTPUT);
//      digitalWrite(SS, HIGH);
//      SPI.begin();
//
//      Serial.begin(57600);
//      debug_pln(F("[MCP Digital Pot Test]"));
//    }
//
//    void loop()
//    {
//      // step through the range of the digital pot
//      for (int i = 0; i < 256; i++)
//      {
//        debug_pln(i);
//        SPIWrite(MCP_WRITE, i, SS);
//        delay(WAIT_DELAY);
//      }
//      delay (WAIT_DELAY*5);
//    }



void setup()
{
    Serial.begin(115200);

    pinMode(BOARD_LED, OUTPUT);
    uint8_t lastBlink=3;
    for (uint8_t blinks=0; blinks < lastBlink;) {
        digitalWrite(BOARD_LED, HIGH);
        delay(200);
        digitalWrite(BOARD_LED, LOW);
        if (++blinks < lastBlink) {
            delay(200);
        }
    }

    debug_pln("\n");
    debug_pln("Kayak Motor Control (server) startup: Built " + String(__DATE__) + ", " + String(__TIME__));
    //delay(1600);  // 400 required for ESP8266 "D1 Mini Pro"
    //snprintf(tft_lin_buf, sizeof(tft_lin_buf), "Built %s @ %s", __DATE__, __TIME__);

    simpleEspConnection.begin();
    //   simpleEspConnection.setPairingBlinkPort(2);
    simpleEspConnection.onMessage( & OnMessage);
    simpleEspConnection.onPaired( & OnPaired);
    simpleEspConnection.onSendError( & OnSendError);
    simpleEspConnection.onConnected( & OnConnected);

    debug_pln("I'm the server. My MAC address is " + WiFi.macAddress());

    //debug_pln("Instantiating the potentiometer");
    //
    //pinMode (SS, OUTPUT);  // ToDo: Possibly redundant ??
    //SPI.begin();            // ToDo: Possibly redundant ??
    //
    //pot = new MCP41xxx(SS, SCK, MOSI);    //     MCP41xxx(int CS, int CLK, int MOSI);
    //
    //debug_pln("It's usable, set to zero!");
    //pot->setPot(0);
    //
    //pinMode(BOARD_LED, OUTPUT);
    //digitalWrite(BOARD_LED, HIGH);
    //
    //digitalWrite(RELAY_FORWARD_PIN, LOW);
    //pinMode(RELAY_FORWARD_PIN, OUTPUT);
    //
    //digitalWrite(RELAY_REVERSE_PIN, LOW);
    //pinMode(RELAY_REVERSE_PIN, OUTPUT);

    /* OUT... was Robojax Crap
    motor.begin();
    */
    BTS7960Setup();

    // NOT necessary on THIS ESP32:  pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, 1);
    pixel.begin();
}



void task50ms(void)
{
    /* reference only
        typedef struct {
            uint32_t color0;
            uint32_t color1;
            uint32_t durationMillis;
            uint8_t flashes;
            uint8_t current_color;    // always 0 (color0) or 1 (color1)
            uint32_t nextChangeMillis;
        } PixelFlashEvent_t;

    */
    //static PixelFlashEvent_t last_pe={NeoBlack, NeoBlack, 0, 0, NeoBlack, PIXEL_TRIG_NOW};
    //static bool didAction=false;
    uint32_t current_millis = millis();

    //if (memcmp((const void *) &last_pe, (const void *) &pixelAction, sizeof(PixelFlashEvent_t)) != 0) {
    //    dump_pixelAction("task50ms new pe: ", pixelAction);
    //    last_pe = pixelAction;
    //}

    // can also pixel.setBrightness(50);  (before .show)

    if (pixelAction.nextChangeMillis > 0 && pixelAction.nextChangeMillis <= current_millis  ) {
        if (pixelAction.current_color == 0) {
            pixel.setPixelColor(0, pixelAction.color0);
            pixel.show();
            //debug_p("Sent color0 (");
            //debug_p(pixelAction.color0);
            //debug_pln(") to pixel");
            pixelAction.nextChangeMillis = millis() + (pixelAction.durationMillis / 2);
            pixelAction.current_color = 1;

        } else if (pixelAction.current_color == 1) {
            pixel.setPixelColor(0, pixelAction.color1);
            pixel.show();
            //debug_p("Sent color1 (");
            //debug_p(pixelAction.color1);
            //debug_pln(") to pixel");
            pixelAction.nextChangeMillis = millis() + (pixelAction.durationMillis / 2);
            pixelAction.current_color = 0 ;
            // pixelAction.flashes = pixelAction.flashes - 1;
            pixelAction.flashes -= 1;

        }
    }

    if (pixelAction.flashes == 0) {
        pixelAction.nextChangeMillis = 0;
    }
}


void loop() {
    uint32_t sysTick = millis();

    // needed to manage the communication in the background!
    simpleEspConnection.loop();

    if (sysTick - tick50 > 50) {
        tick50 = sysTick;
        task50ms();
    }

    while (Serial.available()) {
        char inChar = (char) Serial.read();
        if (inChar == '\n') {
          debug_pln(inputString);

          if (inputString == "startpair") {
            simpleEspConnection.startPairing(30);
          } else if (inputString == "endpair") {
            simpleEspConnection.endPairing();
          } else if (inputString == "changepairingmac") {
            uint8_t np[] {
              0xCE,
              0x50,
              0xE3,
              0x15,
              0xB7,
              0x33
            };

            simpleEspConnection.setPairingMac(np);
          } else if (inputString == "textsend") {
            if (!simpleEspConnection.sendMessage("This comes from the server", clientAddress)) {
              debug_pln("SENDING TO '" + clientAddress + "' WAS NOT POSSIBLE!");
            }
          } else if (inputString == "structsend") {
            if (!sendStructMessage()) {
              debug_pln("SENDING TO '" + clientAddress + "' WAS NOT POSSIBLE!");
            }
          } else if (inputString == "bigsend") {
            if (!sendBigMessage()) {
              debug_pln("SENDING TO '" + clientAddress + "' WAS NOT POSSIBLE!");
            }
          }

          inputString = "";
        } else {
          inputString += inChar;
        }
    }
}