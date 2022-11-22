/*
  pre_Kayak_RC_server.ino    (WAS SimpleEspNowConnectionServer)

 */

#include <SPI.h>
#include "SimpleEspNowConnection.h"
#include "shared_yak_messaging.h"
#include "debug_serial_print.h"
#include "MCP41xxx.h"


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


GEAR_t lastGear=NEUTRAL;
uint8_t lastSpeed=0;


SimpleEspNowConnection simpleEspConnection(SimpleEspNowRole::SERVER);
//MCP41xxx(/*int CS*/ SS, /*int CLK*/ SCK, /*int MOSI*/ MOSI);

MCP41xxx* pot;



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
}

void OnMessage(uint8_t * ad, const uint8_t * message, size_t len)
{

    if ((MSG_t) message[0] == YAK) {    // cast to pointer to this message type?
        yakMessage_t yakMessage;
        memcpy( &yakMessage, message, len);
        //debug_printf("yakMessage: msgType (val):%X. msgID:%d, action:%s, gear:%s, speed:%d\n",
        //              yakMessage.msgType, yakMessage.msgID, ACTION_t_v[yakMessage.action],
        //              GEAR_t_v[yakMessage.gear], yakMessage.speed);

        if (yakMessage.action == MOTOR_CONTROL) {
            if (lastGear != yakMessage.gear) {
                setDirection(yakMessage.gear);
                lastGear = yakMessage.gear;
            }

            if (lastSpeed > yakMessage.speed) {
                debug_p("Ramp DOWN: ");
                for (uint8_t x=lastSpeed; x > yakMessage.speed; x--){
                    debug_printf("%d ", x);
                    pot->setPot(x);
                    delay(10);
                }
                debug_pln("");
                lastSpeed = yakMessage.speed;
            }

            if (lastSpeed < yakMessage.speed) {
                debug_p("Ramp UP: ");
                for (uint8_t x=lastSpeed; x < yakMessage.speed; x++){
                    debug_printf("%d ", x);
                    pot->setPot(x);
                    delay(10);
                }
                debug_pln("");
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
}

/*
        yakMessage: msgType (val):A1. msgID:98, action:MOTOR_CONTROL, gear:NEUTRAL, speed:46

        typedef enum gear { NEUTRAL, FORWARD, REVERSE } GEAR_t;
        typedef enum { YAK=0xA1, COMMS=0xB1 } MSG_t;
        typedef enum { PING, MOTOR_CONTROL, BATTERY_CHECK, BATTERY_MESSAGING, NEW_CLIENT_MAC, BLUETOOTH_PAIR } ACTION_t;
 */


void motorToNeutral(void)
{
    digitalWrite(RELAY_FORWARD_PIN, LOW);
    delay(50);
    digitalWrite(RELAY_REVERSE_PIN, LOW);
    delay(50);
}


void setDirection(GEAR_t gear)
{
     if (gear != lastGear) {
        // Ramp down speed
        debug_p("Gear change ramp: ");
        for (uint8_t x=lastSpeed; x > 0; x--){
            debug_printf("%d ", x);
            pot->setPot(x);
            delay(10);
        }
        debug_p(" -> NEUTRAL");
        motorToNeutral();
        lastGear = NEUTRAL;
     }

    switch(gear) {
        case FORWARD:
            debug_pln(" -> FORWARD");
            digitalWrite(RELAY_REVERSE_PIN, LOW);
            delay(50);
            digitalWrite(RELAY_FORWARD_PIN, HIGH);
            delay(50);
            break;

        case REVERSE:
            debug_pln(" -> REVERSE");
            digitalWrite(RELAY_FORWARD_PIN, LOW);
            delay(50);
            digitalWrite(RELAY_REVERSE_PIN, HIGH);
            delay(50);
            break;
    }

}

void setSpeed(uint8_t speed)
{
    debug_printf("set Speed %d\n", speed);

    pot->setPot(speed);
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

    debug_pln("Instantiating the potentiometer");

    pinMode (SS, OUTPUT);  // ToDo: Possibly redundant ??
    SPI.begin();            // ToDo: Possibly redundant ??

    pot = new MCP41xxx(SS, SCK, MOSI);    //     MCP41xxx(int CS, int CLK, int MOSI);

    debug_pln("It's usable, set to zero!");
    pot->setPot(0);

    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, HIGH);


}

void loop() {
  // needed to manage the communication in the background!
  simpleEspConnection.loop();

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