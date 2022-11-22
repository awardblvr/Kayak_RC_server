/*
  pre_Kayak_RC_server.ino    (WAS SimpleEspNowConnectionServer)

 */

#include "SimpleEspNowConnection.h"
#include "shared_yak_messaging.h"
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




SimpleEspNowConnection simpleEspConnection(SimpleEspNowRole::SERVER);
MCP41xxx(/*int CS*/ SS, /*int CLK*/ SCK, /*int MOSI*/ MOSI);



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
String clientAddress;

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
  Serial.println("SENDING TO '" + simpleEspConnection.macToStr(ad) + "' WAS NOT POSSIBLE!");
}

void OnMessage(uint8_t * ad, const uint8_t * message, size_t len)
{

    if ((MSG_t) message[0] == YAK) {    // cast to pointer to this message type?
        yakMessage_t yakMessage;
        memcpy( &yakMessage, message, len);
        Serial.printf("yakMessage: msgType (val):%X. msgID:%d, action:%s, gear:%s, speed:%d\n",
                      yakMessage.msgType, yakMessage.msgID, ACTION_t_v[yakMessage.action],
                      GEAR_t_v[yakMessage.gear], yakMessage.speed);
        // Serial.printf("  msgType (val):%X\n", yakMessage.msgType);
        // Serial.printf("  msgID:%d\n", yakMessage.msgID);
        // Serial.printf("  action:%s\n", ACTION_t_v[yakMessage.action]);
        // Serial.printf("  gear:%s\n", GEAR_t_v[yakMessage.gear]);
        // Serial.printf("  speed:%d\n", yakMessage.speed);
  } else
    Serial.printf("MESSAGE:[%d]%s from %s\n", len, (char * ) message, simpleEspConnection.macToStr(ad).c_str());
}

void OnPaired(uint8_t * ga, String ad) {
  Serial.println("EspNowConnection : Client '" + ad + "' paired! ");
  simpleEspConnection.endPairing();

  clientAddress = ad;
}


void OnConnected(uint8_t * ga, String ad) {
    Serial.println("EspNowConnection : Client '" + ad + "' connected! ");

    yakCommsMessage_t commsMsg = { COMMS,          // MSG_t msgType
                                   serverMsgID++,  // uint32_t msgID
                                   CONNECTED,      // YAK_COMMS_t commState
                                   0,              // uint32_t rssi
                                 };

    simpleEspConnection.sendMessage((uint8_t * ) &commsMsg, sizeof(yakCommsMessage_t));
}

/*
        yakMessage: msgType (val):A1. msgID:92, action:MOTOR_CONTROL, gear:FORWARD, speed:0
        yakMessage: msgType (val):A1. msgID:93, action:MOTOR_CONTROL, gear:NEUTRAL, speed:0
        yakMessage: msgType (val):A1. msgID:94, action:MOTOR_CONTROL, gear:REVERSE, speed:0
        yakMessage: msgType (val):A1. msgID:95, action:MOTOR_CONTROL, gear:NEUTRAL, speed:0
        yakMessage: msgType (val):A1. msgID:96, action:MOTOR_CONTROL, gear:NEUTRAL, speed:25
        yakMessage: msgType (val):A1. msgID:97, action:MOTOR_CONTROL, gear:NEUTRAL, speed:44
        yakMessage: msgType (val):A1. msgID:98, action:MOTOR_CONTROL, gear:NEUTRAL, speed:46

        typedef enum gear { NEUTRAL, FORWARD, REVERSE } GEAR_t;
        typedef enum { YAK=0xA1, COMMS=0xB1 } MSG_t;
        typedef enum { PING, MOTOR_CONTROL, BATTERY_CHECK, BATTERY_MESSAGING, NEW_CLIENT_MAC, BLUETOOTH_PAIR } ACTION_t;
 */


void setDirection(GEAR_t gear)
{

}

void setSpeed(uint8_t speed)
{

}

void setup() {
  Serial.begin(115200);
  Serial.println("\n");
  delay(1600);  // 400 required for ESP8266 "D1 Mini Pro"

  Serial.print("Server Setup: Server address: ");    // code below pumps out ~"AC:0B:FB:DD:41:00"

  //clientAddress = "CC50E35B56B1"; // Test if you know the client
  //clientAddress = "AC0BFBDCE1F1"; // Address discovered by manual pairing
  //clientAddress = "7CDFA1950C6E"; // old Adafruit ESP32-S2 TFT
  clientAddress = "7CDFA194CA86";   // New Adafruit ESP32-S2 TFT


      Serial.println("DEFAULT SPI CONFIG:");
      Serial.print("  MOSI: ");
      Serial.println(MOSI);
      Serial.print("  MISO: ");
      Serial.println(MISO);
      Serial.print("  SCK: ");
      Serial.println(SCK);
      Serial.print("  SS: ");
      Serial.println(SS);

      Serial.println("\nDEFAULT I2C CONFIG:");
      Serial.print("  SDA: ");
      Serial.println(SDA);
      Serial.print("  SCL: ");
      Serial.println(SCL);








  simpleEspConnection.begin();
  //   simpleEspConnection.setPairingBlinkPort(2);
  simpleEspConnection.onMessage( & OnMessage);
  simpleEspConnection.onPaired( & OnPaired);
  simpleEspConnection.onSendError( & OnSendError);
  simpleEspConnection.onConnected( & OnConnected);

  Serial.println("I'm the server. My MAC address is " + WiFi.macAddress());
}

void loop() {
  // needed to manage the communication in the background!
  simpleEspConnection.loop();

  while (Serial.available()) {
    char inChar = (char) Serial.read();
    if (inChar == '\n') {
      Serial.println(inputString);

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
          Serial.println("SENDING TO '" + clientAddress + "' WAS NOT POSSIBLE!");
        }
      } else if (inputString == "structsend") {
        if (!sendStructMessage()) {
          Serial.println("SENDING TO '" + clientAddress + "' WAS NOT POSSIBLE!");
        }
      } else if (inputString == "bigsend") {
        if (!sendBigMessage()) {
          Serial.println("SENDING TO '" + clientAddress + "' WAS NOT POSSIBLE!");
        }
      }

      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}