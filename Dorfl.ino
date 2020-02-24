/*
 * Firmware for WiFi Curtain/Blinds Switch module based on ESP8285:  
 * https://s.click.aliexpress.com/e/_dXr335D
 * 
 * For more information on wiring, flashing, etc. see https://ibn.by/...
 * 
 * Works via MQTT over WiFi, as well as with directly attached to wall switch.
 * As a rich feature, it also delivers to the MQTT it's state (position of blinds, 
 * as well as whether it's blocked by switch-key).
 * 
 * Working modes:
 *   * LED blinks                      - module in setup mode. At this moment open WiFi hotspot is enabled.
 *                                       Once enabled, one can connect 'Dorfl-xxxxx' WiFi network and open
 *                                       configuration page on 192.168.4.1 address.
 *   * LED is on                       - module in fully operating mode. 
 *
 * Operation controls:
 *   * Lock at specific position       - when movement is activated with manual switch, quickly turn off switch and immediately
 *                                       press it again (delay must be less than 300ms)repeatedly
 *   * Enter setup mode                - quickly press some switch 5 times
 *
 * Configuration (setup mode):
 *   Following parameters can be found under 'Configure WiFi' menu item:
 *    * SSID/password - connection to WiFi
 *    * mqtt server      - MQTT server address (or host name)
 *    * mqtt port        - MQTT port
 *    * mqtt client name - just quess :-)
 *    * mqtt user
 *    * mqtt password
 *    * mqtt position output topic - topic for output of current curtain position. Value is always in range [0, MaxPos]. 
 *                                   Value is an integer with amount of seconds from completely open curtain. Usually it's written
 *                                   on relay open (after curtain movement is done), but when triggered with manual switch - also
 *                                   on trigger time. When triggered with manual switch, it also contains a dot in the end.
 *                                   Examples:
 *                                    '0' - curtain completely open
 *                                    '30' - curtain at position 30 (means was closing at least for 30 seconds. but could be it's a MaxPos)
 *                                    '30.' - curtain at position 30 and manual switch is pushed. Home-server can identify that it was triggered
 *                                            manually by human being.
 *    * mqtt commands topic         - topic for commands input.
 *    * blinds max pos              - MaxPos for position value. Integer with maximum seconds relays can be closed to 
 *                                    get to fully open or fully closed state. 
 *    * invert zero-position        - inverts relays for open/close direction. By default L1 is for opening, L2 for closing.
 *    * invert switch keys          - inverts manual switch logic. By default S1 is for opening, S2 for closing.
 *    * disable manual lock         - when set, the "Lock at specific position" is disabled. Experimental, to get rid of 
 *                                    potential problems when it interferes with normal work.
 * 
 * Supports following commands over MQTT /cmd topic:
 * * mvr<XXX> - move to relative position XXX. XXX can be negative. 
 *              If XXX negative - it moves curtain UP (closes L1 relay). If positive - DOWN (L2 relay)
 *              Absulute value of XXX - is amount of seconds to keep closed relay.
 *              Examples: 
 *                'mvr5' - close relay L2 for 5 seconds (closing curtains).
 *                'mvr-60' - close relay L1 for 60 seconds (opening curtains).
 *                'mvr0' - opens both relays. I.e. stop curtains.
 * * mva<XXX> - move to absolute position XXX. XXX can be negative or any big number. 
 *              If XXX is out of range of curtains [0, MaxPos], then after XXX seconds it will
 *              be set to 0 or MaxPos.
 *              Examples:
 *                'mva0' - completely open curtains.
 *                'mva60' - completely close curtains. Will be closing relay L2 for 60 seconds, and after 60 secs will set pos value to MaxPos
 *                'mva-60' - completely open curtains. Will be closing relay L1 for 60 seconds, and after 60 secs will set pos value to 0
 *  * set - enter setup mode. Analogue to pressing 'Pairing Button' on module. 
 *              Example:
 *                'set' - open WiFi hotspot 'Dorfl-xxxxx' will be enabled for 3 minutes. After 3 minutes will exit setup mode.
 * 
 *  Notes:
 *   Manual switch has following functionality:
 *    Once manual switch is pushed, any running remote command gets terminated, /pos topic get written current position with '.' suffix, 
 *    and appropriate relay gets closed. At this moment no remote control is possible.
 *    Once released - relay gets opened, /pos topic get written position without suffix. Remote control is possible now (though 
 *    any previous remote command is canceled).
 *    But if switch remains in pushed state forever, then after MaxPos seconds timeout, the relay gets automatically opened 
 *    and /pos topic get written position with '.' suffix, which indicates that manual switch blocks remote control. At this moment
 *    /pos topic may get written either '0.' value, or MaxPos value (like '30.').
 *    That allows to block curtains from any remote/automatic control by simply leaving 'open' or 'close' switch button in pushed state. 
 *    Or also allows to terminate automated remote command by quickly pushing/released any manual switch button.
 * 
 * Author: Anar Ibragimoff (anar@ibn.by)
 * 
 */
#include <FS.h>                 // this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino
#include <DNSServer.h>          // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>   // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>        // https://github.com/tzapu/WiFiManager/tree/development

#include <Ticker.h>

#include <WiFiUdp.h>
#include <mDNSResolver.h>       // https://github.com/madpilot/mDNSResolver

#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // https://github.com/knolleary/pubsubclient

// GPIOs
#define LED_BUILTIN 3
#define SWITCH_S1 4
#define SWITCH_S2 5
#define RELAY_L1 12
#define RELAY_L2 14
#define BUTTON 13

#define CONFIG_TIMEOUT_MS 300000

#define LOOP_DELAY_MS 10
#define MANUAL_LOCK_MIN_DELAY_MS 200
#define MANUAL_LOCK_MAX_DELAY_MS 500
#define MANUAL_SETUP_COUNTER 5

// commands
#define CMD_MOVE_ABS "mva"
#define CMD_MOVE_REL "mvr"
#define CMD_SETUP "set"


// --------------------------------------------------
#define CONFIG_FILE "/config.json"

#define CHECKBOX "true' type='checkbox' style='width: auto;"
#define CHECKBOX_CHECKED "true' type='checkbox' checked style='width: auto;"

// MQTT config
char mqttServer[40]   = "TiffanyAching.local";
char mqttPort[6]      = "1883";
char mqttClientName[40];
char mqttUser[40]     = "moist";
char mqttPassword[40] = "password";
char mqttOutTopic[40] = "ibnhouse/blinds/kitchen/pos";
char mqttInTopic[40]  = "ibnhouse/blinds/kitchen/cmd";

boolean offline = true;

WiFiClient espClient;
WiFiManager wifiManager;
WiFiManagerParameter customMqttServer("mqtt_server", "mqtt server", "mqtt server", 40);
WiFiManagerParameter customMqttPort("mqtt_port", "mqtt port", "port", 6);
WiFiManagerParameter customMqttClientName("mqtt_client_name", "mqtt client name", "mqtt client name", 16);
WiFiManagerParameter customMqttUser("mqtt_user", "mqtt user", "mqtt user", 16);
WiFiManagerParameter customMqttPassword("mqtt_password", "mqtt password", "mqtt password", 16);
WiFiManagerParameter customMqttOutTopic("mqtt_out_topic", "mqtt position output topic", "mqtt out topic", 40);
WiFiManagerParameter customMqttInTopic("mqtt_in_topic", "mqtt commands topic", "mqtt in topic", 40);
WiFiManagerParameter customBlindMaxPos("blind_max_pos", "blinds max pos", "60", 3);
WiFiManagerParameter customBlindInvertZero("blind_invert_zero", "", CHECKBOX, 70);
WiFiManagerParameter customBlindInvertZeroLabel("invert zero-position/direction");
WiFiManagerParameter customBlindInvertSwitch("blind_invert_switch", "", CHECKBOX, 70);
WiFiManagerParameter customBlindInvertSwitchLabel("invert switch keys");
WiFiManagerParameter customBlindDisableManualLock("disable_manual_lock", "", CHECKBOX, 70);
WiFiManagerParameter customBlindDisableManualLockLabel("disable manual lock");
bool wifiManagerSetupRunning = false;
bool restart = false;
unsigned long wifiManagerSetupStart;

PubSubClient mqttClient(espClient);
unsigned long mqttConnectAttempt = 0;
unsigned long mqttConnectDelay = 0;
WiFiUDP udp;
mDNSResolver::Resolver mDnsResolver(udp);
IPAddress mqttServerIp = INADDR_NONE;

unsigned long blindMaxPos = 60000;
boolean blindInvertZero = false;
boolean blindInvertSwitch = false;
boolean blindDisableManualLock = false;
long blindPos = 0;
boolean manualMode;
boolean manualLocked;
unsigned long manualLockActivatorTime;
int manualLockActivatorSwitch;
int manualSetupModeCounter;

unsigned long movementStartTime;
unsigned long movementTimeout;
boolean movementL1;
boolean movementProgress;

Ticker ledTicker;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SWITCH_S1, INPUT);
  pinMode(SWITCH_S2, INPUT);
  pinMode(RELAY_L1, OUTPUT);
  pinMode(RELAY_L2, OUTPUT);
  pinMode(BUTTON, INPUT);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP 

  // enable Auto Light Sleep (to reduce consumption during delay() periods in main loop)
  // enabling this + delay() in the main loop reduces consumption by module 0.8w -> 0.5w
  wifi_set_sleep_type(LIGHT_SLEEP_T); 

  digitalWrite(LED_BUILTIN, LOW); // turn it on

  manualLocked = ( digitalRead(SWITCH_S1) == LOW ) || ( digitalRead(SWITCH_S2) == LOW );
  manualSetupModeCounter = 0;

  stopL1();
  stopL2();

  startWifiManager(false);
  
  // MQTT connection
  mqttClient.setCallback(mqttCallback);

}

void loop() {

  gpioLoop();

  movementLoop();

  wifimanagerLoop();

  if (!offline) {
    mqttLoop();
  }

  // try to save power by delaying, which forces a light sleep
  if (!movementProgress) {
    delay(LOOP_DELAY_MS);
  }

  if (restart) {
    ESP.restart();
  }
}

void movementLoop() {
  if (movementProgress && (millis() - movementStartTime >= movementTimeout)) {
    stopL1();
    stopL2();
  }
}

void gpioLoop() {
  
  // is WifiManager configuration portal requested?
  if ( digitalRead(BUTTON) == LOW ) {
    startWifiManager(true);
  }

  // process manual switches
  unsigned long manualActivatorDelay = millis() - manualLockActivatorTime;
  if (manualActivatorDelay > MANUAL_LOCK_MIN_DELAY_MS) {
    if ( digitalRead(SWITCH_S1) == LOW ) {
      if (manualLockActivatorSwitch != SWITCH_S1) {
        manualSetupModeCounter = 0;
      }
      if (!blindDisableManualLock && manualLocked == false && (manualActivatorDelay < MANUAL_LOCK_MAX_DELAY_MS) && manualSetupModeCounter == 1) {
        manualLocked = true;
        manualMode = true;
        mqttCommunicate();
      }
      if (!manualLocked) {
        manualLockActivatorSwitch = SWITCH_S1;
        if (manualMode == false) {
          manualMode = true;
          manualLockActivatorTime = millis();
          mqttCommunicate(); // send to MQTT, to indicate manual mode
          if (blindInvertSwitch) {
            if (movementProgress == false || movementL1 == true) {
              startL2(0);
            }
          } else {
            if (movementProgress == false || movementL1 == false) {
              startL1(0);
            }
          }
        }
      }
    } else if ( digitalRead(SWITCH_S2) == LOW ) {
      if (manualLockActivatorSwitch == SWITCH_S1) {
        manualSetupModeCounter = 0;
      }
      if (!blindDisableManualLock && manualLocked == false && (manualActivatorDelay < MANUAL_LOCK_MAX_DELAY_MS) && manualSetupModeCounter == 1) {
        manualLocked = true;
        manualMode = true;
        mqttCommunicate();
      }
      if (!manualLocked) {
        manualLockActivatorSwitch = SWITCH_S2;
        if (manualMode == false) {
          manualMode = true;
          manualLockActivatorTime = millis();
          mqttCommunicate(); // send to MQTT, to indicate manual mode
          if (blindInvertSwitch) {
            if (movementProgress == false || movementL1 == false) {
              startL1(0);
            }
          } else {
            if (movementProgress == false || movementL1 == true) {
              startL2(0);
            }
          }
        }
      }
    } else {
      if (manualMode) {
        if (manualActivatorDelay < MANUAL_LOCK_MAX_DELAY_MS) {
          if(++manualSetupModeCounter == MANUAL_SETUP_COUNTER) {
            startWifiManager(true);
          }
        } else {
          manualSetupModeCounter = 0;
        }
        manualMode = false;
        manualLockActivatorTime = millis();
        if(manualLocked) {
          mqttCommunicate(); // send to MQTT, to indicate manual mode is off
        } else {
          if (movementProgress) {
            stopL1();
            stopL2();
          } else {
            mqttCommunicate();
          }
        }
      }
      manualLocked = false;
    }
  }
}

void wifimanagerLoop() {

  wifiManager.process();
  if (wifiManagerSetupRunning) {
    if ((millis() - wifiManagerSetupStart) > CONFIG_TIMEOUT_MS) {
      wifiManager.stopConfigPortal();
      wifiManagerSetupStopped();
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (offline) {
      mDnsResolver.setLocalIP(WiFi.localIP());
      mqttServerIp = mDnsResolver.search(mqttServer);
      // MQTT connection
      if(mqttServerIp != INADDR_NONE) {
        mqttClient.setServer(mqttServerIp, atoi(mqttPort));
      } else {
        mqttClient.setServer(mqttServer, atoi(mqttPort));
      }
    }
    offline = false;
  } else {
    offline = true;
  }

  if (!offline) {
    mDnsResolver.loop();
  }
  
}

void ledTick()
{
  int ledState = digitalRead(LED_BUILTIN);
  digitalWrite(LED_BUILTIN, !ledState);
}

//callback notifying us of the need to save config
void saveParamsCallback () {
  
  //save the custom parameters to FS
  //read updated parameters
  strcpy(mqttServer, customMqttServer.getValue());
  strcpy(mqttPort, customMqttPort.getValue());
  strcpy(mqttClientName, customMqttClientName.getValue());
  strcpy(mqttUser, customMqttUser.getValue());
  strcpy(mqttPassword, customMqttPassword.getValue());
  strcpy(mqttOutTopic, customMqttOutTopic.getValue());
  strcpy(mqttInTopic, customMqttInTopic.getValue());
  blindMaxPos = 1000 * atoi(customBlindMaxPos.getValue());
  blindInvertZero = strcmp("true", customBlindInvertZero.getValue()) == 0 ? true : false;
  blindInvertSwitch = strcmp("true", customBlindInvertSwitch.getValue()) == 0 ? true : false;
  blindDisableManualLock = strcmp("true", customBlindDisableManualLock.getValue()) == 0 ? true : false;

  DynamicJsonDocument json(1024);
  json["mqtt_server"] = mqttServer;
  json["mqtt_port"] = mqttPort;
  json["mqtt_client_name"] = mqttClientName;
  json["mqtt_user"] = mqttUser;
  json["mqtt_password"] = mqttPassword;
  json["mqtt_out_topic"] = mqttOutTopic;
  json["mqtt_in_topic"] = mqttInTopic;
  char blindMaxPosStr[4];
  strcpy(blindMaxPosStr, String(blindMaxPos / 1000).c_str());
  json["blind_max_pos"] = blindMaxPosStr;
  json["blind_invert_zero"] = blindInvertZero ? "true" : "false";
  json["blind_invert_switch"] = blindInvertSwitch ? "true" : "false";
  json["disable_manual_lock"] = blindDisableManualLock ? "true" : "false";

  File configFile = SPIFFS.open(CONFIG_FILE, "w");
  serializeJson(json, configFile);
  configFile.close();
  //end save
  
  wifiManagerSetupStopped();
}

void wifiManagerSetupStarted(WiFiManager *myWiFiManager) {
  ledTicker.attach_ms(1000, ledTick); // start slow blinking
  wifiManagerSetupRunning = true;
  wifiManagerSetupStart = millis();
}

void wifiManagerSetupStopped() {
  //ESP.restart();
  restart = true; // don't restart immediately. let WifiManager finish handleWifiSave() execution

  //ledTicker.detach();
  //digitalWrite(LED_BUILTIN, LOW); // turn it on
  //wifiManagerSetupRunning = false;

  //WiFi.hostname(mqttClientName);
}

void startWifiManager(boolean onDemand) {

  if (wifiManagerSetupRunning) {
    return;
  }

  if (!onDemand) {
    String apName = "Dorfl-" + String(ESP.getChipId(), HEX);
    strcpy(mqttClientName, apName.c_str());
    
    if (SPIFFS.begin()) {
      if (SPIFFS.exists("/config.json")) {
        //file exists, reading and loading
        File configFile = SPIFFS.open(CONFIG_FILE, "r");
        if (configFile) {
          size_t size = configFile.size();
          // Allocate a buffer to store contents of the file.
          std::unique_ptr<char[]> buf(new char[size]);
  
          configFile.readBytes(buf.get(), size);
          DynamicJsonDocument json(1024);
          DeserializationError jsonError = deserializeJson(json, buf.get());
          if (!jsonError) {
            if (json.containsKey("mqtt_server") && strlen(json["mqtt_server"]) > 0) strcpy(mqttServer, json["mqtt_server"]);
            if (json.containsKey("mqtt_port") && strlen(json["mqtt_port"]) > 0) strcpy(mqttPort, json["mqtt_port"]);
            if (json.containsKey("mqtt_client_name") && strlen(json["mqtt_client_name"]) > 0) strcpy(mqttClientName, json["mqtt_client_name"]);
            if (json.containsKey("mqtt_user") && strlen(json["mqtt_user"]) > 0) strcpy(mqttUser, json["mqtt_user"]);
            if (json.containsKey("mqtt_password") && strlen(json["mqtt_password"]) > 0) strcpy(mqttPassword, json["mqtt_password"]);
            if (json.containsKey("mqtt_out_topic") && strlen(json["mqtt_out_topic"]) > 0) strcpy(mqttOutTopic, json["mqtt_out_topic"]);
            if (json.containsKey("mqtt_in_topic") && strlen(json["mqtt_in_topic"]) > 0) strcpy(mqttInTopic, json["mqtt_in_topic"]);
            if (json.containsKey("blind_max_pos") && strlen(json["blind_max_pos"]) > 0) blindMaxPos = 1000 * atoi(json["blind_max_pos"]);
            if (json.containsKey("blind_invert_zero") && strlen(json["blind_invert_zero"]) > 0) blindInvertZero = strcmp("true", json["blind_invert_zero"]) == 0 ? true : false;
            if (json.containsKey("blind_invert_switch") && strlen(json["blind_invert_switch"]) > 0) blindInvertSwitch = strcmp("true", json["blind_invert_switch"]) == 0 ? true : false;
            if (json.containsKey("disable_manual_lock") && strlen(json["disable_manual_lock"]) > 0) blindDisableManualLock = strcmp("true", json["disable_manual_lock"]) == 0 ? true : false;
          }
        }
      }
    }
    //end read
  
    WiFi.hostname(mqttClientName);
  
    customMqttServer.setValue(mqttServer, 40);
    customMqttPort.setValue(mqttPort, 6);
    customMqttClientName.setValue(mqttClientName, 40);
    customMqttUser.setValue(mqttUser, 40);
    customMqttPassword.setValue(mqttPassword, 40);
    customMqttOutTopic.setValue(mqttOutTopic, 40);
    customMqttInTopic.setValue(mqttInTopic, 40);
    customBlindMaxPos.setValue(String(blindMaxPos / 1000).c_str(), 3);
  
    wifiManager.setSaveParamsCallback(saveParamsCallback);
    wifiManager.setAPCallback(wifiManagerSetupStarted);
  
    wifiManager.setConfigPortalTimeout(CONFIG_TIMEOUT_MS / 1000);
    wifiManager.setConfigPortalBlocking(false);
  
    //add all your parameters here
    wifiManager.addParameter(&customMqttServer);
    wifiManager.addParameter(&customMqttPort);
    wifiManager.addParameter(&customMqttClientName);
    wifiManager.addParameter(&customMqttUser);
    wifiManager.addParameter(&customMqttPassword);
    wifiManager.addParameter(&customMqttOutTopic);
    wifiManager.addParameter(&customMqttInTopic);
    wifiManager.addParameter(&customBlindMaxPos);
    wifiManager.addParameter(&customBlindInvertZero);
    wifiManager.addParameter(&customBlindInvertZeroLabel);
    wifiManager.addParameter(&customBlindInvertSwitch);
    wifiManager.addParameter(&customBlindInvertSwitchLabel);
    wifiManager.addParameter(&customBlindDisableManualLock);
    wifiManager.addParameter(&customBlindDisableManualLockLabel);

  }
  
  // refresh dirty hacked boolean values
  customBlindInvertZero.setValue(blindInvertZero ? CHECKBOX_CHECKED : CHECKBOX, 70);
  customBlindInvertSwitch.setValue(blindInvertSwitch ? CHECKBOX_CHECKED : CHECKBOX, 70);
  customBlindDisableManualLock.setValue(blindDisableManualLock ? CHECKBOX_CHECKED : CHECKBOX, 70);
  
  if (onDemand) {
    wifiManager.startConfigPortal(mqttClientName);
  } else {
    wifiManager.autoConnect(mqttClientName);
  }

}

void startL1(unsigned long timeout) {
  stopL2(); // should never be running both at same time!!11, as closing both relays may damage blinds motor
  digitalWrite(RELAY_L1, HIGH); // turn on
  movementProgress = true;
  movementL1 = true;
  movementStartTime = millis();
  movementTimeout = timeout == 0 ? blindMaxPos : timeout;
}

void startL2(unsigned long timeout){
  stopL1(); // should never be running both at same time!!11, as closing both relays may damage blinds motor
  digitalWrite(RELAY_L2, HIGH); // turn on
  movementProgress = true;
  movementL1 = false;
  movementStartTime = millis();
  movementTimeout = timeout == 0 ? blindMaxPos : timeout;
}

void stopL1(){
    digitalWrite(RELAY_L1, LOW); // turn off
    if (movementProgress && movementL1 == true) {
      movementProgress = false;
      updateBlindPos();
    }
}

void stopL2(){
    digitalWrite(RELAY_L2, LOW); // turn off
    if (movementProgress && movementL1 == false) {
      movementProgress = false;
      updateBlindPos();
    }
}

void updateBlindPos() {
    unsigned long moved = millis() - movementStartTime;
    //movementStartTime = millis();
    if (movementL1) {
      if (blindInvertZero) {
        blindPos += moved;
      } else {
        blindPos -= moved;
      }
    } else {
      if (blindInvertZero) {
        blindPos -= moved;
      } else {
        blindPos += moved;
      }
    }
    if (blindPos < 0) {
      blindPos = 0;
    } else if (blindPos > blindMaxPos) {
      blindPos = blindMaxPos;
    }

    // send current position to mqtt
    mqttCommunicate();
}

void autoMoveToPos(long newPos) {
  //mqttDebug(newPos);
  if (!manualMode){
    long deltaMove = newPos - blindPos;
    if (blindInvertZero) {
      if (deltaMove > 0) {
        startL1(deltaMove);
      } else if (deltaMove < 0) {
        startL2(-deltaMove);
      }
    } else {
      if (deltaMove > 0) {
        startL2(deltaMove);
      } else if (deltaMove < 0) {
        startL1(-deltaMove);
      }
    }
  }
}

void mqttLoop() {
  if (!mqttClient.connected()) {
    if(!mqttReconnect()){
      return;
    }
  }
  mqttClient.loop();
}

void mqttCommunicate() {
  
  if (!offline && mqttClient.connected()) {
    char mqttMsg[50];
    ltoa(blindPos / 1000, mqttMsg, 10);
    if (manualMode) {
      strcat(mqttMsg, ".");
    }
    mqttClient.publish(mqttOutTopic, mqttMsg, true);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  char payloadCopy[10];
  int _length = std::min<unsigned int>(length, 9);
  strncpy(payloadCopy, (char*)payload, _length);
  payloadCopy[_length] = 0x0;

  String cmd = String(payloadCopy);
  if (cmd.startsWith(CMD_SETUP)){
    startWifiManager(true);
  } else if (cmd.startsWith(CMD_MOVE_ABS)) {
    if (movementProgress && !manualMode) {
      stopL1();
      stopL2();
    }
    int absPos = atoi(cmd.substring(3).c_str());
    long newPos = absPos*1000;
    autoMoveToPos(newPos);
  } else if (cmd.startsWith(CMD_MOVE_REL)) {
    if (movementProgress && !manualMode) {
      stopL1();
      stopL2();
    }
    int relPos = atoi(cmd.substring(3).c_str());
    long newPos = blindPos + relPos*1000;
    autoMoveToPos(newPos);
  }

}

boolean mqttReconnect() {
  if (!mqttClient.connected()) {
    if (millis() - mqttConnectAttempt > mqttConnectDelay ) { // don't attempt more often than a delay
      mqttConnectDelay += 1000; // increase reconnect attempt delay by 1 second
      if (mqttConnectDelay > 60000) { // don't attempt more often than once a minute
        mqttConnectDelay = 60000;
      }
      // Attempt to connect
      mqttConnectAttempt = millis();
      if (mqttClient.connect(mqttClientName, mqttUser, mqttPassword)) {
        // Once connected resubscribe
        mqttClient.subscribe(mqttInTopic);
        mqttConnectDelay = 0;
        return true;
      }
    }
  }
  return false;
}
