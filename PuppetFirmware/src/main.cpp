#include <Arduino.h>


#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "PCA9685.h"
#include <string>

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// 150
#define UPDATE_RATE 400
#define UPDATE_DELAY 1000/UPDATE_RATE

#define V_BAT_CHARGER 34
#define CELL_INNER 35

unsigned long prevCycleTime = 0;
unsigned long lastStreamTime = 0;

uint16_t streamTimeout = 5000;

enum states {ON, IDLE_SETUP, IDLE, STREAM};
enum states applicationState;
bool paused = false;

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

uint8_t txMsg[] = {0, 0, 0};
bool sendBatt = false;

uint8_t macAdress[6];

Preferences pref;
PCA9685 pwmController;                
String macString;
String tmpMsg;

char tmpChar = 'a';

// Handles the Bluetooth LE connection and disconnection calllbacks.
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        Serial.println("Connected");
    };

    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
        Serial.println("Disconnected");
    }
};

// This class is used to interface the servos. Similar to Arduino's default Servo class with added features.
class simpleServo {

    public:

    uint8_t homePos = 127, currentPos = 127, targetPos = 127, step, amplitude, period, pausePos = homePos, channel, resolution = 16, freq = 50;
    int lowerLimit = 0, upperLimit = 255; 
    double factor = 65536.0 * (1.0/20.0), offset = factor;

    double lastMoved = 0;
    double timeout = 2000;

    // Attaches the PWM controller to the appropiate channel.
    void attach() {
        pwmController.setChannelPWM(channel, 0);
    }

    // Writes a position to the servo. Input value ranging from 0 to 255 gets remapped to hardcoded values.
    void write(uint8_t pos) {

        if(pos < lowerLimit) {
            //Serial.println(String(pin) + "pin servo lowerLimit reached, pos: " + String(pos) + ", " + String(lowerLimit));
            pos = lowerLimit;
        } else if (pos > upperLimit) {
            //Serial.println(String(pin) + "pin upperLimit reached, pos: " + String(pos) + ", " + String(upperLimit));
            pos = upperLimit;
        }

        pwmController.setChannelPWM(channel, map(pos, 0, 255, 205, 410));
    }

    // Shuts off the servo.
    void writeLow() {
        pwmController.setChannelPWM(channel, 0);
    }

    // Sets the servo's home position.
    void setHome(uint8_t _homePos) {
        homePos = _homePos;
    }

    // Sets the servo's step variable, which is used during interpolation.
    void setStep(uint8_t _step) {
        step = _step;
    }

    // Sets targetPos variable, used in interpolation.
    void setTargetPos(uint8_t _targetPos) {
        targetPos = _targetPos;
    }

    // Interpolates from currentPos to targetPos with step varable. One function call does one step towars target position. 
    void interpolate()  {

        if (currentPos != targetPos) {
            lastMoved = millis();
        }
        
        if (currentPos < targetPos - step) {
            currentPos += step;
        } else if (currentPos > targetPos + step){
            currentPos -= step;
        } else {
            currentPos = targetPos;

        }

        if (lastMoved + timeout <= millis()){

            // Uncomment this to save power
            writeLow();

            //Serial.println("timeout");
        } else {
            write(currentPos);
            //Serial.println(String(pin) + " target: " + String(targetPos) +  ", current: " + String(currentPos) + ", in degrees:" + String(map(currentPos, 0, 255, 9, 171)));
        }
    }

    // Detects is the servo is stationary
    bool isStaionary () {
        if (targetPos == currentPos) {
            return true;
        } else 
            return false;
    }
};

simpleServo servos[6];

// Character to HEX converter.
char c2h(char c) {
    return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}

// Saves the user's settings to Preferences
void saveSettings(){

    pref.begin("marionette", false);

    pref.putLong("streamTimeout", streamTimeout);
    // Serial.println("Saved streamTimeout: " + String(streamTimeout) );

    for(uint8_t i = 0; i < 6; i++) {
        char step_i[] = {'s', 't', 'e', 'p', i + 48, '\0'};
        pref.putUInt( step_i, (uint32_t) servos[i].step);
        // Serial.println("Saved step: " + String(servos[i].step) );
        
        /*
        char amp_i[] = {'a', 'm', 'p', i + 48, '\0'};
        pref.putUInt( amp_i, (uint32_t) servos[i].amplitude);
        // Serial.println("Saved amplitude: " + String(servos[i].amplitude) );

        char period_i[] = {'p', 'e', 'r', 'i','o', 'd', i + 48, '\0'};
        pref.putUInt( period_i, (uint32_t) servos[i].period);
        // Serial.println("Saved period: " + String(servos[i].period) );
        */

        char home_i[] = {'h', 'o', 'm', 'e', i + 48, '\0'};
        pref.putUInt( home_i, (uint32_t) servos[i].homePos);
        // Serial.println("Saved homePos " + String(servos[i].homePos) );

        char timeout_i[] = {'t', 'i', 'm', 'e','o', 'u', 't', i + 48, '\0'};
        pref.putULong( timeout_i, (uint32_t) servos[i].timeout);
        // Serial.println("Saved timeout: " + String(servos[i].timeout) );

        char lowcut_i[] = {'l', 'o', 'w', 'c', 'u', 't', i + 48, '\0'};
        pref.putUInt( lowcut_i, (uint32_t) servos[i].lowerLimit);
        // Serial.println("Saved lowerLimit: " + String(servos[i].lowerLimit) );

        char upcut_i[] = {'u', 'p', 'c', 'u', 't', i + 48, '\0'};
        pref.putUInt( upcut_i, (uint32_t) servos[i].upperLimit);
        // Serial.println("Saved upperLimit: " + String(servos[i].upperLimit) );
    }
    pref.end();
}

// Loads the user's settings from Preferences
void loadSettings(){

    pref.begin("marionette", true);

    streamTimeout = pref.getLong("streamTimeout");
    // Serial.println("streamTimeout: " + String(pref.getLong("streamTimeout")) );

    for(uint8_t i = 0; i < 6; i++) {
        char step_i[] = {'s', 't', 'e', 'p', i + 48, '\0'};
        servos[i].step = pref.getUInt( step_i );
        // Serial.println("step" + String(i) + ": " + String(pref.getUInt( step_i)) );

        /*
        char amp_i[] = {'a', 'm', 'p', i + 48, '\0'};
        servos[i].amplitude = pref.getUInt( amp_i );
        // Serial.println("amplitude" + String(i) + ": " + String(pref.getUInt( amp_i)) );

        char period_i[] = {'p', 'e', 'r', 'i','o', 'd', i + 48, '\0'};
        servos[i].period = pref.getUInt( period_i );
        // Serial.println("period" + String(i) + ": " + String(pref.getUInt( period_i)) );
        */

        char home_i[] = {'h', 'o', 'm', 'e', i + 48, '\0'};
        servos[i].homePos = pref.getUInt( home_i );
        // Serial.println("home" + String(i) + ": " + String(pref.getUInt( home_i)) );

        char lowcut_i[] = {'l', 'o', 'w', 'c', 'u', 't', i + 48, '\0'};
        servos[i].lowerLimit = pref.getUInt( lowcut_i );
        // Serial.println("lowerLimit" + String(i) + ": " + String(pref.getUInt( lowcut_i)) );
        
        char upcut_i[] = {'u', 'p', 'c', 'u', 't', i + 48, '\0'};
        servos[i].upperLimit = pref.getUInt( upcut_i );
        // Serial.println("upperLimit" + String(i) + ": " + String(pref.getUInt( upcut_i)) );
    }
    pref.end();
}

// Parses the incomming message, either BLE or serial and executes the appropriate action.  
void parseValue(std::string msg) {

    if(msg.length() == 0)
        return;

    if (msg[0] == 0x10 && msg.length() == 7) {
        // Get servo stream

        lastStreamTime = millis();

        paused = false;
        applicationState = STREAM;

        for(int i = 0; i < 6; i++) {
            servos[i].setTargetPos(msg[i+1]);
        }

        return;

    } else if(msg[0] == 0x20) {
        // Stop
        // Serial.println("Idle");

        applicationState = IDLE_SETUP;

        return;

    } else if(msg[0] == 0x21) {
        // Pause
        Serial.println("Paused");

        paused = true;

        for(int i = 0; i < 6; i++) {   
            servos[i].targetPos = servos[i].currentPos;
            servos[i].pausePos = servos[i].currentPos;

            // DEBUG For testing "violent breathing" bug
            // Serial.printf("   Servo #%d: | A: %d | T: %d\r\n", i, servos[i].amplitude, servos[i].period);

        }
        
        return;

    } else if(msg[0] == 0x22 && msg.length() == 2) {
        // Set Step
        // Serial.println("Set Step");

        for(int i = 0; i < 6; i++) {
            servos[i].setStep(msg[1]);
        }

    } else if(msg[0] == 0x23 && msg.length() == 13) {
        // Set servo breath function parameters
        // Serial.println("Set servo breath function parameters");

        /*for(int i = 0; i < 6; i++) {
            servos[i].amplitude = msg[i+1];
            servos[i].period = msg[i + 7];
        }*/

    } else if(msg[0] == 0x24 && msg.length() == 7) {
        // Set servo home postion
        // Serial.println("Set servo home postion");

        for(int i = 0; i < 6; i++) {
            servos[i].setHome(msg[i+1]);
        }

    } else if(msg[0] == 0x25 && msg.length() == 3) {
        // Set Timeout
        // Serial.println("Set Timeout");

        streamTimeout = msg[1] << 8 | msg[2];

    }else if(msg[0] == 0x26 && msg.length() == 13) {
        // Set Timeout
        // Serial.println("Set Limits");

        for(int i = 0; i < 6; i++) {
            servos[i].lowerLimit = msg[i+1];
            servos[i].upperLimit = msg[i + 7];
        }

    } else if(msg[0] == 0x30) {
        // Get battery charge
        // Serial.println("Get battery charge");

        uint16_t VbatCharget = analogRead(V_BAT_CHARGER);
        uint16_t CellInner = analogRead(CELL_INNER);

        Serial.print("VbatCharger:" + String(VbatCharget));
        Serial.println(",CellInner:" + String(CellInner));

        txMsg[0] = 0x30;
        txMsg[1] = VbatCharget / 16;
        txMsg[2] = CellInner / 16;

        sendBatt = true;

    } else if(msg[0] == 0x31) {
        // Get battery charge
        // Serial.println("PC disconnect");

        applicationState = ON;

    } else if(msg == "puppet") {
        // Get battery charge
        Serial.println(macString);

        return;

    } else {
        return;
    }

    // Save only when something changed
    saveSettings();

}

// Handles incoming BLE messages.
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();
        parseValue(rxValue);
    }
};

// Generates an unique mac adress for the puppet if there is not one already.
void generateMacIfNeeded() {
    pref.begin("marionette", false);
    
    pref.getBytes("MAC", macAdress, 6);

    Serial.print("Found macAdress:");
    for (int i = 0; i < 6; i++) {
        Serial.print(String(c2h(macAdress[i] >> 4)) + String(c2h(macAdress[i])));
    }
    Serial.println("");



    if(macAdress[0] != 0x3c) {
        macAdress[0] = 0x3c;
        macAdress[1] = 0x4a;
        macAdress[2] = 0xd5;
        macAdress[3] = random(0, 255);
        macAdress[4] = random(0, 255);
        macAdress[5] = random(0, 255);
        pref.putBytes("MAC", macAdress, 6);
    }

    pref.end();

    esp_base_mac_addr_set(macAdress); // weird bug found: sixth character is increased by 2 in the mac address
    macAdress[5] += 2; 
}

// Arduino's setup function. Runs once during code execution.
void setup() {
    // Signal Reset on DEVBOARD:
    pinMode(26, OUTPUT);
    digitalWrite(26, 0);
    delay(200);
    digitalWrite(26, 1);

    Serial.begin(115200);

    generateMacIfNeeded();

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmController.init(0x40);        // Address pins A5-A0 set to B000000
    pwmController.setPWMFrequency(50); // Default is 200Hz, supports 24Hz to 1526Hz

    applicationState = ON;
   
    pinMode(V_BAT_CHARGER, INPUT);
    pinMode(CELL_INNER, INPUT);

    // Default values for testing purposes 
    for(int i = 0; i < 6; i++) {

        servos[i].amplitude = 0;
        servos[i].period = 4;
        servos[i].homePos = 127;
        servos[i].channel = i;
        servos[i].setStep(3);

        servos[i].attach();

        loadSettings();

        servos[i].setTargetPos(servos[i].homePos);
    }

    servos[1].amplitude = 11;

    loadSettings(); 

    // Create the BLE Device
    // BLEDevice::init("ESP32");

    macString = "puppet" 
        + String(c2h(macAdress[3] >> 4)) + String(c2h(macAdress[3]))
        + String(c2h(macAdress[4] >> 4)) + String(c2h(macAdress[4])) 
        + String(c2h(macAdress[5] >> 4)) + String(c2h(macAdress[5]));

    // Serial.println("macString is " + macString);

    BLEDevice::init( macString.c_str() );

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY);

    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE);

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");

}

// Moves the puppet based on a sinusoidal curve.
void breath(int basePos0, int basePos1, int basePos2, int basePos3, int basePos4, int basePos5) {
    
    int basePos[] = {basePos0, basePos1, basePos2, basePos3, basePos4, basePos5};

    for(int i = 0; i < 6; i++) {
        // BUG FIXME Timing is not right, maybe the radians cause that or the 250 multiplying value
        servos[i].setTargetPos(basePos[i] + sin( (float) millis() / ( (float) 250 * (float) servos[i].period ) ) * servos[i].amplitude);
    }
}

// Arduino's loop function. Runs program logic continuously. Main state machine is here.
void loop() {

    if(Serial.available() > 0) {
        while (Serial.available() > 0) {
            tmpMsg += (char) Serial.read();
        }
        parseValue(tmpMsg.c_str());
        tmpMsg = "";
    }


    if (prevCycleTime + UPDATE_DELAY <= millis()) {
        prevCycleTime = millis();

        for(int i = 0; i < 6; i++) {
            servos[i].interpolate();
        }

        if (applicationState == ON) {

        } else if (applicationState == IDLE_SETUP) {
            
            applicationState = IDLE;
            
            for(int i = 0; i < 6; i++) {
                servos[i].setTargetPos(servos[i].homePos);

                if (!servos[i].isStaionary()) {
                    applicationState = IDLE_SETUP;
                }
            }
            
        } else if (applicationState == IDLE) {
            breath( servos[0].homePos, servos[1].homePos, servos[2].homePos, servos[3].homePos, servos[4].homePos, servos[5].homePos );
        } else if (applicationState == STREAM) {
            
            if(paused) {
                breath( servos[0].pausePos, servos[1].pausePos, servos[2].pausePos, servos[3].pausePos, servos[4].pausePos, servos[5].pausePos );

            } else {
                /* if(lastStreamTime + streamTimeout <= millis()) {
                    applicationState = IDLE_SETUP;
                 } */
            }
            
        }
    
    
        if (deviceConnected) {

           if(sendBatt) {
               sendBatt = false;

                pTxCharacteristic->setValue(txMsg, 3);
                pTxCharacteristic->notify();

           }

        }

        // disconnecting
        if (!deviceConnected && oldDeviceConnected) {

            applicationState = ON;

            delay(500);                  // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;
        }

        // connecting
        if (deviceConnected && !oldDeviceConnected) {

            applicationState = IDLE_SETUP;

            // do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
    }

}