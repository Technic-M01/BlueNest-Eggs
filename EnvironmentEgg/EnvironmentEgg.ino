/*
  Callback LED

  This example creates a Bluetooth® Low Energy peripheral with service that contains a
  characteristic to control an LED. The callback features of the
  library are used.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <NoDelay.h>

#define SEA_LEVEL_PRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service

BLEService nameService("181C");

BLEService bmeService("1809");

// create switch characteristic and allow remote device to read and write
// BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEStringCharacteristic nameCharacteristic("2A00", BLERead | BLENotify, 17);

BLECharacteristic bmeCharacteristic("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 16);

#define INFO_LENGTH 20

BLECharacteristic infoCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify, INFO_LENGTH);

typedef struct __attribute__( ( packed ) ) {
  char bytes[INFO_LENGTH + 1];
  bool updated = false;
} text_data_t;

text_data_t textData = {"Hello Xiao", false};

const int led_r = LEDR;
const int led_b = LEDB;
const int led_g = LEDG;

int leds[] = {led_r, led_g, led_b};

bool updateName = false;
long wait = 2000;
unsigned long now = 0;

union ConvertBme {
  unsigned long longNumber;
  byte longBytes[4];
};

union FloatVal {
  float f;
  unsigned long l;
  byte b[4];
} value;


// byte array to hold value of each sensor reading.
byte sensorHoldArray[16];


void sampleEnvironment() {
  sensors_event_t temp_event, humidity_event, pressure_event;
  bme_temp->getEvent(&temp_event);
  bme_humidity->getEvent(&humidity_event);
  bme_pressure->getEvent(&pressure_event);

  float temp_c, temp_f;
  temp_c = temp_event.temperature;
  temp_f = (temp_c * 1.80) + 32;


/*
  ConvertBme cvtbme;
  cvtbme.longNumber = temp_f;
  Serial.println(cvtbme.longBytes[0], HEX);

  byte bytesToSend[sizeof(temp_f)];
  char szChar[20];

  Serial.print("float val is: "); Serial.println(temp_f, 2);

  Serial.print("Bytes: ");
  memcpy(bytesToSend, (uint8_t *)&temp_f, sizeof(float));
  for (uint8_t i=0; i<sizeof(float); i++) {
        sprintf( szChar, "0x%02X ", bytesToSend[i] ); // <- Prints BACKWARDS
        Serial.print( szChar );
  }

  Serial.println();
*/

/*
  unsigned y;
  unsigned char *chpt;
  chpt = (unsigned char *)&temp_f;
  Serial.print("Float value             : ");
  Serial.println(temp_f);
  Serial.print("4 byte hexadecimal value: ");
  for (uint8_t i = 0; i < sizeof(temp_f); i++) {
    Serial.print(chpt[i], HEX);
  }
  Serial.println();

  for (uint8_t i = sizeof(temp_f); i > 0; i--) {
    Serial.print(i - 1); Serial.print(" "); Serial.println(chpt[i - 1], HEX);    
  }


  Serial.print("size of float arr: "); Serial.println(sizeof(chpt));
  int fStrSize = sizeof(chpt) * 2 + 1;
  Serial.println(fStrSize);

  char floatString[fStrSize];
  sprintf(floatString, "%02X%02X%02x%02x", chpt[3], chpt[2], chpt[1], chpt[0]);
  floatString[8] = '\0';
  Serial.print("float string: "); Serial.println(floatString);
*/
  bytesFromFloat(temp_f, 0);


  Serial.print("Temperature: "); Serial.println(temp_f);
  Serial.println(typeStr(temp_f));


  bytesFromFloat(humidity_event.relative_humidity, 4);

  Serial.print("Humidity: "); Serial.println(humidity_event.relative_humidity);
  Serial.println(typeStr(humidity_event.relative_humidity));

  bytesFromFloat(pressure_event.pressure, 8);

  Serial.print("Pressure: "); Serial.println(pressure_event.pressure);
  Serial.println(typeStr(pressure_event.pressure));

  bytesFromFloat(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA), 12);
  
  Serial.print("Altitude: "); Serial.println(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA));
  Serial.println(typeStr(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA)));


  bmeCharacteristic.writeValue(sensorHoldArray, 16);

}

// puts bytes from inputByte param into global var that holds each sensor reading. 
void returnByte(byte* inputByte, int multiplier) {
  for (int i = 0; i < sizeof(inputByte); i++) {
    Serial.print("return byte - sensorHoldArray size: "); Serial.println(sizeof(sensorHoldArray));

    sensorHoldArray[i + multiplier] = inputByte[i];
  }
}

// prints value of given byte array in hex
void dumpByteArray(const byte * byteArray, const byte arraySize) {
  Serial.print("dump bytes              : ");
  for (int i = 0; i < arraySize; i++) {
    Serial.print("0x");
    if (byteArray[i] < 0x10)
    Serial.print("0");
    Serial.print(byteArray[i], HEX);
    Serial.print(", ");
  
  }
  Serial.println();
}

void bytesFromFloat(float fValue, int m) {
  unsigned y;
  unsigned char *chpt;
  chpt = (unsigned char *)&fValue;
  Serial.print("Float value             : ");
  Serial.println(fValue);
  Serial.print("4 byte hexadecimal value: ");
  for (uint8_t i = 0; i < sizeof(fValue); i++) {
    Serial.print(chpt[i], HEX);
  }
  Serial.println();

  // for (uint8_t i = sizeof(fValue); i > 0; i--) {
  //   Serial.print(i - 1); Serial.print(" "); Serial.println(chpt[i - 1], HEX);    
  // }

  // Serial.print("size of float arr: "); Serial.println(sizeof(chpt));
  int fStrSize = sizeof(chpt) * 2 + 1;

  char floatString[fStrSize];
  sprintf(floatString, "%02X%02X%02x%02x", chpt[3], chpt[2], chpt[1], chpt[0]);
  floatString[8] = '\0';
  Serial.print("float string            : "); Serial.println(floatString);

  int arrSize = (sizeof(floatString) - 1) / 2;
  byte sendBytes[arrSize];
  

  int r = 0;
  for (uint8_t i = arrSize; i > 0; i--) {
    sendBytes[r] = chpt[i - 1];
    r++;
  }

  returnByte(sendBytes, m);
  dumpByteArray(sensorHoldArray, 16);
}

noDelay getEnvironment(5000);

bool centralSubscribed = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  pinMode(led_b, OUTPUT); // use the LED pin as an output
  pinMode(led_r, OUTPUT);
  pinMode(led_g, OUTPUT);

  resetLeds();

  // -- testing leds work (seeed documentation is ~always~ wrong)

  // setLedState(0, true);
  // delay(2000);
  // setLedState(0, false);
  // setLedState(1, true);
  // delay(2000);
  // setLedState(1, false);
  // setLedState(2, true);
  // delay(2000);
  // setLedState(2, false);

  unsigned status;
  status = bme.begin(0x76);

  bme_temp->printSensorDetails();
  bme_humidity->printSensorDetails();
  bme_pressure->printSensorDetails();

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("LEDCallback");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);
  
  nameService.addCharacteristic(infoCharacteristic);
  nameService.addCharacteristic(nameCharacteristic);

  bmeService.addCharacteristic(bmeCharacteristic);

  infoCharacteristic.writeValue(textData.bytes, sizeof textData.bytes);
  nameCharacteristic.writeValue("tkm01 test xiao");


  // add service
  BLE.addService(ledService);
  BLE.addService(nameService);
  BLE.addService(bmeService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
  // set an initial value for the characteristic
  switchCharacteristic.setValue(0);

  infoCharacteristic.setEventHandler(BLEWritten, infoCharacteristicWritten);
  infoCharacteristic.setEventHandler(BLESubscribed, infoCharacteristicSubscribed);
  infoCharacteristic.setEventHandler(BLEUnsubscribed, infoCharacteristicUnsubscribed);

  bmeCharacteristic.setEventHandler(BLESubscribed, bmeCharacteristicSubscribed);
  bmeCharacteristic.setEventHandler(BLEUnsubscribed, bmeCharacteristicUnsubscribed);


  sampleEnvironment();

  // start advertising
  BLE.advertise();

  Serial.println(("Bluetooth® device active, waiting for connections..."));
}

void loop() {
  // poll for Bluetooth® Low Energy events
  BLE.poll();

  // testing the BLESubscribe method
  if (updateName) {
    // after 2 seconds, change the value of the info char and check change was shown on central.
    if (millis() > now + wait) {
      Serial.println("Updating info characterisitc");
      updateName = false;
      text_data_t nName = {"updated name!!", false};
      infoCharacteristic.writeValue(nName.bytes, sizeof nName.bytes);
    }
  }

  if (centralSubscribed) {
    if(getEnvironment.update()) {
      sampleEnvironment();
    }
  }

}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  setLedState(2, true);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  setLedState(2, false);
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");

  uint32_t len = characteristic.valueLength();
  byte val = 0;
  switchCharacteristic.readValue(val);

  if (val == 1 || val == 0) {
    if (val == 1) {
      Serial.println("LED on");
      setLedState(0, true);
    } else {
      Serial.println("LED off");
      setLedState(0, false);
    }
  }
  else {
    Serial.println("VALUE OUT OF RANGE OR UNSUPPORTED DATA TYPE");
  }

  Serial.println(typeStr(switchCharacteristic.value()));
  Serial.println(switchCharacteristic.value());
}

void infoCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Characteristic event (info), written: ");

  uint32_t length = characteristic.valueLength();
  infoCharacteristic.readValue(textData.bytes, length);
  textData.bytes[length] = '\0';

  Serial.println(textData.bytes);
  Serial.println(typeStr(textData.bytes));

}

void infoCharacteristicSubscribed(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic
  Serial.print("Characterisitic subscribed. UUID: ");
  Serial.println(characteristic.uuid());

  updateName = true;
  now = millis();

  // centralSubscribed = true;

}

void infoCharacteristicUnsubscribed(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Characterisitic unsubscribed. UUID: ");
  Serial.println(characteristic.uuid());

  // centralSubscribed = false;
}

void bmeCharacteristicSubscribed(BLEDevice central, BLECharacteristic characterisitc) {
  Serial.print("Characterisitic subscribed. UUID: ");
  // Serial.println(characteristic.uuid());

  centralSubscribed = true;

}

void bmeCharacteristicUnsubscribed(BLEDevice central, BLECharacteristic characterisitc) {
  Serial.print("Characterisitic unsubscribed. UUID: ");
  // Serial.println(characteristic.uuid());

  centralSubscribed = false;
}


// for the builtin LEDs, HIGH = off, LOW = on
void setLedState(int led, bool state) {
  if (state) {
    digitalWrite(leds[led], LOW);
  } else {
    digitalWrite(leds[led], HIGH);
  }
}

// for some reason, turning on led_r for the first time turns on all 3 leds.
// turning the led on, and quickly turning all colors off seems to fix any issues that crop up regarding this initial led state.
void resetLeds() {
  digitalWrite(led_r, LOW);
  digitalWrite(led_r, HIGH);
  digitalWrite(led_g, HIGH);
  digitalWrite(led_b, HIGH);
  delay(10);
}

inline const char * typeStr (int   var) { return "int"; }
inline const char * typeStr (long  var) { return "long"; }
inline const char * typeStr (float var) { return "float"; }
inline const char * typeStr (const char *var) { return "char"; }
inline const char * typeStr (uint8_t* var)    { return "uint8_t"; }



