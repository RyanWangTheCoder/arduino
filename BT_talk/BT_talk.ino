#include <ArduinoBLE.h>

BLEService ledService("180A"); // BLE customized service

// BLE customized Switch Characteristic - custom 128-bit UUID, read and writable by central
//BLEByteCharacteristic dataCharacteristic("2A57", BLERead | BLEWrite);
BLEStringCharacteristic dataCharacteristic("2A57", BLERead | BLEWrite, 20);
int i = 0, count;
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("BLE_alpineridge2");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(dataCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characteristic:
  //dataCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE talk to Android");
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to android: ");
    // print the central's MAC address:
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);            // turn on the LED to indicate the connection
    String receivedData = "";
    // while the central is still connected to peripheral:
    i = 0;
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      while (dataCharacteristic.written()) {
          count = dataCharacteristic.valueSize();
          Serial.print("valueLength: ");
          Serial.println(count);
          if (count > 0) {
            //Serial.print("read ");
            //Serial.println(dataCharacteristic.read());
            //dataCharacteristic.readValue()
            receivedData = dataCharacteristic.value();
            // Process the received data (e.g., parse JSON)
            Serial.print("Received data: ");
            Serial.println(receivedData);
            //dataCharacteristic.
          }
          else
            break;
        }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());  }
}