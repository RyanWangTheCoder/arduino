#include <ArduinoBLE.h>

// Define the name of the Bluetooth peripheral
const char* peripheralName = "BT_new";

// Create a BLE Service
BLEService fileTransferService("19B10000-E8F2-537E-4F6C-D104768A1214");

// Create a BLE Characteristic for receiving data
const char* dataCharacteristicUUID = "19B10001-E8F2-537E-4F6C-D104768A1214";
BLECharacteristic dataCharacteristic(dataCharacteristicUUID, BLEWrite | BLERead, "");

void setup() {
  // Initialize the Serial communication for debugging
  Serial.begin(9600);

  // Initialize the BLE library
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the local name of the Bluetooth peripheral
  BLE.setLocalName(peripheralName);
  BLE.setAdvertisedService(fileTransferService);

  // Add the characteristics to the service
  fileTransferService.addCharacteristic(dataCharacteristic);

  // Start advertising
  BLE.advertise();

  Serial.println("Bluetooth peripheral advertising...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    char receivedData [200] ;

    while (central.connected()) {
      if (dataCharacteristic.written()) {
        strcpy (receivedData, (const char*) dataCharacteristic.value());
        // Process the received data (e.g., parse JSON)
        Serial.print("Received data: ");
        Serial.println(receivedData);
      }
    }
  }
}