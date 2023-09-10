#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"

//#include <Arduino_LSM6DS3.h>
//#include <Arduino_LSM9DS1.h>

BLEService atheletService("180A"); // BLE customized service

// BLE customized Switch Characteristic - custom 128-bit UUID, read and writable by central
//BLEByteCharacteristic dataCharacteristic("2A57", BLERead | BLEWrite);
BLEStringCharacteristic dataCharacteristic("2A57", BLERead | BLEWrite, 20);

int i = 0, count;
int nano33IMUSetup = 0;
float x, y, z;  // IMU data, shared for both accellerometer and gyro

void setupAD8232() {
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -
}
void setupNano33Accelerometer() {
  if (nano33IMUSetup == 0) {
    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    nano33IMUSetup = 1;
  }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
}
void setupNano33Gyroscope() {
  if (nano33IMUSetup == 0) {
    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    nano33IMUSetup = 1;
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}
void setupNano33Magnetometer() {
  if (nano33IMUSetup == 0) {
    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    nano33IMUSetup = 1;
  }
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" uT");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
}
/*
void setupNano33Temperature() {
  if (nano33IMUSetup == 0) {
    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    nano33IMUSetup = 1;
  }
  Serial.print("Temperature sensor sample rate = ");
  Serial.print(IMU.temperatureSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Temperature reading in degrees C");
  Serial.println("T");
}*/

void setupBLE()
{
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("BLE_alpineridge2");
  BLE.setAdvertisedService(atheletService);

  // add the characteristic to the service
  atheletService.addCharacteristic(dataCharacteristic);

  // add service
  BLE.addService(atheletService);

  // set the initial value for the characteristic:
  //dataCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE talk to Android");  

  BLEDevice central = BLE.central();
  Serial.print("Connected to android: ");
  // print the central's MAC address:
  Serial.println(central.address());
}

void loopNano33Accelerometer()
{
  float x, y, z;
  while ( ! IMU.accelerationAvailable());

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
  else {
    //Serial.println("Accel data not available");
  }
}
void loopNano33Gyroscope() {
  while ( ! IMU.gyroscopeAvailable());

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  } 
  else {
    //Serial.println("Gyro data not available");
  }
}
void loopNano33Magnetometer() {
  float x, y, z;
  while ( ! IMU.magneticFieldAvailable());

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
  else {
    //Serial.println("Magnet data not available");
  }  
}

void loopBLE()
{
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    digitalWrite(LED_BUILTIN, HIGH);            // turn on the LED to indicate the connection
    String receivedData = "";
    // while the central is still connected to peripheral:
    
    i = 0;
    if (central.connected()) {
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
    else {
      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());  
    }  
  }
}
void loopAD8232ECG() {
    if((digitalRead(10) == 1)||(digitalRead(11) == 1)){
      Serial.println('!');
    }
    else{
      // send the value of analog input 0:
      Serial.println(analogRead(A0));
    }
    //Wait for a bit to keep serial data from saturating
    delay(1);  
}

void setup() {
  Serial.println("This is a new run");  

  Serial.begin(9600);
  while (!Serial);
  
  setupBLE();
  //setupAD8232();
  setupNano33Accelerometer();
  setupNano33Gyroscope();
  setupNano33Magnetometer();
}
void loop() {
  Serial.println("loop again");
  loopBLE();
  loopNano33Accelerometer();
  loopNano33Gyroscope();
  loopNano33Magnetometer();
  //loopAD8232ECG();
}