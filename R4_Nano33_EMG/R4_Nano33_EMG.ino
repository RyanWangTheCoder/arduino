// Grove - EMG Sensor demo code
// This demo will need a Grove - Led Bar to show the motion 
// Grove - EMG Sensor connect to A0
// Grove - LED Bar connect to D8, D9
// note: it'll take about serval seconds to detect static analog value
// when you should hold your muscle static. You will see led bar from level 10 turn to 
// level 0, it means static analog value get ok

//#include <LED_Bar.h>

//LED_Bar bar(9, 8);

#include <ArduinoBLE.h>

const int GSR=A3;
const int EMG=A0;

// GSR
int gsrData = 999;
int lastGsrData = 0;

// EMG
int max_analog_dta      = 300;              // max analog data
int min_analog_dta      = 100;              // min analog data
int static_analog_dta   = 0;                // static analog data

int level       = 5;
int level_buf   = 5;
int emgData = 555;
int lastEmgData = 0;

// get analog value
int getEMGAnalog(int pin)
{
    long sum = 0;
    
    for(int i=0; i<32; i++)
    {
        sum += analogRead(pin);
    }
    
    int dta = sum>>5;
    
    max_analog_dta = dta>max_analog_dta ? dta : max_analog_dta;         // if max data
    min_analog_dta = min_analog_dta>dta ? dta : min_analog_dta;         // if min data
    
    return sum>>5;
}

int getGSRAnalog(int pin) {
  int sensorGSRValue=0;
  int gsr_average=0;
  long sum=0;
  for(int i=0;i<10;i++)           //Average the 10 measurements to remove the glitch
  {
    sensorGSRValue=analogRead(pin);
    sum += sensorGSRValue;
    delay(5);
  }
  gsr_average = sum/10;
  return gsr_average;
}


void setupEMG()
{
    Serial.begin(9600);
    
    long sum = 0;

    for(int i=0; i<=10; i++)
    {
        for(int j=0; j<100; j++)
        {
            sum += getEMGAnalog(EMG);
            delay(1);
        }
        
        //bar.setLevel(10-i);
    }
    
    sum /= 1100;
    
    static_analog_dta = sum;

    Serial.print("static_analog_dta = ");
    Serial.println(static_analog_dta);
}
void loopGSR() 
{
  int val = getGSRAnalog(GSR);
  gsrData = val;
  Serial.print("GSR: ");
  Serial.println(gsrData);
}

void loopEMG()
{
    int val = getEMGAnalog(EMG);                    // get Analog value
    
    // int level2;
    
    // if(val>static_analog_dta)                   // larger than static_analog_dta
    // {
    //     level2 = 5 + map(val, static_analog_dta, max_analog_dta, 0, 5);
    // }
    // else 
    // {
    //     level2 = 5 - map(val, min_analog_dta, static_analog_dta, 0, 5);
    // }
    
    // // to smooth the change of led bar
    // if(level2 > level)
    // {
    //     level++;
    // }
    // else if(level2 < level)
    // {
    //     level--;
    // }

    // if(level != level_buf)
    // {
    //     level_buf = level;
    //     //bar.setLevel(level);
    //     //Serial.print("level: ");
    //     //Serial.println(level);
    // }
    Serial.print("EMG data:");
    Serial.println(val);
    emgData = val;
    //delay(10);
}


// Define the name of the Bluetooth peripheral
const char* peripheralName = "R4_RYANW_EMG";

// // Create a BLE Service
// BLEService fileTransferService("19B10000-E8F2-537E-4F6C-D104768A1214");
// // Create a BLE Characteristic for receiving data
// const char* dataCharacteristicUUID = "19B10001-E8F2-537E-4F6C-D104768A1214";
// BLECharacteristic dataCharacteristic(dataCharacteristicUUID, BLEWrite | BLERead, "");
const int characteristicSize = 128;
BLEService stringService("7DEF8317-7300-4EE6-8849-46FACE74CA2A");
BLEStringCharacteristic txCharacteristic("7DEF8317-7301-4EE6-8849-46FACE74CA2A",
    BLERead | BLENotify, characteristicSize);
BLEStringCharacteristic rxCharacteristic("7DEF8317-7302-4EE6-8849-46FACE74CA2A",
    BLEWrite, characteristicSize);

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
  BLE.setAdvertisedService(stringService);

  // Add the characteristics to the service
  stringService.addCharacteristic(txCharacteristic);
  stringService.addCharacteristic(rxCharacteristic);

  BLE.addService(stringService);
  txCharacteristic.writeValue("hello");

  // Start advertising
  BLE.advertise();

  Serial.println("Bluetooth peripheral advertising...");
  
  setupEMG();
}
void RxFromBT() {
  int count;
  String receivedData = "";
  count = rxCharacteristic.valueSize();
  Serial.print("valueLength: ");
  Serial.println(count);
  if (count > 0) {
    //Serial.print("read ");
    //Serial.println(dataCharacteristic.read());
    //dataCharacteristic.readValue()
    receivedData = rxCharacteristic.value();
    // Process the received data (e.g., parse JSON)
    Serial.print("Received data: ");
    Serial.println(receivedData);
  }
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      loopEMG();
      loopGSR();
      RxFromBT();
      if ((lastEmgData != emgData) || (lastGsrData != gsrData)) {
        String txString = String(emgData) + ' ' + String(gsrData);
        txCharacteristic.setValue(txString);
        lastEmgData = emgData;
        lastGsrData = gsrData;
        Serial.println(txString);
      }
    }
  }
}