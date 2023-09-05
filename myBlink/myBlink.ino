/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>


#define BLUE 4
#define GREEN 5
#define RED 6
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
const int trigPin = 4;
const int echoPin = 2;

// defines variables
long duration;
int distance;

// bluetooth
SoftwareSerial hc06(2,3);
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  Serial.begin(9600); // Starts the serial communication
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  //lcd.print("hello, Amy, Ryan!");

    //Initialize Serial Monitor
  Serial.begin(9600);
  //Serial.println("ENTER AT Commands....:");
  //Initialize Bluetooth Serial Port
  hc06.begin(9600);
  lcd.clear();
  lcd.setCursor(0,0);
}

void turnRed()
{
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
}
void turnBlue()
{
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, HIGH);
}
void turnGreen()
{
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW);
}
void ultraSonic()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  lcd.print("Dist: ");
  lcd.println(distance);

  Serial.print("Distance: ");
  Serial.println(distance);
}
char val[20];//variable to store character from serial.read command 
int c=0;
int r=0;
int R=0;
int i=0;
int totalbytes;
void do_bluetooth() {
  // put your main code here, to run repeatedly:
if(hc06.available())
{
  delay(50);
  totalbytes = hc06.available();
 for(i=0;i<=totalbytes;i++)
  {
    val[i] = hc06.read();
  }
  if(val[1]=='#')
  {
    lcd.clear();
    c=0;
  }
  else
  {
  printonLCD();
  lcd.setCursor(0,1);
  lcd.print("total bytes RX");
  lcd.setCursor(14,1);
  lcd.print(totalbytes);
  }
 /* if((c<=15)&&(r==0))
  {
  lcd.print(val);
  //Serial.print(val);
  c++;
  lcd.setCursor(c,r);
  
  }
  if(c>15)
  {
    lcd.setCursor(R,1);
    lcd.print(val);
    //Serial.print(val);
    R++;
  }
  if(R>15)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    c=0;
    r=0;
    R=0;
  }*/
  }
delay(50);
}
void printonLCD()
{
  for(i=1;i<totalbytes;i++)
  {
    //String tmp = String((char)receivedBytes[0]) + String((char)receivedBytes[1]) + String((char)receivedBytes[2]);
    lcd.setCursor(c,0);
    lcd.print((char)val[i]);
    delay(500);
    c++;
  }
}
void do_bluetooth1() {
  if(hc06.available())
  {
  //val = hc06.read();
  if((c<=15)&&(r==0))
  {
  lcd.write(val);
  Serial.write(val);
  c++;
  lcd.setCursor(c,r);

  }
  if(c>15)
  {
    lcd.setCursor(R,1);
    lcd.write(val);
    R++;
  }
  if(R>15)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    c=0;
    r=0;
    R=0;
  }
  }
  delay(200);
}

void do_bluetooth0() {
  
  if (hc06.available()) {
    char  s = hc06.read();
    lcd.print(s);
    Serial.write(s);
  }
  /*
  if(hc06.read() == '1')
    lcd.print("is 1");
  else
    lcd.print("not 1"); //hc06.read());*/
  delay(200);          
 
  //lcd.print("alive");

  /*
  if (hc06.available()){
    Serial.write(hc06.read());
  }
  
  //Write from Serial Monitor to HC06
  if (Serial.available()){
    hc06.write(Serial.read());
  } */ 
}
// the loop function runs over and over again forever
void loop() {
  ///redValue = 255; // choose a value between 1 and 255 to change the color.
  //greenValue = 0;
  //blueValue = 0;
  turnRed();
  delay(200);                      // wait for a second
  turnBlue();
  delay(200);   
  turnGreen();
                    // wait for a second
  delay(200);                      // wait for a second
  // Turn off the display:
  lcd.noDisplay();
  delay(500);
  // Turn on the display:
  lcd.display();
  delay(200);
  //ultraSonic();
  do_bluetooth();
  //Serial.println("LOOP HERE");
}
