void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("Pin36:"); Serial.print(analogRead(36)); Serial.print("\t");
  Serial.print("Pin37:"); Serial.print(analogRead(37)); Serial.print("\t");
  Serial.print("Pin39(ENV):"); Serial.print(analogRead(39)); Serial.print("\t");
  Serial.print("Pin32:"); Serial.print(analogRead(32)); Serial.print("\t");
  Serial.print("Pin33:"); Serial.print(analogRead(33)); Serial.print("\t");
  Serial.print("Pin34:"); Serial.print(analogRead(34)); Serial.print("\t");
  Serial.print("Pin35:"); Serial.print(analogRead(35)); Serial.println();
  delay(100);
}