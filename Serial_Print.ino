void serialprint (){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;


Serial.print("RUD: ");
        Serial.print(RUD);
        Serial.print("\t  MOT: ");
        Serial.print(MOT);
        Serial.print("\t  rightsumscaled: ");
        Serial.print(rightsumscaled);
        Serial.print("\t  leftsum: ");
        Serial.print(leftSum);
        Serial.print("\t  average: ");
        Serial.print(average);
        Serial.print("\t  turnmulti ");
        Serial.print(turnmulti);
        Serial.print("\t  avoidmode: ");
        Serial.print(avoidmode);
        Serial.print("\t  avoiddirection: ");
        Serial.println(avoiddirection);
}
}
