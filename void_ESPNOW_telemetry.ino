void ESPNOW(){
  // Set values to send
  TELEMETRY_DATA.eh1 = val;
  TELEMETRY_DATA.eh2 = angle;
  TELEMETRY_DATA.eh3 = avoidmode;
  TELEMETRY_DATA.eh4 = temp.temperature;
  TELEMETRY_DATA.eh5 = g.gyro.z;
  TELEMETRY_DATA.eh6 = a.acceleration.z;


  
     

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &TELEMETRY_DATA, sizeof(TELEMETRY_DATA));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}
