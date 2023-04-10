/////////////////////////////////////////////////////////////////Measure distance+ move servo


void ReadRADAR(){

 radar.read();
  if(radar.isConnected()){
  if(radar.movingTargetDetected()); { val = radar.movingTargetDistance();}
  pos += dir;
  ESP32_ISR_Servos.setPosition(scanner,(pos*stepAng));
  
  if (val > 40){ distances[pos] = val;}
  else { 
        val = 0;
        distances[pos] = val;
        }
        
  if (pos == numStep)
  {
    dir = -1;
  }
  
  else if (pos == 0)
  {
    dir = 1;
  }
}

 

////////////////////////////////////////LEFT RIGHT AVERAGE        
   // find the left and right average 



 if (pos < (numStep/2) && pos < (numStep/2)+(numStep/4)) {frontleftSum = (((averagedivider*frontleftSum) + distances[pos]) /(averagedivider+1));} 
 if (pos > (numStep/2) && pos > (numStep/2)-(numStep/4)) {frontrightSum = (((averagedivider*frontrightSum) + distances[pos])/(averagedivider+1));}

 if (pos > ((numStep/2) -2) && pos < ((numStep/2) +2)) {frontSum = (((averagedivider*frontSum) + distances[pos])/(averagedivider+1));}


 if (val < minreverse);{
                if (pos < (numStep/2)+(numStep/4));  {leftSum = (((averagedivider* leftSum) + distances[pos])/(averagedivider+1));}
                if (pos > (numStep/2)-(numStep/4)); {rightSum = (((averagedivider*rightSum) + distances[pos])/(averagedivider+1));}
 }
               if (val > minreverse);{
  leftSum = 0;
  rightSum = 0;
 }
    
 turnmulti = map (MULTI, 1000, 2000, 0.1, 10);
 leftsumscaled = (((0.3*leftSum)+ (0.6*frontleftSum) + frontSum) * turnmulti);
 rightsumscaled = (((0.3*rightSum)+ (0.6*frontrightSum) + frontSum)* turnmulti);
 yaw = (int)((rightsumscaled-leftsumscaled)+1500); 


 
  
 
//////////////////////////////////////////FIND TOTAL AVERAGE///////////
  total = total - readings[readIndex];
  readings[readIndex] = val;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }  
 average = total / numReadings;

 //////////////////////////////////////////FIND CLOSEST OBJECT////



  /////////////////////////////////////////LED//
static bool toggle0 = false;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
     digitalWrite(ledPin, ledState);
}
