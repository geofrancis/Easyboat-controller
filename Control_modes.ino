/////////////////////////////////////////////////CONTROL MODES///////
void controlmodes(){

  

// AVOID AVERAGE LEFT + RIGHT----------------------------------------------
   // find the left and right average sum
 
if (yawsmoothen == 1)
  { 
    if (average < turnrange)
  { 

if (yaw > 1800){yaw = 1800;}
if (yaw < 1200) {yaw = 1200;}

  }
  else {(yaw = RCRud);
  }

 yawsmooth = LIDARPIDOUTPUT;
 
  }
// AVERAGE THROTTLE AVOID-----------------------------------------------

if (escsmoothen == 1)
  {
    average = total / numReadings;
    if (average < minreverse)
  {  
    esc = map (average, minreverse, fullreverse, 1500, 900);
  }
    else{
      esc = RCThr;
        }  
        

if (esc > 2000){esc = 2000;}
if (esc < 1000) {esc = 1000;}   
  
 escsmooth = esc;
  }
//AVOID POINT DIRECTION  -----------------------------------

if (pointyawen == 1)
  {
  if (average < turnrange)
  {  
   if (avoiddirection < numStep/2){
      avoidturn =0;
    }
    else avoidturn = 1;
      
   if (avoidturn = 0) {
       pointyaw = map (avoiddirection, 0, (numStep/2), 1500, 1000);}
  if (avoidturn = 1) 
      {pointyaw = map (avoiddirection, (numStep/2), (numStep), 1500, 2000);}
  }
    else{
      pointyaw = RCRud;
        }

  }   
// POINT THROTTLE AVOID-----------------------------------------------
 if (pointescen == 1)
  {
 if (closest < minreverse)
  {  
 pointesc = map (closest, minreverse, fullreverse, 1500, 900);
  }
    else{
    esc = RCThr;
        }          
  }

//POINT YAW TO FOLLOW CLOSEST OBJECT----------------------------------------------

if (pointyawen == 1)
  {
if (average < turnrange){
if (avoidturn = 0) 
   followturn = map (avoiddirection, 0, (numStep/2), 1500, 1700);}
  else {followturn = map (avoiddirection, (numStep/2), numStep, 1700, 1500);}       
  }


//AVERAGE YAW TO FOLLOW -----------------------------------------------------

if (followturnen == 1)
  {
 if (average < turnrange)
  { 
    yawfollow = (int)((leftsumscaled - rightsumscaled)+1500); 
  }
  else {(yawfollow = RCRud);
  }
 for (int i=0; i < 40; i++) {
 yawsmooth = yawsmooth + yawfollow;
 }
 yawsmooth = yawsmooth/40;
  }

  
//SCALE YAW TO FOLLOW WALL----------------------------------------------------------

if ( wallsteeren == 1);
{
if (leftwallaverage<=rightwallaverage){
wallsteer = map (leftsumscaled, 2000, 1500,  1400, 1600);
}
if (rightwallaverage<leftwallaverage){
  wallsteer = map (rightsumscaled, 2000, 1500,  1600, 1400);
}
if (wallsteer > 1600){wallsteer = 1600;}
if (wallsteer < 1400) {wallsteer = 1400;}
}
}
