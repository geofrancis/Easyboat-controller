///////////////////////////////////////////////////CONTROL MODE SELECTION////
void avoidmodes(){

 if (avoidmode == 0) {
  
 yawsmoothen = 0;
 escsmoothen = 0;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;

   out = RCThr;
   rudout = RCRud;
}

//average steering only
 if (avoidmode == 1) {
  
 yawsmoothen = 1;
 escsmoothen = 0;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;
  
   rudout = ((yawsmooth + RCRud)/2);
   out = RCThr;
}
//throttle
if (avoidmode == 2) {
  
 yawsmoothen = 0;
 escsmoothen = 1;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;
 
   out = ((escsmooth + RCThr)/2);
   rudout = RCRud;
}  
//steering and throttle
if (avoidmode == 3) {
 yawsmoothen = 1;
 escsmoothen = 1;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;

   out = ((escsmooth + RCThr)/2);
   rudout = ((yawsmooth + RCRud)/2);

}  
//point steering only
if (avoidmode == 4) {
 yawsmoothen = 0;
 escsmoothen = 0;
 pointyawen = 1; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;

   rudout = ((pointyaw + RCRud)/2);
   out = RCThr;
}
//point throttle
if (avoidmode == 5) {
 yawsmoothen = 0;
 escsmoothen = 0;
 pointyawen = 0; 
 pointescen = 1;
 wallsteeren = 0;
 followturnen = 0;
 
   out = ((pointesc + RCThr)/2);
   rudout = RCRud;
}



//wall following
if (avoidmode == 6) {
 yawsmoothen = 0;
 escsmoothen = 1;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 1;
 followturnen = 0;
out = ((esc + RCThr)/2);
rudout = ((yaw + RCRud + wallsteer)/3);
}  

//object following
if (avoidmode == 7) {
 yawsmoothen = 1;
 escsmoothen = 1;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 1;
 out = ((esc + RCThr)/2);
 rudout = ((yaw + RCRud + followturn)/3);
}  


  
}
