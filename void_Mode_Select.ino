//////////////////////////////////////////////////AVOID MODE SELECTION/////////
void modeselect(){


if (AVOIDMODE <= 1000) {avoidmode = 0;}
if (AVOIDMODE > 1001 && AVOIDMODE < 1100) {avoidmode = 1;}
if (AVOIDMODE > 1101 && AVOIDMODE < 1200) {avoidmode = 2;}
if (AVOIDMODE > 1201 && AVOIDMODE < 1300) {avoidmode = 3;}
if (AVOIDMODE > 1301 && AVOIDMODE < 1400) {avoidmode = 4;}  
if (AVOIDMODE > 1401 && AVOIDMODE < 1500) {avoidmode = 5;}  
if (AVOIDMODE > 1501 && AVOIDMODE < 1600) {avoidmode = 6;}  
if (AVOIDMODE > 1601 && AVOIDMODE < 1700) {avoidmode = 7;}  
if (AVOIDMODE > 1701 && AVOIDMODE < 1800) {avoidmode = 8;}  
if (AVOIDMODE > 1801 && AVOIDMODE < 1900) {avoidmode = 9;}  
if(AVOIDMODE > 1901 && AVOIDMODE < 2000) {avoidmode = 10;}
if (AVOIDMODE >= 2000) {avoidmode = 11;}
      

}
