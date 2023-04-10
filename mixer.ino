void Mixer (){
  
if (stabyawen ==1){
stabyaw = map (angle, -100, 100, 1000, 2000);
target = map (rudout,1000,2000,-20,20);
rudservoout = (stabyaw + rudout)/2;
}
 
if (stabyawen ==0){
stabyaw = 1500;
target = 0;
rudservoout = rudout;
}

outch1 = map(out, 1000, 2000, 0, 180);
outch2 = map(rudservoout, 1000, 2000, 0, 180);
outch3 = map(flightmode, 1000, 2000, 0, 180);
outch4 = ch4;
outch5 = ch5;
outch6 = ch6;
outch7 = ch7;
outch8 = ch8;
outch9 = ch9;
outch10 = ch10;
outch11 = ch11;
outch12 = ch12;
outch13 = ch13;
outch14 = ch14;

}
