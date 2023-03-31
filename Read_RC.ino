///////////////////////////////////////////////READ RC CHANNELS//////////////
void readrc(){
IBus.loop();
ch1 = IBus.readChannel(0);
ch2 = IBus.readChannel(1);
ch3 = IBus.readChannel(2);
ch4 = IBus.readChannel(3);
ch5 = IBus.readChannel(4);
ch6 = IBus.readChannel(5);
ch7 = IBus.readChannel(6);
ch8 = IBus.readChannel(7);
ch9 = IBus.readChannel(8);
ch10 = IBus.readChannel(9);
ch11 = IBus.readChannel(10);
ch12 = IBus.readChannel(11);
ch13 = IBus.readChannel(12);
ch14 = IBus.readChannel(13);


RCThr = ch1;
RCRud = ch2; 
AVOIDMODE = ch10;
MULTI = ch9;
flightmode = ch8;
}
