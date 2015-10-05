#include <Wire.h>

#include <TinyGPS++.h>

#include <sensor_library.h>

#include <SD.h>

#include <SPI.h>

//interface pins
#define fpga_sck 2
#define fpga_cs0 3
#define fpga_miso 5
#define fpga_captd 7
#define fpga_clr 14
#define fpga_cs1 16
#define dac_mosi 17
#define dac_sclk 20
#define dac_sync 21
#define reg_en 22
#define vb_mon A14
#define temp_sense A11
#define accel_interrupt 1


#include <RFM98W_library.h>//radio
RFMLib radio =RFMLib(10,8,255,255);
#define sdnss 15

//Sensor
SensLib sns;
TinyGPSPlus gps;

//SD card structures
File logfile;
String logname = "";

void setup(){
  Serial.begin(38400);//serial for debugging
  Serial1.begin(9600);//GPS serial...eventually
  Wire.begin();
  sns.initialise();//initialise sensors
  //while(!Serial.available());
  SPI.begin();//radio
  byte my_config[5] = {0x64,0x74,0xFA,0xAC,0xCD};//radio settings
  radio.configure(my_config);
  
  //set up SD card
  pinMode(sdnss,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
  digitalWrite(sdnss,HIGH);
  if(!SD.begin(sdnss)){
   Serial.println("Initialisation failed.");
  }
  else Serial.println("Initialisation successful.");
  genNewLogFileName();//set up a new log file name in a global var. Sorry, World.
radio.rfm_done = true;
}

void loop(){
  while(!radio.rfm_done);
  radio.endTX();
  Serial.println("logging pressure:");
  sns.pollMS5637();
  Serial.println(sns.pressure);
  char lognamechar[logname.length()];
  logname.toCharArray(lognamechar, logname.length());
  logfile = SD.open(lognamechar, FILE_WRITE);
  boolean log_success = false;
  if(logfile){//actually logging stuff here
   logfile.print(millis());
   logfile.print(",");
   logfile.print(sns.pressure); 
   logfile.print(",");
   logfile.println(sns.internal_temperature);
   logfile.close();
   log_success = true;
  }
  else {
    Serial.print("Couldn't open log file");
    Serial.println(logname);
  }
  send_data(log_success);
  
  delay(500);
}

void send_data(boolean success){
  RFMLib::Packet p;
  uint32_t t = millis();
  p.data[0] = t >>24;
  p.data[1] = (t>>16) & 0xFF;
  p.data[2] = (t>>8) & 0xFF;
  p.data[3] = t & 0xFF;
  p.data[4] = sns.pressure>>24;
  p.data[5] = (sns.pressure >> 16) & 0xFF;
  p.data[6] = (sns.pressure>>8) & 0xFF;
  p.data[7] = sns.pressure & 0xFF;
  p.data[8] = sns.internal_temperature >>24;
  p.data[9] = (sns.internal_temperature >>16)&0xFF;
  p.data[10] =(sns.internal_temperature >>8)&0xFF;
  p.data[11] = (sns.internal_temperature & 0xFF);
  p.data[12] = (success)? 1 : 0;
  p.len = 13;

  radio.beginTX(p);
  attachInterrupt(8,RFMISR, RISING);
}


void RFMISR(){
   radio.rfm_done = true; 
}

void genNewLogFileName(){
   int16_t logcnt = 0;
  while(true){
    logname = "HAB_";
   logname = logname + "LOG";
   logname = logname + logcnt;
   logname = logname + ".csv\n";
   char lognamechar[logname.length()];
   logname.toCharArray(lognamechar, logname.length());
   logfile = SD.open( lognamechar);
   if(!logfile){
     Serial.println("Generated logfile name:");
     Serial.println(logname);
     break;
   }
   logfile.close();
   
   logcnt++;
  }//generate a file name that doesn't overwrite anything
   
}


