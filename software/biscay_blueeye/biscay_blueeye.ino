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
#define bias_mon A14
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

//constants 
const double vref = 2.5; 
const double vbias_scale = -15;
const double bias_mon_scale = -(500.0/15.0);
//settings
const double vbias = -31;
const double vmid = 1.65;
const double thresh = 0.025;
const int interval = 5000;

void write_dac(uint32_t data) {
   digitalWrite(dac_sclk, LOW);
   digitalWrite(dac_sync, LOW);
   
   for(int32_t curbit = 23; curbit >= 0; curbit--) {
     digitalWrite(dac_sclk, HIGH);
     digitalWrite(dac_mosi, ((data & (1 << curbit)) != 0));
     delayMicroseconds(10);
     digitalWrite(dac_sclk, LOW);
     delayMicroseconds(10);
   }
   digitalWrite(dac_sync, HIGH);
}

void set_dac_channel_voltage(uint8_t channel, double voltage) {
 uint16_t rawval = 4096 * (voltage / vref);
 if(rawval >= 4096) rawval = 4095;
 Serial.print("rawval === ");
 Serial.println(rawval);
 write_dac((0x01 << 20UL) | ((channel & 0x03) << 17UL) | (rawval << 4UL));
}

double read_channel_averaging(int channel) {
    long sum = 0;
    for(int i = 0; i < 20; i++) {
       sum += analogRead(channel); 
    }
    double val = sum / 20.0;
    return (val * 3.3) / 1023;
}

double read_vbias() {
    double unscaled = read_channel_averaging(bias_mon);
    return unscaled * bias_mon_scale;
}

void read_fpga(uint8_t cspin, uint16_t *buffer, int count) {
   digitalWrite(fpga_sck, LOW);
   delayMicroseconds(2);
   digitalWrite(cspin, LOW);
   for(int curword = 0; curword < count; curword++) {
     buffer[curword] = 0;
     for(int curbit = 15; curbit >= 0; curbit--) {
         if(digitalRead(fpga_miso)==1) {
         buffer[curword] |= (1 << curbit);
        }
        digitalWrite(fpga_sck, HIGH);
        delayMicroseconds(2);

        digitalWrite(fpga_sck, LOW);
        delayMicroseconds(2);
     }
   }
   digitalWrite(cspin, HIGH); 
}

long count = 0;
long lastT = 0;




void setup(){
         pinMode(13,OUTPUT);
         digitalWrite(13,LOW);
         delay(1000);
  Serial.begin(38400);//serial for debugging
  //while(!Serial.available());
//  Serial.read();
  Serial1.begin(9600);//GPS serial...eventually
  Wire.begin();
  sns.initialise();//initialise sensors
  //while(!Serial.available());
  SPI.begin();//radio
  byte my_config[5] = {0x64,0x74,0xFA,0xAC,0xCD};//radio settings
  radio.configure(my_config);
  Serial.println("config1");
  //set up SD card
//  pinMode(sdnss,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
//  digitalWrite(sdnss,HIGH);
 /* if(!SD.begin(sdnss)){
   Serial.println("Initialisation failed.");
  }
  else Serial.println("Initialisation successful.");
  genNewLogFileName();//set up a new log file name in a global var. Sorry, World.
  */
radio.rfm_done = true;
Serial.println("config2");
   pinMode(reg_en, OUTPUT);
   digitalWrite(reg_en, LOW);
   pinMode(dac_sync, OUTPUT);
   digitalWrite(dac_sync, HIGH);
   pinMode(dac_sclk, OUTPUT);
   pinMode(dac_mosi, OUTPUT);
   
   pinMode(fpga_sck, OUTPUT);
   pinMode(fpga_cs0, OUTPUT);
   digitalWrite(fpga_cs0, HIGH);
   pinMode(fpga_miso, INPUT);
   pinMode(fpga_cs1, OUTPUT);
   digitalWrite(fpga_cs1, HIGH);
   
   pinMode(fpga_clr, OUTPUT);
   digitalWrite(fpga_clr, HIGH);
   delay(1);
   Serial.println("config3");
  
   
      pinMode(fpga_captd, INPUT);
   Serial.println("Turning on supplies");
   digitalWrite(reg_en, HIGH);
   Serial.print("Setting bias to ");
   Serial.print(vbias);
   Serial.println("V");
   set_dac_channel_voltage(0, vbias / vbias_scale);
   Serial.print("Setting midpoint to ");
   Serial.print(vmid);
   Serial.println("V");
   set_dac_channel_voltage(2, vmid);
   Serial.print("Setting threshold voltage to ");
   Serial.print(vmid-thresh);
   Serial.println("V");
   set_dac_channel_voltage(1, vmid-thresh);
   set_dac_channel_voltage(0, vbias / vbias_scale);

Serial.println("config4");
     Serial.println("Please wait for Vbias to stabilise.");
  delay(2000);
    double biasVal = read_vbias();
      Serial.print("Vbias=");
      Serial.println(biasVal);
       digitalWrite(fpga_clr, LOW);
       radio.rfm_done = true;
       
       //stuff
Serial.println("config5");
        pinMode(13,OUTPUT);
       digitalWrite(13,HIGH);
       delay(1000);
       digitalWrite(13,LOW);
}

void loop(){
  /*
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
  */
/*    while(!radio.rfm_done);
  radio.endTX();
  send_data(true);*/

        if(digitalRead(fpga_captd)==1) {
          Serial.println("EVENT!!!");
        //uint16_t data[17];
        //read_fpga(fpga_cs0, data, 17);

          /*Serial.print("===");
          for(int i = 0; i < 17; i++) {
           Serial.print(reverse_bits(data[i]));
           if(i!=16) Serial.print(","); 
          }*/

          
          //Serial.println();
          //rf print reverse_bits(data[16])
          /*
          uint16_t rfprint = reverse_bits(data[16]);
          RFMLib::Packet p;
          p.data[0] = (rfprint>>8)&0xFF;
          p.data[1] = (rfprint)&0xFF;
          p.len = 2;
          radio.beginTX(p);
        attachInterrupt(8,RFMISR, RISING);*/
        
          count++;
        

        digitalWrite(fpga_clr, HIGH);
        delayMicroseconds(10);
        digitalWrite(fpga_clr, LOW);
        digitalWrite(13,HIGH);
        delay(200);
        digitalWrite(13,LOW);
        delay(200);        
      }
      

}

void send_data(boolean success){
  RFMLib::Packet p;
  sns.pollMS5637();
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


uint16_t reverse_bits(uint16_t in) {
  uint16_t result = 0;
  for(int cbit = 0; cbit < 16; cbit++) {
     if((in & (1<<cbit)) != 0) {
      result |= (1 << (15 - cbit)); 
     }
  } 
  return result;
}


