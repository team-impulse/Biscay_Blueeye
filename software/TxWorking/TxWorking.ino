#include <Wire.h>

#include <TinyGPS++.h>

#include <sensor_library.h>

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

#include <RFM98W_library.h>
RFMLib radio =RFMLib(10,8,255,255);
#define nss 10

SensLib sns;
TinyGPSPlus gps;


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

//============================================================
const byte threshold = 100;
const long transmit_period = 5000;
uint64_t timer;
uint16_t counts[2];//total counts, counts above threshold

//============================================================
void setup(){
  pinMode(13,OUTPUT);
  Serial1.begin(9600);//GPS serial...eventually
  Wire.begin();
  sns.initialise();//initialise sensors

  SPI.begin();
  Serial.begin(38400);
  byte my_config[5] = {
    0x64,0x74,0xFA,0xAC,0xCD  };//radio settings
  radio.configure(my_config);

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
  //   radio.rfm_done = true;
  Serial.println("config5");
  timer = millis();
}

void loop(){
  
  while(Serial1.available())gps.encode(Serial1.read());//feed tinyGPS++ object  
  if((millis()-timer)>transmit_period){
    send_data();
    timer = millis();
    counts[0]=0;
    counts[1]=0;
  }
  if(digitalRead(fpga_captd)==1) {
    Serial.println("EVENT!!!");
    uint16_t data[17];
    read_fpga(fpga_cs0, data, 17);
    if(reverse_bits(data[16])>=threshold)
      counts[1]++;
    counts[0]++;
    /*Serial.print("===");
     for(int i = 0; i < 17; i++) {
     Serial.print(reverse_bits(data[i]));
     if(i!=16) Serial.print(","); 
     }*/


    //Serial.println();
    //rf print reverse_bits(data[16])
    /*    if(radio.rfm_status==0){//if radio idle
     uint16_t rfprint = reverse_bits(data[16]);
     RFMLib::Packet p;
     p.data[0] = (rfprint>>8)&0xFF;
     p.data[1] = (rfprint)&0xFF;
     p.data[2] = (rejected>>8)&0xFF;
     p.data[3] = rejected & 0xFF;
     p.len = 4;
     rejected = 0;
     radio.beginTX(p);
     
     
     attachInterrupt(8,RFMISR, RISING);
     }
     else rejected++;*/


    digitalWrite(13,LOW);
    delay(200);
    digitalWrite(13,HIGH);
    delay(200);
    digitalWrite(13,LOW);

    digitalWrite(fpga_clr, HIGH);
    delayMicroseconds(10);
    digitalWrite(fpga_clr, LOW);
  }


  if(radio.rfm_done){
    Serial.println("Ending");   
    radio.endTX();
  }

}

void send_data(){
  RFMLib::Packet p;
  sns.pollMS5637();
  p.data[0] = sns.pressure>>24;
  p.data[1] = (sns.pressure >> 16) & 0xFF;
  p.data[2] = (sns.pressure>>8) & 0xFF;
  p.data[3] = sns.pressure & 0xFF;
  byte temp = (byte)((int)((sns.internal_temperature/100)+0.5)+(int)128);
  p.data[4] = temp;
  p.data[5] = (counts[0]<<8)&0xFF;
  p.data[6] = (counts[0]&0xFF);
  p.data[7] = (counts[1]<<8)&0xFF;
  p.data[8] = (counts[1]&0xFF);
  double posn_mod = gps.location.lat() - 50;
  uint32_t posn = (uint32_t)(abs(posn_mod) * 10000);
  p.data[9] = (posn>>24)&0xFF;
  p.data[10] = (posn>>16)&0xFF;
  p.data[11] = (posn>>8)&0xFF;
  p.data[12] = posn &0xFF;
  posn_mod = gps.location.lng();
  posn = (uint32_t)(abs(posn_mod) * 10000);
  p.data[13] = (posn>>24)&0xFF;
  p.data[14] = (posn>>16)&0xFF;
  p.data[15] = (posn>>8)&0xFF;
  p.data[16] = posn &0xFF;
  
  p.len = 17;
  radio.beginTX(p);
  Serial.println("begin");

  attachInterrupt(8,RFMISR, RISING);
}

void RFMISR(){
  Serial.println("interrupt");
  radio.rfm_done = true; 
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

