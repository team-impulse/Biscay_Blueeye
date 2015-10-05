#include <SPI.h>

#include <RFM98W_library.h>
RFMLib radio =RFMLib(20,7,16,21);
#define nss 20
void setup(){
  SPI.begin();
  Serial.begin(38400);
  byte my_config[5] = {0x64,0x74,0xFA,0xAC,0xCD};//radio settings
  radio.configure(my_config);
}
void handlePacket(RFMLib::Packet &p);
void loop(){
  if(radio.rfm_status == 0){
    radio.beginRX(); 
    attachInterrupt(7,RFMISR,RISING);
  }

  if(radio.rfm_done && radio.rfm_status==2){
    RFMLib::Packet rx;
    radio.endRX(rx);
    handlePacket(rx);
    transmitMaybe();
  }
  
  if(radio.rfm_done && radio.rfm_status==1){
    radio.endTX();  
    transmitMaybe();
  }
}

void RFMISR(){
  radio.rfm_done = true; 
}

void transmitMaybe(){
  RFMLib::Packet t;
  t.len = 0;
  while(Serial.available()){
    t.data[t.len] = Serial.read();
    t.len++;
  }
  if(t.len != 0){
    radio.beginTX(t);
    attachInterrupt(7,RFMISR,RISING);
  }
}

void handlePacket(RFMLib::Packet &p){
  Serial.print((p.crc)?"PASS," : "PASS,");//CRC
  Serial.print(p.rssi);
  Serial.print(",");
  Serial.print(p.snr);
  Serial.print(",");
  for(int i = 0;i<p.len-1;i++){
    Serial.print(p.data[i]);
    Serial.print(",");
  }
  Serial.println(p.data[p.len-1]);
}

