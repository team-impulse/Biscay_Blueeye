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

void loop(){
  if(radio.rfm_status == 0){
    radio.beginRX(); 
    attachInterrupt(7,RFMISR,RISING);
  }

  if(radio.rfm_done){
        Serial.println("Ending");   
    RFMLib::Packet rx;
    radio.endRX(rx);
    Serial.println((rx.data[0]<<8)|rx.data[1]);
    Serial.println((rx.data[2]<<8)|rx.data[3]);
  }
}

void RFMISR(){
  Serial.println("interrupt");
 radio.rfm_done = true; 
}


