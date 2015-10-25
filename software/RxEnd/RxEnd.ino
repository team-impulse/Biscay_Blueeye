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
   Serial.print("RSSI:");
   Serial.println(rx.rssi);
   Serial.print("SNR:");
   Serial.println(rx.snr);
   Serial.print("Pressure:");
   Serial.print((rx.data[0]<<24 | rx.data[1]<<16 | rx.data[2]<<8 | rx.data[3]));
   Serial.println("hPa");
   Serial.print("Temperature:");
   Serial.print(rx.data[4]-128);
   Serial.println("degC");
   Serial.print("GPS lat/long: ");
   uint32_t posn = rx.data[9]<<24 | rx.data[10]<<16 | rx.data[11]<<8 | rx.data[12];
   double posn_mod = posn/10000.0;
   posn_mod+=50;
   Serial.print(posn_mod,4);
posn = rx.data[13]<<24 | rx.data[14]<<16 | rx.data[15]<<8 | rx.data[16];
    posn_mod = posn/10000.0;
   posn_mod *=-1;
   Serial.print(", ");
   Serial.println(posn_mod,4);
   
   Serial.print("Total Count:");
   Serial.println(rx.data[5]<<8 | rx.data[6]);
   Serial.print("Thresholded Count:");
   Serial.println(rx.data[7]<<8 | rx.data[8]);
   Serial.println("=============================\n");   
  }
  
}

void RFMISR(){
  Serial.println("interrupt");
 radio.rfm_done = true; 
}


