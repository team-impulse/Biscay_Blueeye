
#include <SPI.h>

#include <RFM98W_library.h>
RFMLib radio =RFMLib(3,2,255,255);
#define nss 3

const char *status_lbl[] = {"SD","HDOP","SAT","GPS"};
const int status_lbl_len = 4;

bool sd_error = false;
void setup(){

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64); 
  Serial.begin(38400);
  Serial.println("INIT!");

  
  byte my_config[5] = {0x34,0xC4,0xFA,0xAC,0xCD};//radio settings
  radio.configure(my_config);
  delay(800);
}

void loop(){
  if(radio.rfm_status == 0){
    radio.beginRX(); 
            Serial.println("Starting");   
    attachInterrupt(1,RFMISR,RISING);
  }

  if(radio.rfm_done){
        Serial.println("Ending");

    RFMLib::Packet rx;
    radio.endRX(rx);
    
   if(!rx.crc) {
     Serial.println("CRC FAIL!!!");

   }
   Serial.print("RSSI:");
   Serial.println(rx.rssi);
   Serial.print("SNR:");
   Serial.println(rx.snr);
   if(rx.snr < -15) {
     digitalWrite(29, HIGH);
   }
   Serial.print("Pressure:");
   Serial.print((rx.data[0]<<24UL | rx.data[1]<<16UL | rx.data[2]<<8UL | rx.data[3]));
   Serial.println("hPa");
   Serial.print("Temperature:");
   Serial.print(rx.data[4]-128L);
   Serial.println("degC");
   Serial.print("GPS lat/long: ");
   uint32_t posn = rx.data[9]<<24UL | rx.data[10]<<16UL | rx.data[11]<<8UL | rx.data[12];
   double posn_mod = posn/10000.0;
   posn_mod+=50;
   Serial.print(posn_mod,4);
posn = rx.data[13]<<24UL | rx.data[14]<<16UL | rx.data[15]<<8UL | rx.data[16];
    posn_mod = posn/10000.0;
   posn_mod *=-1;
   Serial.print(", ");
   Serial.println(posn_mod,4);
   
   Serial.print("Total Count:");
   Serial.println(rx.data[5]<<8 | rx.data[6]);
   Serial.print("Thresholded Count:");
   Serial.println(rx.data[7]<<8 | rx.data[8]);
   
      Serial.print("Status: ");

   for(int i = 0; i < status_lbl_len; i++) {
      Serial.print(status_lbl[i]);
      Serial.print(" ");
      if(bitRead(rx.data[17], i) == 1) {
        Serial.print("OK");
      } else {
        Serial.print("FAIL");
      }
      if(i < (status_lbl_len-1))
        Serial.print(", ");
   }
   Serial.println();
      Serial.print("#");
   Serial.print(rx.rssi);
   Serial.print(",");
   Serial.print(rx.snr);
   
   for(int i = 0; i < rx.len; i++) {
     Serial.print(",");
     Serial.print(rx.data[i], 16);
     
   }
   Serial.println();
   Serial.println("=============================\n");   
   Serial.println();

  }
//  Serial.println(rRFM(0x12), 2);
 //     delay(50);
}

byte rRFM(byte ad){//single byte read
   digitalWrite(nss,LOW);
   SPI.transfer(ad & B01111111);//wrn bit low
   byte val = SPI.transfer(0);//read, but we still have to spec a value?
   digitalWrite(nss,HIGH);
   return val;
}
void RFMISR(){
  Serial.println("interrupt");
 radio.rfm_done = true; 
}



