
#include <SdFat.h>


#include <SPI.h>

#include <RFM98W_library.h>
RFMLib radio =RFMLib(27,2,255,255);
#define nss 27
SdFat sd;
bool sd_error = false;
String logFileName;

#define printsdline(a,b) a.print(b);a.print(",");
#define printsdline4(a,b) a.print(b,4);a.print(",");

const char *status_lbl[] = {"SD","HDOP","SAT","GPS"};
const int status_lbl_len = 4;

void setup(){
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  digitalWrite(28, HIGH);
    digitalWrite(29, HIGH);
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64); 
  Serial.begin(38400);
  Serial.println("INIT!");
  
  if (!sd.begin(26, SPI_HALF_SPEED)) {
    Serial.println("SD initialization failed!");
    sd_error = true;
  }
  logFileName = genNewLogFileName("R","millis,RSSI,SNR,p,t,lat,long,count,count_thresh,raw");
  byte my_config[5] = {0x34,0xC4,0xFA,0xAC,0xCD};//radio settings
  radio.configure(my_config);
  delay(800);

  digitalWrite(28, LOW);
  digitalWrite(29, sd_error);
  digitalWrite(4, LOW);
  
  
}

void loop(){
  if(radio.rfm_status == 0){
    radio.beginRX(); 
            Serial.println("Starting");   
    attachInterrupt(1,RFMISR,RISING);
  }

  if(radio.rfm_done){
        Serial.println("Ending");
     digitalWrite(28, HIGH);   
    RFMLib::Packet rx;
    radio.endRX(rx);
    
   if(!rx.crc) {
     Serial.println("CRC FAIL!!!");
     digitalWrite(29, HIGH);
   }

  
  //open file
  
     char lognamechar[logFileName.length()+1];
   logFileName.toCharArray(lognamechar, logFileName.length()+1);
   File logFile =sd.open( lognamechar, FILE_WRITE);
   printsdline(logFile, millis())
   Serial.print("RSSI:");
   Serial.println(rx.rssi);
   printsdline(logFile, rx.rssi)
   Serial.print("SNR:");
   Serial.println(rx.snr);
   printsdline(logFile, rx.snr)
   if(rx.snr < -15) {
     digitalWrite(29, HIGH);
   }
   Serial.print("Pressure:");
   Serial.print((rx.data[0]<<24UL | rx.data[1]<<16UL | rx.data[2]<<8UL | rx.data[3]));
   printsdline(logFile,(rx.data[0]<<24UL | rx.data[1]<<16UL | rx.data[2]<<8UL | rx.data[3]))
   Serial.println("hPa");
   Serial.print("Temperature:");
   Serial.print(rx.data[4]-128L);
   printsdline(logFile,rx.data[4]-128L)
   Serial.println("degC");
   Serial.print("GPS lat/long: ");
   uint32_t posn = rx.data[9]<<24UL | rx.data[10]<<16UL | rx.data[11]<<8UL | rx.data[12];
   double posn_mod = posn/10000.0;
   posn_mod+=50;
   Serial.print(posn_mod,4);
   printsdline4(logFile, posn_mod)
  posn = rx.data[13]<<24UL | rx.data[14]<<16UL | rx.data[15]<<8UL | rx.data[16];
    posn_mod = posn/10000.0;
   posn_mod *=-1;
   printsdline4(logFile, posn_mod)
   Serial.print(", ");
   Serial.println(posn_mod,4);
   
   Serial.print("Total Count:");
   Serial.println(rx.data[5]<<8 | rx.data[6]);
   printsdline(logFile, rx.data[5]<<8 | rx.data[6])
   Serial.print("Thresholded Count:");
   Serial.println(rx.data[7]<<8 | rx.data[8]);
   printsdline(logFile, rx.data[7]<<8 | rx.data[8])
   Serial.print("Status: ");
   bool isOk = true;
   for(int i = 0; i < status_lbl_len; i++) {
      Serial.print(status_lbl[i]);
      Serial.print(" ");
      if(bitRead(rx.data[17], i) == 1) {
        Serial.print("OK");
      } else {
        Serial.print("FAIL");
        isOk = false;
      }
      if(i < (status_lbl_len-1))
        Serial.print(", ");
   }
   digitalWrite(4, isOk);
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
   logFile.println( ByteArrayToStr(rx.data, rx.len));
   logFile.close();
   delay(200);
     digitalWrite(28, LOW);
  digitalWrite(29, sd_error);
  digitalWrite(4, LOW);
  Serial.println("All done!");
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

String genNewLogFileName(String base,String header){
  File lf;
  String ln;
   int16_t logcnt = 0;
   
  while(true){
    ln = base;
   ln = ln + "LOG";
   ln = ln + logcnt;
   ln = ln + ".csv";
   char lognamechar[ln.length()+1];
   ln.toCharArray(lognamechar, ln.length()+1);
   lf =sd.open( lognamechar);
   if(!lf){
     Serial.println("Generated logfile name:");
     Serial.println(ln);
     break;
   }
   lf.close();
   
   logcnt++;
  }//generate a file name that doesn't overwrite anything
  char lognamechar[ln.length()+1];
  ln.toCharArray(lognamechar, ln.length()+1);
  File f = sd.open( lognamechar,FILE_WRITE);
  f.println(header);
  f.close();
  return ln;
}
//convert bytes to a single hex string
String ByteArrayToStr(byte vals[], int count) {
  char buf[count*2+1];
  char *bufptr = buf;
  for(int i = 0; i < count; i++) {
    
    sprintf(bufptr,"%02x",vals[i]);
    bufptr += 2;
  }
  *bufptr = 0;
  return String(buf);
}
