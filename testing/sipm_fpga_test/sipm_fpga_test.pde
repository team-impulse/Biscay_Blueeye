//pin definitions
const int reg_en = 2;
const int dac_sync = 3;
const int dac_sclk = 4;
const int dac_mosi = 5;

const int bias_mon = A0;


const int fpga_sclk = 70;
const int fpga_cs0 = 71;
const int fpga_miso = 72;
const int fpga_cs1 = 73;
const int fpga_clr = 74;
const int fpga_captd = 75;
#define COUNT_IN_PORT PORTE
#define COUNT_IN_MASK (1<<9)

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
   digitalWrite(fpga_sclk, LOW);
   delayMicroseconds(2);
   digitalWrite(cspin, LOW);
   for(int curword = 0; curword < count; curword++) {
     buffer[curword] = 0;
     for(int curbit = 15; curbit >= 0; curbit--) {
         if(digitalRead(fpga_miso)==1) {
         buffer[curword] |= (1 << curbit);
        }
        digitalWrite(fpga_sclk, HIGH);
        delayMicroseconds(2);

        digitalWrite(fpga_sclk, LOW);
        delayMicroseconds(2);
     }
   }
   digitalWrite(cspin, HIGH); 
}

long count = 0;
long lastT = 0;


void setup() {
  Serial.begin(115200);
  
   pinMode(reg_en, OUTPUT);
   digitalWrite(reg_en, LOW);
   pinMode(dac_sync, OUTPUT);
   digitalWrite(dac_sync, HIGH);
   pinMode(dac_sclk, OUTPUT);
   pinMode(dac_mosi, OUTPUT);
   
   pinMode(fpga_sclk, OUTPUT);
   pinMode(fpga_cs0, OUTPUT);
   digitalWrite(fpga_cs0, HIGH);
   pinMode(fpga_miso, INPUT);
   pinMode(fpga_cs1, OUTPUT);
   digitalWrite(fpga_cs1, HIGH);
   
   pinMode(fpga_clr, OUTPUT);
   digitalWrite(fpga_clr, HIGH);
   delay(1);
  
   
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

     Serial.println("Please wait for Vbias to stabilise.");
  delay(20000);
    double biasVal = read_vbias();
      Serial.print("Vbias=");
      Serial.println(biasVal);
       digitalWrite(fpga_clr, LOW);
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

void loop() {
      
      if(digitalRead(fpga_captd)==1) {
        uint16_t data[17];
        read_fpga(fpga_cs0, data, 17);

          Serial.print("===");
          for(int i = 0; i < 17; i++) {
           Serial.print(reverse_bits(data[i]));
           if(i!=16) Serial.print(","); 
          }
          
          Serial.println();
          count++;
        

        digitalWrite(fpga_clr, HIGH);
        delayMicroseconds(10);
        digitalWrite(fpga_clr, LOW);
        
      }


      
      
   /*   double biasVal = read_vbias();
      Serial.print("Vbias=");
      Serial.println(biasVal);
      
      Serial.print("Version=");
      Serial.println(data[1], BIN);
      Serial.print("Pins=");
      Serial.println(data[0], BIN);
      Serial.print("Count=");
      uint32_t count = (data[2] << 16UL) | data[3];
      Serial.println(count, BIN);
      
      Serial.println();

     delay(1000);*/
}
