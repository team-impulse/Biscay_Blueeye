# This Python file uses the following encoding: utf-8
import serial,sys,time,os

#const_default_serial_port = "/dev/tty.usbserial-A6009CSG"
const_default_serial_port = "/dev/tty.Bluetooth-Incoming-Port"
const_default_serial_baud = 38400
const_data_bad_time = 3.0
const_serial_timeout = 0.01
const_lng_mult = -1

def nameLogFile(base):
  file_counter = 0
  file_name = "data/"+base + str(file_counter) + ".CSV"
  while os.path.isfile(file_name):
    file_counter += 1
    file_name =  "data/"+base + str(file_counter) + ".CSV" ##make sure we don't overwrite anything.

  return file_name

def open_serial_port(port_name):#tries to open a specified serial port
    serial_port.port = port_name
    try:
        print("opening",serial_port.port)
        serial_port.timeout = const_serial_timeout #set a timeout for the readline command
        serial_port.open()
        print("Successfully opened serial port.")
        return True
    except (OSError) as e:#if we get a SerialException, the port is
                                                  #probably open already.
        print("Bad serial port name. Try again.")
        print(e)
        return False

def check_serial():
    try:
        return serial_port.readline()
    except:
        return ""

def printData(rx):
    #begin by clearing console
    os.system('cls' if os.name == 'nt' else 'clear')
    print("RX: "+rx)
    print("RSSI:%d, SNR:%d",latest[0],latest[1])
    print("Altitude: %d m",latest[3])
    print("P: %d hPa  |  T: %d °C  |  Lat,long: %d, %d",latest[2],latest[4],latest[5],latest[6])
    print("GPS %s; has %sgot more than three satellites; hdop %s less than 40m.","valid" if last_status[0]  else "invalid","" if last_status[1]  else "not ", "is" if last_status[2]  else "is not")
    print("SD init %ssuccessful.","" if last_status[3]  else "un")
    print("============")
    print("Waiting...")

def processdata(received):
    global latest,QNH
    rawlog = open(logfile_names[0])
    rawlog.write(received)
    rawlog.write("\n")
    rawlog.close()
    proclog = open(logfile_names[1])
    split = received.split(",")
    latest[0]=int(split[0])
    latest[1] = int(split[1])
    latest[2] = (int(split[2],16)<<24 | int(split[3],16)<<16 | int(split[4],16)<<8 | int(split[5],16))/100.0
    #calc altitude here...
    ht = (1-pow((latest[3]/QNH),0.190284))*145366.45
    latest[3] = ht*0.3048 #convert to m


    latest[4] = int(split[6],16)-128
    latest[5] = (int(split[7],16)<<24 | int(split[8],16)<<16 | int(split[9],16)<<8 | int(split[10],16))/10000.0
    latest[6] = (int(split[11],16)<<24 | int(split[12],16)<<16 | int(split[13],16)<<8 | int(split[14],16))/10000.0

    latest[7] = int(split[15],16)<<8 | int(split[16],16)
    latest[8] = int(split[17],16)<<8 | int(split[18],16)
    latest[9]*=const_lng_mult
    for (n,i) in enumerate(latest):
        proclog.write(i)
        if n!=8:
            proclog.write(",")
        else:
            proclog.write("\n")
    proclog.close()
    printData(received)


serial_port = serial.Serial() #instantiate a new Serial port object
serial_port.baudrate = const_default_serial_baud
if not open_serial_port(const_default_serial_port):
    print("bad serial port.")
    sys.exit(0)
logfile_names = [nameLogFile("GND_RAW_"),nameLogFile("GND_PROCESSSED_")]
#to clear serial console os.system('cls' if os.name == 'nt' else 'clear')
QNH = input('Please input QNH in hPa')
latest = [0,  0,  0.0,     0.0,0,      0.0, 0.0, 0,                0]
#rssi,snr,pressure,calc_alt(m),temp,lat,long,total event count,thresholded event count

last_status = [False,False,False,False]#GPS valid, GPS satellites >3, GPS HDOP<40m,SD init Successfully

#output required: rssi(dec),snr(dec),pressure x4 (hex),temp x1(hex),count[0]x2 (hex),count[1]x2(hex),positionx8 (hex),status (hex)
pktcount = 0
print "Waiting..."
while True:
    rec = check_serial()
    if rec!="":#if we've got something
        pktcount+=1
        processdata(rec)
