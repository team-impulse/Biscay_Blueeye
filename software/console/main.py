# This Python file uses the following encoding: utf-8
import serial,sys,time,os

#const_default_serial_port = "/dev/tty.usbserial-A6009CSG"
const_default_serial_port = "/dev/tty.usbserial-A6009CSG"
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
  fx = open(file_name,'w')
  fx.close()
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
    print("RSSI:{0}, SNR:{1}".format(latest[0],latest[1]))
    print("Altitude: {0} m".format(latest[3]))
    print("P: {0} hPa  |  T: {1} °C  |  Lat,long: {2}, {3}".format(latest[2],latest[4],latest[5],latest[6]))
    print("Total counts: {0} | Thresholded counts: {1}".format(latest[7],latest[8]))
    print("GPS {0}; has {1}got more than three satellites; hdop {2} less than 40m.".format("valid" if last_status[0]  else "invalid","" if last_status[1]  else "not ", "is" if last_status[2]  else "is not"))
    print("SD init {0}successful.".format("" if last_status[3]  else "un"))
    print("============")
    print("Waiting...")

def processdata(received):
    global latest,QNH
    rawlog = open(logfile_names[0],'a')
    rawlog.write(received)
    rawlog.write("\n")
    rawlog.close()
    proclog = open(logfile_names[1],'a')
    split = received.split(",")
    try:
        latest[0]=int(split[0])
        latest[1] = int(split[1])
        latest[2] = (int(split[2],16)<<24 | int(split[3],16)<<16 | int(split[4],16)<<8 | int(split[5],16))/100.0
        #calc altitude here...
        ht = (1-pow((latest[2]/QNH),0.190284))*145366.45
        latest[3] = ht*0.3048 #convert to m


        latest[4] = int(split[6],16)-128

        latest[5] = (int(split[7],16)<<24)|(int(split[8],16)<<16)|(int(split[9],16)<<8)|(int(split[10],16))
        latest[6] = (int(split[11],16)<<24)|(int(split[12],16)<<16)|(int(split[13],16)<<8)|(int(split[14],16))
        latest[6]*=const_lng_mult
        latest[7] = int(split[15],16)<<8 | int(split[16],16)
        latest[8] = int(split[17],16)<<8 | int(split[18],16)

        last_status[3]=True if (int(split[19])&0x1) == 1 else False
        last_status[2] = True if (int(split[19])&0x2)>>1 ==1 else False
        last_status[1] = True if (int(split[19])&0x4)>>2==1 else False
        last_status[0] = True if (int(split[19])&0x8)>>3==1 else False

    except IndexError:
        print "yuck"
    for (n,i) in enumerate(latest):
        proclog.write(str(i))
        if n!=8:
            proclog.write(",")
        else:
            proclog.write("\n")
    proclog.close()
    printData(received)


serial_port = serial.Serial() #instantiate a new Serial port object
serial_port.baudrate = const_default_serial_baud
#if not open_serial_port(const_default_serial_port):
#    print("bad serial port.")
#    sys.exit(0)
logfile_names = [nameLogFile("GND_RAW_"),nameLogFile("GND_PROCESSSED_")]
#to clear serial console os.system('cls' if os.name == 'nt' else 'clear')
QNH = input('Please input QNH in hPa')
latest = [0,  0,  0.0,     0.0,0,      0.0, 0.0, 0,                0]
#rssi,snr,pressure,calc_alt(m),temp,lat,long,total event count,thresholded event count

last_status = [False,False,False,False]#GPS valid, GPS satellites >3, GPS HDOP<40m,SD init Successfully

#output required: rssi(dec),snr(dec),pressure x4 (hex),temp x1(hex),count[0]x2 (hex),count[1]x2(hex),positionx8 (hex),status (hex)
pktcount = 0

while True:
    time.sleep(4)
    rec = "#-88,9,0,1,89,4F,95,0,3,0,3,0,7,A1,20,0,0,0,0,9"
    #rec = check_serial()
    if rec!="" and rec[0]=="#":#if we've got something
        pktcount+=1
        rec = rec[1:]
        print("====")
        print rec
        print "===="
        processdata(rec)
