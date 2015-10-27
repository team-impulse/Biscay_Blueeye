# This Python file uses the following encoding: utf-8
import serial,sys,time,os

const_default_serial_port = "/dev/tty.usbserial-A6009CSG"
#const_default_serial_port = "/dev/tty.Bluetooth-Incoming-Port"
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

def processdata(received):
    rawlog = open(logfile_names[0],'a')
    rawlog.write(received)
    rawlog.write("\n")
    rawlog.close()
    print(received)


serial_port = serial.Serial() #instantiate a new Serial port object
serial_port.baudrate = const_default_serial_baud
if not open_serial_port(const_default_serial_port):
    print("bad serial port.")
    sys.exit(0)
logfile_names = [nameLogFile("GND_RAW_"),nameLogFile("GND_PROCESSSED_")]
#to clear serial console os.system('cls' if os.name == 'nt' else 'clear')


#output required: rssi(dec),snr(dec),pressure x4 (hex),temp x1(hex),count[0]x2 (hex),count[1]x2(hex),positionx8 (hex),status (hex)
pktcount = 0
while True:
    rec = check_serial()
    if rec!="":#if we've got something
        processdata(rec)
