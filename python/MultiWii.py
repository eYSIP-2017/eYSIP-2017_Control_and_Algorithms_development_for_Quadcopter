import serial, time, struct

class MultiWii:

    """Multiwii Serial Protocol message ID"""
    # FC -->
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    SERVO_CONF = 120

    # --> FC
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SELECT_SETTING = 210
    SET_HEAD = 211
    SET_SERVO_CONF = 212
    SET_MOTOR = 214
    BIND = 240
    SET_LED = 244
    EEPROM_WRITE = 250

    def __init__(self):

        # Global variables of data
        self.rx_PID = {'rp':0,'ri':0,'rd':0,'pp':0,'pi':0,'pd':0,'yp':0,'yi':0,'yd':0,'ap':0,'ai':0,'ad':0}
        self.tx_PID = [0]*12
        self.rx_rcData = {'roll':0,'pitch':0,'yaw':0,'throttle':0}
        self.tx_rcData = [0]*4
        self.rx_rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'mx':0,'my':0,'mz':0}
        self.rx_motor = {'m1':0,'m2':0,'m3':0,'m4':0}
        self.tx_motor = [0]*4
        self.rx_attitude = {'angx':0,'angy':0,'heading':0}
        self.rx_altitude = {'estalt':0,'vario':0}
        self.elapsed = 0
        self.PRINT = 1

    def start(self, serPort):
        # Configure serial port
        self.ser = serial.Serial()
        self.ser.port = serPort
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2
        
        # Time to wait until the board becomes operational
        wakeup = 2
        try:
            self.ser.open()
            if self.PRINT:
                print "\n>>> Opening COM Port: "+self.ser.port
            for i in range(1,wakeup):
                if self.PRINT:
                    time.sleep(1)
                else:
                    time.sleep(1)
                    
        except Exception as error:
            print error
            self.ser.close()

    def generateFrame(self, code, data, data_format, mode='tx', crc=0):
        checksum = 0
        frame = ''
        direc = {'tx':'<', 'rx':'>'}

        # Pack data into bytes
        header = struct.pack('ccc','$', 'M', direc[mode])
        code = struct.pack('B', code)
        data_bytes = struct.pack(data_format, *data)
        data_length = struct.pack('B',len(data_bytes))
        payload = data_length + code + data_bytes

        # Calculate checksum
        if crc:
            for byte in payload: checksum += ord(byte)
            checksum = struct.pack('B', (0xFF - (0xFF & checksum)))
        else:
            for byte in payload: checksum ^= ord(byte)
            checksum = struct.pack('B', checksum)
            
        frame = header + payload + checksum

        return frame

    """Function for sending a command to the board"""
    def sendFrame(self, code, data, data_format, mode='tx'):
        # Make frame
        tx_frame = self.generateFrame(code, data, data_format, mode)
        
        # Send data
        try:
            self.ser.write(tx_frame)
            return tx_frame

        except Exception as error:
            print error

    def receiveData(self):
        try:
            if self.ser.inWaiting():
                #if self.ser.read(1):
                print
                for i in (self.ser.read(self.ser.inWaiting())):
                    print ord(i),
                    
        except Exception as error:
            print error

    def receivePacket(self, data_format, data_length):
        payload = ''
        rx_data = []

        time.sleep(0.5)

        try:
            if (self.ser.inWaiting() >= data_length):
                payload += self.ser.read(data_length)

                # Unpack data
                rx_data = list(struct.unpack(data_format, payload))
                #for i in rx_data: print i,

                # Clear buffer
                self.ser.flushInput()
                self.ser.flushOutput()
                
                return rx_data

        except Exception as error:
            print error

    def receiveRaw(self):
        payload = ''
        rx_data = []

        time.sleep(0.5)

        try:
            if (self.ser.inWaiting()):
                payload += self.ser.read(self.ser.inWaiting())

                # Print data
                print payload
                return payload

        except Exception as error:
            print error
                
    """Function to receive a data packet from the board"""
    def receiveFrame(self, code, data_format, data_length, crc=1):
        checksum = 0
        calcsum = 0
        payload = ''
        rx_data = []

        # Recieve data
        try:
            if self.ser.inWaiting() > (data_length + 5):
                temp = []
                for i in (self.ser.read(12)): temp.append(ord(i))
                print temp
                return None
                # MSP header
                if (self.ser.read(1) != '$'): return None
                if (self.ser.read(1) != 'M'): return None
                if (self.ser.read(1) != '>'): return None

                # Data length
                data = self.ser.read(1)
                if (ord(data) != data_length): return None
                else: payload += data

                # MSP code
                data = self.ser.read(1)
                if (ord(data) != code): return None
                else: payload += data

                payload += self.ser.read(data_length)
                checksum = self.ser.read(1)

                # Clear buffer
                self.ser.flushInput()
                self.ser.flushOutput()

                # Verify checksum
                if crc:
                    for byte in payload:
                        calcsum += ord(byte)                 
                    if (0xFF - (calcsum & 0xFF)) != ord(checksum): return None
                else:
                    for byte in payload:
                        calcsum ^= ord(byte)                 
                    if calcsum != ord(checksum): return None

                # Unpack data
                rx_data = list(struct.unpack(data_format, payload[2:len(payload)]))

                # Parse data
                return rx_data
            
        except Exception as error:
            print error

    def parseFrame(self, code, data):
        try:
            if code == MultiWii.ATTITUDE:
                self.rx_attitude['angx']=float(temp[0]/10.0)
                self.rx_attitude['angy']=float(temp[1]/10.0)
                self.rx_attitude['heading']=float(temp[2])
                #self.rx_attitude['elapsed']=round(elapsed,3)
                #self.rx_attitude['timestamp']="%0.2f" % (time.time(),) 
                return self.rx_attitude
            
            elif code == MultiWii.ALTITUDE:
                self.rx_altitude['estalt']=float(temp[0])
                self.rx_altitude['vario']=float(temp[1])
                #self.rx_altitude['elapsed']=round(elapsed,3)
                #self.rx_altitude['timestamp']="%0.2f" % (time.time(),) 
                return self.rx_altitude
            
            elif code == MultiWii.RC:
                self.rx_rcData['roll']=temp[0]
                self.rx_rcData['pitch']=temp[1]
                self.rx_rcData['yaw']=temp[2]
                self.rx_rcData['throttle']=temp[3]
                #self.rx_rcData['elapsed']=round(elapsed,3)
                #self.rx_rcData['timestamp']="%0.2f" % (time.time(),)
                return self.get_rcData
            
            elif code == MultiWii.RAW_IMU:
                self.rx_rawIMU['ax']=float(temp[0])
                self.rx_rawIMU['ay']=float(temp[1])
                self.rx_rawIMU['az']=float(temp[2])
                self.rx_rawIMU['gx']=float(temp[3])
                self.rx_rawIMU['gy']=float(temp[4])
                self.rx_rawIMU['gz']=float(temp[5])
                self.rx_rawIMU['mx']=float(temp[6])
                self.rx_rawIMU['my']=float(temp[7])
                self.rx_rawIMU['mz']=float(temp[8])
                #self.rx_rawIMU['elapsed']=round(elapsed,3)
                #self.rx_rawIMU['timestamp']="%0.2f" % (time.time(),)
                return self.rx_rawIMU
            
            elif code == MultiWii.MOTOR:
                self.rx_motor['m1']=float(temp[0])
                self.rx_motor['m2']=float(temp[1])
                self.rx_motor['m3']=float(temp[2])
                self.rx_motor['m4']=float(temp[3])
                #self.get_motor['elapsed']="%0.3f" % (elapsed,)
                #self.get_motor['timestamp']="%0.2f" % (time.time(),)
                return self.get_motor
            
            elif code == MultiWii.PID:
                dataPID=[]
                if len(temp)>1:
                    d=0
                    for t in temp:
                        dataPID.append(t%256)
                        dataPID.append(t/256)
                    for p in [0,3,6,9]:
                        dataPID[p]=dataPID[p]/10.0
                        dataPID[p+1]=dataPID[p+1]/1000.0
                    self.rx_PID['rp']= dataPID=[0]
                    self.rx_PID['ri']= dataPID=[1]
                    self.rx_PID['rd']= dataPID=[2]
                    self.rx_PID['pp']= dataPID=[3]
                    self.rx_PID['pi']= dataPID=[4]
                    self.rx_PID['pd']= dataPID=[5]
                    self.rx_PID['yp']= dataPID=[6]
                    self.rx_PID['yi']= dataPID=[7]
                    self.rx_PID['yd']= dataPID=[8]
                return self.get_PID
            
            else:
                return None

        except Exception as error:
            print error

    def requestFrame(self, code):
        tx_frame = struct.pack('cccBBB','$', 'M', '<', 0, code, code)
        
        # Send data
        try:
            self.ser.write(tx_frame)
            print '\n\n>>> Request Frame:'
            for i in tx_frame: print ord(i),
            return tx_frame

        except Exception as error:
            print error

    def close(self):
        self.ser.close()

'''drone = MultiWii()
drone.start('COM2')
##drone.requestFrame(101)

while(1):
    try:
        frames = drone.sendFrame(108, [-250, 140, 45], '<hhh', mode='rx')
        for i in frames: print ord(i),
        print

#a = [36, 77, 62, 7, 100, 231, 3, 0, 4, 0, 0, 0]
#for i in a: print hex(i),

        time.sleep(1)

    except KeyboardInterrupt:
        print '\n>>> Closing'
        break
##drone.receiveData()
drone.close()'''
