import time
from Tkinter import *
from MultiWii import MultiWii
from PIL import Image, ImageTk

drone = MultiWii()

AXIS = 0
ARMED = 0
COM_OPEN = 0
pid_index = [0, 1, 2]
set_point_list = [0, 0, 0, 0]

msp_rxf_set_rc = [10, 10, 180, 0, 0, 0, 100, 100]
msp_rxf_set_pid = [38, 4, 90, 45, 4, 90, 0, 0, 0, 0, 0, 0]

def mapfloat(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def printf(frame, mode=1):
    print
    for i in frame:
      if not mode: print hex(ord(i)).split('0x')[1].upper().zfill(2),
      else: print ord(i),
    print

def toggleCOM():
    global COM_OPEN
    text = ['Close COM Port', 'Open COM Port']
    if not COM_OPEN:
      COM_OPEN = 1
      buttonOpen.configure(bg="green", fg="black", text=text[0])
      drone.start('COM12')
    else:
      COM_OPEN = 0
      print '\n>>> Closing COM Port: ' + drone.ser.port
      buttonOpen.configure(bg="red", fg="white", text=text[1])
      drone.close()

def set_pid_index(val):
    global pid_index, AXIS
    axis_text = ['Roll', 'Pitch', 'Yaw', 'Altitude']

    sel1()
    sel2()
    sel3()
    
    if val is 'Roll':
      pid_index = [0, 1, 2]
      AXIS = 0
    elif val is 'Pitch':
      pid_index = [3, 4, 5]
      AXIS = 1
    elif val is 'Yaw':
      pid_index = [6, 7, 8]
      AXIS = 2
    elif val is 'Altitude':
      pid_index = [9, 10, 11]
      AXIS = 3

    text = "%s Set Point = %.1f"%(axis_text[AXIS], set_point_list[AXIS])
    label5.config(text=text)
    scale5.set(set_point_list[AXIS])
    
    label1.config(text=str(int(msp_rxf_set_pid[pid_index[0]])))
    label2.config(text=str(int(msp_rxf_set_pid[pid_index[1]])))
    label3.config(text=str(int(msp_rxf_set_pid[pid_index[2]])))
    
    scale1.set(int(msp_rxf_set_pid[pid_index[0]]))
    scale2.set(int(msp_rxf_set_pid[pid_index[1]]))
    scale3.set(int(msp_rxf_set_pid[pid_index[2]]))

def sel1():
    global pid_index
    selection = "KP = " + str(int(var1.get()))
    label1.config(text = selection)
    msp_rxf_set_pid[pid_index[0]] = int(var1.get())

def sel2():
    global pid_index
    selection = "KI = " + str(int(var2.get()))
    label2.config(text = selection)
    msp_rxf_set_pid[pid_index[1]] = int(var2.get())

def sel3():
    global pid_index
    selection = "KD = " + str(int(var3.get()))
    label3.config(text = selection)
    msp_rxf_set_pid[pid_index[2]] = int(var3.get())

def get_ident():
    drone.requestFrame(drone.IDENT)
    time.sleep(0.5)
    drone.receiveData()

def get_attitude():
    drone.requestFrame(drone.ATTITUDE)
    time.sleep(0.5)
    d = drone.receivePacket('<3h', 6)
    if d:
        print '\nPitch:\t%.0f'%(float(d[1])/10.0)
        print 'Roll:\t%.0f'%(float(d[0])/10.0)
        print 'Yaw:\t%.0f'%(float(d[2]))

def get_raw_imu():
    drone.requestFrame(drone.RAW_IMU)
    time.sleep(0.5)
    drone.receiveData()

def get_rc():
    drone.requestFrame(drone.RC)
    time.sleep(0.5)
    drone.receiveData()

def get_motor():
    drone.requestFrame(drone.MOTOR)
    time.sleep(0.5)
    drone.receiveData()

def send_pid():
    global msp_rxf_set_pid, AXIS
    axis_text = ['Roll', 'Pitch', 'Yaw', 'Altitude']
    print '\n>>> Send SET_PID'
    sel1()
    sel2()
    sel3()
    for i in msp_rxf_set_pid: print i,
    printf(drone.sendFrame(drone.SET_PID, msp_rxf_set_pid+[0]*18, '<30B'))
    d = drone.receivePacket('<16f', 64)
    if d: print '\n<<< PID Set Successfully!'
    print'PITCH --> Set Point: %.1f\nP:\t%.1f\nI:\t%.3f\nD:\t%.2f'%(d[0], d[1], d[2], d[3])
    print'ROLL  --> Set Point: %.1f\nP:\t%.1f\nI:\t%.3f\nD:\t%.2f'%(d[4], d[5], d[6], d[7])
    print'YAW   --> Set Point: %.1f\nP:\t%.1f\nI:\t%.3f\nD:\t%.2f'%(d[8], d[9], d[10], d[11])
    print'ALT   --> Set Point: %.1f\nP:\t%.1f\nI:\t%.3f\nD:\t%.2f'%(d[12], d[13], d[14], d[15])

def send_rc():
    global msp_rxf_set_rc
    print '\n>>> Send SET_RC'
    for i in msp_rxf_set_rc: print i,
    printf(drone.sendFrame(drone.SET_RAW_RC, [10, 10, 180] + msp_rxf_set_rc[3:8], '<3h5H'))

def set_throttle():
    global msp_rxf_set_rc
    selection = "Throttle = " + str(var4.get())
    label4.config(text = selection)
    msp_rxf_set_rc[3] = int(var4.get())
    send_rc()

def set_target():
    global msp_rxf_set_rc, AXIS, set_point_list
    axis_text = ['Roll', 'Pitch', 'Yaw', 'Altitude']
    set_point_list[AXIS] = var5.get()
    selection = "%s Set Point = %s"%(axis_text[AXIS], str(var5.get()))
    label5.config(text = selection)
    #msp_rxf_set_rc[AXIS] = int(var5.get())
    if AXIS is 1: msp_rxf_set_rc[6] = int((float(var5.get()) + 10) * 10)
    elif AXIS is 0: msp_rxf_set_rc[7] = int((float(var5.get()) + 10) * 10)
    send_rc()

def arm():
    global ARMED, msp_rxf_set_rc
    text = ['ARM DRONE', 'DISARM DRONE']
    if ARMED:
      ARMED = 0
      msp_rxf_set_rc[4] = 0
      button10.configure(bg="red", fg="white", text=text[0])
    else:
      ARMED = 1
      msp_rxf_set_rc[4] = 2000
      button10.configure(bg="green", fg="black", text=text[1])
    send_rc()
    print '\n\n>>> %s <<<\n'%text[not ARMED]

################## GUI ##################

root = Tk()
root.wm_title("Pluto Drone - MSP Control Panel")

im = Image.open('bg.jpg')
tkimage = ImageTk.PhotoImage(im)
imgvar = Label(root, image = tkimage)
imgvar.place(x=0, y=0, relwidth=1, relheight=1)

frame1 = Frame(root)
frame1.pack(side=RIGHT, expand = 1)

var1 = DoubleVar()
var2 = DoubleVar()
var3 = DoubleVar()
var4 = DoubleVar()
var5 = DoubleVar()

scale1 = Scale(root, from_ = 0, to = 100, length=300, troughcolor='#AAAAAA', fg='white', bg='#222222', orient=HORIZONTAL, variable = var1)
scale1.pack(anchor=CENTER)
frame2 = Frame(root)
frame2.pack()
button1 = Button(frame2, text="Set KP", command=sel1)
button1.pack(side=LEFT)
label1 = Label(frame2)
label1.pack()

scale2 = Scale(root, from_ = 0, to = 100, length=300, troughcolor='#AAAAAA',
               fg='white', bg='#222222', orient=HORIZONTAL, variable = var2)
scale2.pack(anchor=CENTER)
frame3 = Frame(root)
frame3.pack()
button2 = Button(frame3, text="Set KI", command=sel2)
button2.pack(side=LEFT)
label2 = Label(frame3)
label2.pack()

scale3 = Scale(root, from_ = 0, to = 100, length=300, troughcolor='#AAAAAA',
               fg='white', bg='#222222', orient=HORIZONTAL, variable = var3)
scale3.pack(anchor=CENTER)
frame4 = Frame(root)
frame4.pack()
button3 = Button(frame4, text="Set KD", command=sel3)
button3.pack(side=LEFT)
label3 = Label(frame4)
label3.pack()

scale5 = Scale(root, from_=-10, to=10, resolution=0.2, length=300, troughcolor='#AAAAAA',
               fg='white', bg='#222222', orient=HORIZONTAL, variable = var5)
scale5.pack(anchor=CENTER)
frame6 = Frame(root)
frame6.pack()
buttonSP = Button(frame6, text="Set Target", command=set_target)
buttonSP.pack(side=LEFT)
label5 = Label(frame6, text="0")
label5.pack()

scale4 = Scale(root, from_ = 1000, to = 2000, length=300, troughcolor='#AAAAAA',
               fg='white', bg='#222222', orient=HORIZONTAL, variable = var4)
scale4.pack(anchor=CENTER)
frame5 = Frame(root)
frame5.pack()
button11 = Button(frame5, text="Set Throttle", command=set_throttle)
button11.pack(side=LEFT)
label4 = Label(frame5, text="0")
label4.pack()

lbvar = StringVar(root)
choices = ['Roll', 'Pitch', 'Yaw', 'Altitude']
lbvar.set('Roll')
lb1 = OptionMenu(frame1, lbvar, *choices, command=set_pid_index)
lb1.config(width=18, pady=8)
lb1.pack()

buttonOpen = Button(frame1, text="Open COM Port", bg='red', fg='white', command=toggleCOM, width=20, pady=5)
buttonOpen.pack()
button4 = Button(frame1, text="GET IDENT", command=get_ident, width=20, pady=5)
button4.pack()
button5 = Button(frame1, text="GET ATTITUDE", command=get_attitude, width=20, pady=5)
button5.pack()
button6 = Button(frame1, text="GET RAW_IMU", command=get_raw_imu, width=20, pady=5)
button6.pack()
button7 = Button(frame1, text="GET RC", command=get_rc, width=20, pady=5)
button7.pack()
button8 = Button(frame1, text="GET MOTOR", command=get_motor, width=20, pady=5)
button8.pack()
button9 = Button(frame1, text="SET PID", bg='#777777', fg='white', command=send_pid, width=20, pady=5)
button9.pack()
button10 = Button(frame1, text="ARM DRONE", bg='red', fg='white', command=arm, width=20, pady=5)
button10.pack()

set_pid_index(AXIS)

root.mainloop()
