from __future__ import print_function
import pygame
import ctypes
from MultiWii import *

MAX_ANGLE_FACTOR = 6
ALT_HOLD = 0
ARM_DRONE = 0
LAST_S1 = 0
LAST_L1 = 0
LAST_A = 0
LAST_B = 0
LAST_X = 0
LAST_Y = 0
VIBRATE = 0
THROTTLE = 1300
HEADING = 0
PITCH = 0
ROLL = 0
PITCH_SP = 0
ROLL_SP = 0

set_raw_rc = [0]*8
flasher = 0
vibrate_start_time = 0
vibrate_sec = 0

print('\nWireless MSP Controller\n')
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print('>>> Initialized Joystick: ' + j.get_name())
drone = MultiWii()
drone.start('COM12')

print('\n'*4 + '-'*24 + ' CONTROLLER ' + '-'*24 + '\n')
print('PITCH\tROLL\tYAW\tTHR\tARM\tALT\tP-TR\tR-TR')
print('-'*60)

def getJoystick():
    out = [0]*5 + [0]*10 + [0]
    it = 0 #iterator
    pygame.event.pump()
    
    #Read input from the two joysticks       
    for i in range(0, j.get_numaxes()):
        out[it] = j.get_axis(i)
        it+=1
        
    #Read input from buttons
    for i in range(0, j.get_numbuttons()):
        out[it] = j.get_button(i)
        it+=1

    out[it] = j.get_hat(0)  
    return out

def constrain(x, xmin, xmax):
    if x > xmax: return xmax
    elif x < xmin: return xmin
    else: return x

def mapfloat(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def printf(frame, mode=0):
    for i in frame:
        pass
        #if not mode: print(hex(ord(i)).split('0x')[1].upper().zfill(2),)
        #else: print(ord(i),)
    #print

# Define necessary structures
class XINPUT_VIBRATION(ctypes.Structure):
    _fields_ = [("wLeftMotorSpeed", ctypes.c_ushort),
                ("wRightMotorSpeed", ctypes.c_ushort)]

xinput = ctypes.windll.xinput1_1  # Load Xinput.dll

# Set up function argument types and return type
XInputSetState = xinput.XInputSetState
XInputSetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_VIBRATION)]
XInputSetState.restype = ctypes.c_uint

# Now we're ready to call it.  Set left motor to 100%, right motor to 50%
# for controller 0
vibration = XINPUT_VIBRATION(65535, 32768)
XInputSetState(0, ctypes.byref(vibration))

# You can also create a helper function like this:
def set_vibration(controller, left_motor, right_motor):
    vibration = XINPUT_VIBRATION(int(left_motor * 65535), int(right_motor * 65535))
    XInputSetState(controller, ctypes.byref(vibration))

def vibrate(sec):
    global VIBRATE, vibrate_start_time, vibrate_sec
    if not VIBRATE:
        VIBRATE = True
        vibrate_start_time = time.time()
        vibrate_sec = sec
        set_vibration(0, 0.7, 0.7)

def vibration_handler():
    global VIBRATE, vibrate_start_time, vibrate_sec
    delta = time.time() - vibrate_start_time
    
    if VIBRATE and (delta >= vibrate_sec):
        VIBRATE = False
        vibrate_sec = 0
        set_vibration(0, 0, 0)

#####################
''' Main Function '''
#####################

vibrate(1)

while True:
    try:
        js = getJoystick()

        LX = int(js[0]*60)
        LY = int(js[1]*-60)
        TH = int(js[2]*-100)
        RY = int(js[3]*-100)
        RX = int(js[4]*100)

        A = int(js[5]*2000)
        B = int(js[6]*2000)
        X = int(js[7]*2000)
        Y = int(js[8]*2000)

        L1 = int(js[9]*2000)
        R1 = int(js[10]*2000)
        S1 = int(js[11]*2000)
        S2 = int(js[12]*2000)
        L3 = int(js[13]*2000)
        R3 = int(js[14]*2000)

        HATX, HATY = js[15]

        ##print (LX, LY, RX, RY, TH, A, B, X, Y, S1, S2, L1, R1)

        # Handle emergency stop
        if S1 or S2:
            print('\n'*5 + '\t\t   >>> EMERGENCY STOP <<<')
            printf(drone.sendFrame(drone.SET_RAW_RC, [0]*8, '<8H'), 1)
            break

        if L3: PITCH_SP = ROLL_SP = 0
        if R3: HEADING = 0

        # Handle AUX1 / ARM Drone (0/2000)
        if A and not LAST_A:
            if not ARM_DRONE:
                ARM_DRONE = 1
                vibrate(0.2)
            else: ARM_DRONE = 0
        LAST_A = A
        if ARM_DRONE:
            if(flasher%4==0):
                if arm_text == '\tARM': arm_text = '\t'
                else: arm_text = '\tARM'
        else: arm_text = '\tOFF'
        set_raw_rc[4] = int(ARM_DRONE*2000)
        
        # Handle AUX2 / Altitude Hold (0/2000)
        if Y and not LAST_Y:
            if not ALT_HOLD:
                ALT_HOLD = 1
                vibrate(0.2)
            else:
                ALT_HOLD = 0
                vibrate(0.2)
        LAST_Y = Y
        if ALT_HOLD:
            if (flasher%4==0):
                if alt_text == '\tHOLD': alt_text = '\t'
                else: alt_text = '\tHOLD'
        else: alt_text = '\tOFF'
        set_raw_rc[5] = int(ALT_HOLD*2000)

        # Handle AUX3 / Trim Pitch / ACTUAL:(-10 -> 10) MSP:(0 -> 200)
        PITCH_SP = constrain(PITCH_SP + (HATY * 0.4), -10, 10)
        set_raw_rc[6] = int((PITCH_SP + 10) * 10)

        # Handle AUX4 / Trim Roll / ACTUAL:(-10 -> 10) MSP:(0 -> 200)
        ROLL_SP = constrain(ROLL_SP + (HATX * 0.4), -10, 10)
        set_raw_rc[7] = int((ROLL_SP + 10) * 10)

        # Handle throttle
        if abs(TH) > 20:
            THROTTLE = constrain(THROTTLE + (float(TH)/3.0), 1000, 1900)
        if L1: THROTTLE = 1500
        elif R1: THROTTLE = 1750
        set_raw_rc[3] = int(THROTTLE)

        # Handle pitch / ACTUAL:(-60 -> 60) MSP:(0 -> 20)
        if abs(LY) < 20: LY = 0
        set_raw_rc[1] = int((LY + 60)/MAX_ANGLE_FACTOR)

        # Handle roll / ACTUAL:(-60 -> 60) MSP:(0 -> 20)
        if abs(LX) < 20: LX = 0
        set_raw_rc[0] = int((LX + 60)/MAX_ANGLE_FACTOR)
        
        # Handle yaw / ACTUAL:(-180 -> 180) MSP:(0 -> 360)
        if abs(RX) > 20:
            HEADING = HEADING + (float(RX)/12.0)
            if HEADING > 180: HEADING -= 360
            if HEADING < -180: HEADING += 360
        set_raw_rc[2] = int(HEADING) + 180

        # CMD display
        text = str(LY) + '\t' + str(LX) + '\t' + str(int(HEADING)) + '\t'
        text += str(int(THROTTLE)) + arm_text + alt_text
        text += '\t%.1f\t%.1f\t'%(PITCH_SP, ROLL_SP)
        print(text, end='\r')
        #print(set_raw_rc)

        # Send frame
        printf(drone.sendFrame(drone.SET_RAW_RC, set_raw_rc, '<8H'), 1)
        time.sleep(0.1)

        # Handlers
        flasher += 1
        vibration_handler()
        
    except KeyboardInterrupt:
        print('\n'*5 + '>>> USER STOPPED')
        break

vibrate(1)
while(VIBRATE): vibration_handler()
drone.close()
#raw_input('\n\n\nPress any key to close...')
