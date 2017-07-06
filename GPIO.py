import time
import RPi.GPIO as GPIO
import numpy as np

#Use BCM mappings
GPIO.setmode(GPIO.BCM)

#Pins used by base and arm (output)
base_pins = [2,3,4,14]
arm_pins = [17,27,22,23]

#limit switches
arm_ccw_pin = 18
arm_cw_pin = 19
base_ccw_pin = 24
base_cw_pin = 10

#Resolution of stepper motors
base_res = 360/512
arm_res = 360/512

#default GPIO value
#GPIO_DEFAULT = GPIO.HIGH

#Set pin modes to output and set to false
for pin in base_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)
for pin in arm_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)

#Sequence by which to drive stepper motors
Seq = [[1, 0, 0, 1],
       [1, 0, 0, 0],
       [1, 1, 0, 0],
       [0, 1, 0, 0],
       [0, 1, 1, 0],
       [0, 0, 1, 0],
       [0, 0, 1, 1],
       [0, 0, 0, 1]]

#current angles
theta_base = 0
theta_arm = 0

#current sequence number
seq_base = 0
seq_arm = 0

#tolerance for presuming two steps are colinear relative to seq number
tol_colinear_rel = 0.5

#time between interpolation steps
interp_time = 0.05

#test time interval
test_interval = 0.5

#range of theta
arm_theta_range = 90
base_theta_range = 90

#range of sequences
arm_seq_range = 0
base_seq_range = 0

def adj_seq(adj_arm, adj_base):
    global seq_arm, seq_base, Seq, arm_pins, base_pins
    if adj_arm > 1 or adj_arm < -1 or adj_base > 1 or adj_base < -1:
        print("Error: Adjusted by too much")
        return
    new_seq_arm = seq_arm + adj_arm
    new_seq_base = seq_base + adj_base
    if new_seq_arm < 0:
        new_seq_arm += len(Seq)
    if new_seq_arm >= len(Seq):
        new_seq_arm -= len(Seq)
    if new_seq_base < 0:
        new_seq_base += len(Seq)
    if new_seq_base >= len(Seq):
        new_seq_base -= len(Seq)
    for idx, pin in enumerate(Seq[new_seq_arm]):
        #print('idx: ' + str(idx) + 'arm pin: ' + str(arm_pins[idx]) + 'value: ' + str(pin)) 
        GPIO.output(arm_pins[idx], GPIO.LOW if pin == 1 else GPIO.HIGH)
    for idx, pin in enumerate(Seq[new_seq_base]):
        #print('idx: ' + str(idx) + 'base pin: ' + str(base_pins[idx]) + 'value: ' + str(pin))
        GPIO.output(base_pins[idx], GPIO.LOW if pin == 1 else GPIO.HIGH)
    seq_arm = new_seq_arm
    seq_base = new_seq_base


def set_angles(new_theta_base, new_theta_arm):
    global interp_time, theta_base, base_res, theta_arm, arm_res
    d_theta_base = new_theta_base - theta_base
    d_seq_base = int(d_theta_base / base_res)
    theta_act_base = d_seq_base * base_res
    d_theta_arm = new_theta_arm - theta_arm
    d_seq_arm = int(d_theta_arm / arm_res)
    theta_act_arm = d_seq_arm * arm_res
    i = 0 #arm counter
    j = 0 #base counter
    arm_inc = 1 if d_seq_arm >= 0 else -1
    base_inc = 1 if d_seq_base >= 0 else -1
    d_seq_base = abs(d_seq_base)
    d_seq_arm = abs(d_seq_arm)
    tol_colinear = tol_colinear_rel / float(d_seq_base + d_seq_arm)
    #print('Adjustment: theta: ' + str(d_theta_arm) + ',' + str(d_theta_base) + ' seq: ' + str(d_seq_arm) + ',' + str(d_seq_base))
    while i < d_seq_arm or j < d_seq_base:
        if d_seq_arm != 0:
            frac_arm = i / float(d_seq_arm)
        else:
            frac_arm = 1
        if d_seq_base != 0:
            frac_base = j / float(d_seq_base)
        else:
            frac_base = 1
        if frac_arm - frac_base > tol_colinear:
            #base needs to move next
            adj_seq(0, base_inc)
            j += 1
        elif frac_arm - frac_base < -tol_colinear:
            #arm needs to move next
            adj_seq(arm_inc, 0)
            i += 1
        else:
            #both need to move together
            adj_seq(arm_inc, base_inc)
            i += 1
            j += 1
        time.sleep(interp_time)
    theta_arm += theta_act_arm
    theta_base += theta_act_base

def calibrate():
    global arm_res, base_res, arm_theta_range, base_theta_range, arm_seq_range, base_seq_range, arm_cw_pin, \
        arm_ccw_pin, theta_arm, theta_base
    while GPIO.input(arm_ccw_pin) == GPIO.LOW:
        adj_seq(-1, 0)
        time.sleep(interp_time)
    arm_min = seq_arm
    arm_seq_range = 0
    while GPIO.input(arm_cw_pin) == GPIO.LOW:
        adj_seq(1,0)
        arm_seq_range += 1
        time.sleep(interp_time)
    arm_res = arm_theta_range / arm_seq_range
    theta_arm = arm_theta_range / 2
    set_angles(0, theta_base)
    while GPIO.input(base_ccw_pin) == GPIO.LOW:
        adj_seq(0, -1)
        time.sleep(interp_time)
    base_min = seq_base
    base_seq_range = 0
    while GPIO.input(base_cw_pin) == GPIO.LOW:
        adj_seq(0, 1)
        base_seq_range += 1
        time.sleep(interp_time)
    base_res = base_theta_range / base_seq_range
    theta_base = base_theta_range / 2
    set_angles(0,0)

def center():
    global theta_arm, theta_base
    while GPIO.input(arm_cw_pin) == GPIO.LOW:
        adj_seq(1, 0)
        time.sleep(interp_time)
    theta_arm = arm_theta_range / 2
    set_angles(0, theta_base)
    while GPIO.input(base_cw_pin) == GPIO.LOW:
        adj_seq(0, 1)
        time.sleep(interp_time)
    theta_base = base_theta_range / 2
    set_angles(0,0)

#calibrate()

if __name__ == '__main__':
    #global interp_time
    #interp_time = 0.1
    print('GPIO test mode')
    '''
    for pin in base_pins:
        GPIO.output(pin, GPIO.LOW)
        time.sleep(test_interval)
        GPIO.output(pin, GPIO.HIGH)

    for pin in arm_pins:
        GPIO.output(pin, GPIO.LOW)
        time.sleep(test_interval)
        GPIO.output(pin, GPIO.HIGH)
    print("GPIO test complete")
    
    for i in range(1,20):
        adj_seq(0,1)
        time.sleep(test_interval)

    for i in range(1,20):
        adj_seq(0,-1)
        time.sleep(test_interval)

    for i in range(1,20):
        adj_seq(1,0)
        time.sleep(test_interval)

    for i in range(1,20):
        adj_seq(-1,0)
        time.sleep(test_interval)

    for i in range(1,20):
        adj_seq(1,1)
        time.sleep(test_interval)

    for i in range(1,20):
        adj_seq(-1,-1)
        time.sleep(test_interval)

    for i in range(1,20):
        adj_seq(-1,1)
        time.sleep(test_interval)

    for i in range(1,20):
        adj_seq(1,-1)
        time.sleep(test_interval)

    print('Seq test complete')
    '''
    angles = [(-45,-45),
              (45,45),
              (-45,-45),
              (0,0),
              (45,45),
              (0,0),
              (-45,-45),
              (-35,-35),
              (-25,-25),
              (-15,-15),
              (-5,-5),
              (5,5),
              (15,15),
              (25,25),
              (35,35),
              (45,45),
              (30,45),
              (15,45),
              (0,45),
              (0,30),
              (0,15),
              (0,0),
              (0,-15),
              (0,-30),
              (0,-45),
              (-15,-45),
              (-30,-45),
              (-45,-45)]
    for angle in angles:
        print('Angle: ' + str(angle[0]) + ',' + str(angle[1]))
        set_angles(angle[0], angle[1])
        time.sleep(test_interval)

    '''
    dec_arm = arm_theta_range / 10
    dec_base = base_theta_range / 10
    arm_res = 2
    base_res = 2
    while(dec_arm >= arm_res or dec_base >= base_res):
        for i in np.arange(-base_theta_range/2, base_theta_range/2, dec_base):
            for j in np.arange(-arm_theta_range/2, arm_theta_range/2, dec_arm):
                print('Angles: ' + str(i) + ',' + str(j))
                set_angles(i,j)
                time.sleep(test_interval)
        if dec_arm >= arm_res:
            dec_arm /= 2
        if dec_base >= base_res:
            dec_base /= 2
    print('Angle test complete')
    '''
    print('Test Complete')
