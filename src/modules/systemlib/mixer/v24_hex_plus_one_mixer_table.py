import numpy as np

# (3)           (5)   ^
#  |             |    |
#  |             |    b
# (2)           (1)^  |
#  |_____________| a  v
#  |             | x
#  |             | |
#  |             | c
#  |             | |
#  |             | |
# (6)      <-d->(4)v

# In format out =   roll*rollScale + pitch*pitchScale + yaw*yawScale + thrust*thrustScale
# Roll constraint:  Left moment = right moment, do not induce pitch nor yaw. Use maximum of middle motor
# Pitch constraint: Front moment = rear moment, do not use middle to produce pitch control
#                   pitchScaleFront*b = pitchScaleRear*c
# Yaw constraint:   a + yawScaleFront*b = yawScaleRear*c (zero moment around pitch)
#                   and
#                   yawScaleFront + yawScaleRear = yawScaleMid  (zero moment around roll)

a = 190.0 # [mm]
b = 760.0
c = 800.0
d = 575.0

CCW = 1
CW = -CCW

w1 = CW
w2 = CCW
w3 = CW
w4 = CCW
w5 = CCW
w6 = CW

# Roll constraints M*r = f
M_r = np.matrix([[b, -c],
                [w5, w4]])
M_inv = np.linalg.pinv(M_r)
f_r = np.matrix([-a, -w1]).T
r = np.dot(M_inv, f_r)

rollScaleFront = np.asscalar(r[0])
rollScaleMid = 1.0
rollScaleRear = np.asscalar(r[1])
pitchScaleFront = 1.0                   # Front is fully used for pitch since it has less effect than rear
pitchScaleMid = 0.0                     # Since the drone is a bit underpowered, we don't use middle for pitch
pitchScaleRear = pitchScaleFront*b/c    # Rear lever arm is bigger than front, the gain is reduced accordingly
thrustScaleRear = 1.0                   # Alone behind the CG, use max gain
thrustScaleMid = 1.0                    # Produce less pitch moment than rear
thrustScaleFront = (c-a)/b              # Add the missing pitch moment so Mf+Mm = Mr
yawScaleMid = 1.0                       # Since middle motor is alone for yaw, use full gain
yawScaleRear = (a+b)/(b+c)
yawScaleFront = yawScaleMid - yawScaleRear

def printReport():

    print('Front gains (r,p,y,t) = {:9f}, {:9f}, {:9f}, {:9f}'.format(rollScaleFront, pitchScaleFront, yawScaleFront, thrustScaleFront))
    print('Middle gains (r,p,y,t) = {:9f}, {:9f}, {:9f}, {:9f}'.format(rollScaleMid, pitchScaleMid, yawScaleMid, thrustScaleMid))
    print('Rear gains (r,p,y,t) = {:9f}, {:9f}, {:9f}, {:9f}'.format(rollScaleRear, pitchScaleRear, yawScaleRear, thrustScaleRear))
    print('Roll actuation = {:9f}'.format(2*(rollScaleFront + rollScaleMid + rollScaleRear)))
    print('Pitch actuation = {:9f}'.format(2*(pitchScaleFront + pitchScaleMid + pitchScaleRear)))
    print('Yaw actuation = {:9f}'.format(2*(yawScaleFront + yawScaleMid + yawScaleRear)))
    print('Thrust actuation = {:9f}'.format(2*(thrustScaleFront + thrustScaleMid + thrustScaleRear)))

    print('Load front = {:9f}'.format(rollScaleFront+pitchScaleFront+yawScaleFront+thrustScaleFront))
    print('Load mid = {:9f}'.format(rollScaleMid+pitchScaleMid+yawScaleMid+thrustScaleMid))
    print('Load rear = {:9f}'.format(rollScaleRear+pitchScaleRear+yawScaleRear+thrustScaleRear))

def generateHP1Table():

    # For PX4, format is out = (roll*rollScale + pitch*pitchScale + yaw*yawScale + thrust)*thrustScale
    rSF = rollScaleFront / thrustScaleFront
    rSM = rollScaleMid / thrustScaleMid
    rSR = rollScaleRear / thrustScaleRear
    pSF = pitchScaleFront / thrustScaleFront
    pSM = pitchScaleMid / thrustScaleMid
    pSR = pitchScaleRear / thrustScaleRear
    ySF = yawScaleFront / thrustScaleFront
    ySM = yawScaleMid / thrustScaleMid
    ySR = yawScaleRear / thrustScaleRear
    tSF = thrustScaleFront
    tSM = thrustScaleMid
    tSR = thrustScaleRear

    print("const MultirotorMixer::Rotor _config_v24_hp1[] = {")
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(-rSM,  pSM,  w1*ySM, tSM ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format( rSM,  pSM,  w2*ySM, tSM ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format( rSF,  pSF,  w3*ySF, tSF ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(-rSR, -pSR,  w4*ySR, tSR ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(-rSF,  pSF,  w5*ySF, tSF ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format( rSR, -pSR,  w6*ySR, tSR ))
    print("};\n")
