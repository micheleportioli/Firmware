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

def generateHP1Table():
    a = 200.0 # [mm]
    b = 760.0
    c = 805.0
    d = 570.0


    # Roll constraints M*r = f
    M_r = np.matrix([[b, -c],
                    [-1, -1]])
    M_inv = np.linalg.pinv(M_r)
    f_r = np.matrix([-a, -1]).T
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

    # For PX4, format is out = (roll*rollScale + pitch*pitchScale + yaw*yawScale + thrust)*thrustScale
    rSF = rollScaleFront / thrustScaleFront
    rSM = rollScaleMid / thrustScaleMid
    rSR = rollScaleRear / thrustScaleRear
    pSF = pitchScaleFront / thrustScaleFront
    pSM = pitchScaleMid / thrustScaleMid
    pSR = pitchScaleRear / thrustScaleRear
    ySF = yawScaleRear / thrustScaleFront
    ySM = yawScaleMid / thrustScaleMid
    ySR = yawScaleRear / thrustScaleRear
    tSF = thrustScaleFront
    tSM = thrustScaleMid
    tSR = thrustScaleRear


    print("const MultirotorMixer::Rotor _config_v24_hp1[] = {")
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(-rSM,  pSM, -ySM, tSM ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format( rSM,  pSM,  ySM, tSM ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format( rSF,  pSF, -ySF, tSF ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(-rSR, -pSR,  ySR, tSR ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format(-rSF,  pSF,  ySF, tSF ))
    print("\t{{ {:9f}, {:9f}, {:9f}, {:9f} }},".format( rSR, -pSR, -ySR, tSR ))
    print("};\n")
