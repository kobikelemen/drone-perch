from math import sin, cos, atan2, asin



def eulerToQuat(roll, pitch, yaw):
    cy = cos(yaw*0.5)
    sy = sin(yaw*0.5)
    cp = cos(pitch*0.5)
    sp = sin(pitch*0.5)
    cr = cos(roll*0.5)
    sr = sin(roll*0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy     ### NOT SURE SHOULD PUBLISH QUAT AS X,Y,Z,W OR W,X,Y,Z ???
    return x, y, z, w                   ## DIFFERENT ON WIKIPEDIA COMPARED TO MAVROS ...


def quatToEuler(w, x, y, z):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)
    return roll_x, pitch_y # in radians