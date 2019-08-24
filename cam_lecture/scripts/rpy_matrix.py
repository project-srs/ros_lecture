#!/usr/bin/env python

import sys
import numpy as np
import tf.transformations as tr

try:
    args = sys.argv
    roll = float(args[1])
    pitch = float(args[2])
    yaw = float(args[3])
except:
    print("syntax: rpy_matrix.py roll pitch yaw")
else:
    np.set_printoptions(precision=3, floatmode='fixed')
    R0 = tr.euler_matrix(roll, pitch, yaw, 'sxyz')
    R0 = R0[0:3, 0:3].reshape(-1)
    print(np.array2string(R0, precision=3, separator=',', suppress_small=True))