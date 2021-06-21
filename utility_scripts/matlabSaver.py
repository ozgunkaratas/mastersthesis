from scipy.io import savemat

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation

#utility script for when post-processing in matlab is necessary

xsens_qw = xsensnewOrientd
xsens_qx = xsensnewOrienta
xsens_qy = xsensnewOrientb
xsens_qz = xsensnewOrientc

hl_qw = hlnewOrientd
hl_qx = hlnewOrienta
hl_qy = hlnewOrientb
hl_qz = hlnewOrientc

mdic = {"xsens_qw": xsens_qw, "xsens_qx": xsens_qw, "xsens_qy": xsens_qy, "xsens_qz": xsens_qz,
        "hl_qw": hl_qw, "hl_qx": hl_qw, "hl_qy": hl_qy, "hl_qz": hl_qz,
        "R_hl_xsens": R_hl_xsens,
        "avg_R_hl_xsens": avg_R_hl_xsens,
        "xsensstackedMatrix":xsensstackedMatrix,
        "hlstackedMatrix":hlstackedMatrix}
savemat("orients.mat", mdic)
