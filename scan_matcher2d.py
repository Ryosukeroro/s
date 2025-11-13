#!/usr/bin/python
# coding: utf-8
#  This Python program is translated by Shuro Nakajima from the following C++ software:
#   LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#    Future Robotics Technology Center (furo), Chiba Institute of Technology.
#  This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
#  If a copy of the MPL was not distributed with this file, you can obtain one
#   at https:/mozilla.org/MPL/2.0/.

import numpy as np
import math
import copy

from l_point2d import LPoint2D, ptype
from pose2d import Pose2D
from scan2d import Scan2D
from point_cloud_map import PointCloudMap
from covariance_calculator import CovarianceCalculator
from ref_scan_maker import RefScanMaker