#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (http://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (furo), Chiba Institute of Technology.
# This source code form is subject to the terms of the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at http://mozilla.org/MPL/2.0/.

import numpy as np
import mathi
import copy

from pose2d import Pose2D
from Scan2d import Scan2D
from nn_grid_table import NNGridTable


# 部分地図
class Submap:
    def __init__(self, atdS=0.0, cntS=0, cntE=-1, mps=None):
        self.atdS = atdS # 部分地図の始点での累積走行距離
        self.cntS = cntS # 部分地図の最初のスキャン番号
        self.cntE = cntE # 部分地図の最後のスキャン番号
        self.mps = mps if mps else np.empty(0) # 部分地図内のスキャン点群 vector<LPoint2D>

    def addPoints(self, lps):
        mps_list = self.mps.tolist()
        for i in range(len(lps)):
            mps_list.append(lps[i])
        self.mps = np.asarray(mps_list)

    # 格子テーブルを用いて部分地図の代表点を得る
    def subsamplePoints(self, nthre):
        nntab = NNGridTable() # 格子テーブル
        for i in range(len(self.mps)):
            lp = self.mps[i]
            nntab.addPoint(lp) # 全点を登録
        sps = np.empty(0)
        sps = nntab.makeCellPoints(nthre, sps) # nthre個以上のセルの代表点をspsに入れる
        return sps

# 点群地図の基底クラス
class PointCloudMap:
    MAX_POINT_NUM = 1000000 # globalMapの最大点数

    def __init__(
        self,
        nthre=5, # nthre=0,
        poses=None,
        lastPose=None,
        lastScan=None,
        globalMap=None,
        localMap=None,
        nntab=None,
        atdThre=5.,
        atd=0.,
        submaps=None
    ):
        # 格子テーブルセル点数閾値(GTとLPのみ)
        self.nthre = nthre
        # ロボット軌跡
        self.poses = poses if poses else np.empty([0, 0])
        # 最後に推定したロボット位置
        self.lastPose = lastPose if lastPose else Pose2D()
        # 最後に処理したスキャン
        self.lastScan = lastScan if lastScan else Scan2D()
        