#!/usr/bin/env python3
# coding: UTF-8
#####################################################################################################################
#このノードは，ワーク検出モジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

import rospy
import yaml
import rospkg
from rospy.topics import Subscriber

from std_msgs.msg import *
from geometry_msgs.msg import *
from work_detection.msg import *
from work_detection.srv import *

class WorkDetect:
    def __init__(self):
        self.setup_req = rospy.Service('work_det_service', WorkDetection, self.pose_request)
        print("Initialization done")

    def ArucoCallback(self, aruco_):
        self.aruco_data = aruco_.markers

    def pose_request(self, req_):
        #ワーク検知処理.
       #ここから.
        
        # リクエストの確認のみ
        print("")
        print("Request Data:")
        print(req_)
        print("")

        srv = WorkDetectionResponse()
        set_data = WorkDetectionResult()
        set_data.work_type_id = req_.work_type_id
        srv.work_detection_result_list.work_info_list.append(set_data)
        return srv
        #ここまで.

if __name__ == "__main__":
    rospy.init_node("work_detection_node")
    wd = WorkDetect()
    rospy.spin()
  
