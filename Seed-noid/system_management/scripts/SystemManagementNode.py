#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは，上位アプリを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#10/05 test_server.pyと単体検証を行うために，クラス部分をコメントアウト→統合試験でも成功
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

from abc import ABCMeta
from abc import abstractmethod
import rospy
from system_management.msg import *
from system_management.srv import *
from std_msgs.msg import Empty
import sys

#TestMover
import time
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import String

PICK_COMMAND = 0
DOWN_COMMAND = 1
PLACE_COMMAND = 2

class SystemManagementBase(metaclass=ABCMeta):
    """
    Base class that SystemManagement communication
    Define machine-dependent packages
    """
    @abstractmethod
    def execute(self, data):
        pass


class SystemManagementPublisher(SystemManagementBase):
    def __init__(self, topic_name, class_type):
        self.pub = rospy.Publisher(topic_name, class_type, queue_size = 10)

    def execute(self, data):
        self.pub.publish(data)

class SystemManagementClient(SystemManagementBase):
    def __init__(self, service_name, service_class):
        self.proxy = rospy.ServiceProxy(service_name, service_class)

    def execute(self, data):
        try :
            result = self.proxy(data)
            print(result)
            return result
        except rospy.ServiceException as e:
            rospy.loginfo("ServiceException : %s" % e)
            return False

#システム終了指令.
class SystemManagementHalt(SystemManagementPublisher):
    def __init__(self,):
        super().__init__('sys_manage_halt', Empty)

    def execute(self):
        super().execute(Empty())

#作業開始指令.
class SystemManagementTaskCommand(SystemManagementClient):
    def __init__(self):
        super().__init__('sys_manage_service', SystemManagement)

    def execute(self, command_id):
        print('{} start'.format(self.__class__.__name__))

        ###作業コマンド作成 ここから.

        #作業コマンドリスト作成.
        set_task_command_list = TaskCommandList()
        set_task_command = TaskCommand()
        set_task_command.task_command_id = command_id
        set_task_command.work_type_id = 1
        set_task_command.number_of_items_picked = 1
        
        #ワーク検知エリア設定.
        set_task_command.work_presence_area = WorkPresenceArea()
        set_task_command.work_presence_area.start_point.x = -2.0
        set_task_command.work_presence_area.start_point.y = -2.0
        set_task_command.work_presence_area.start_point.z = 0.0
        set_task_command.work_presence_area.end_point.x = 2.0
        set_task_command.work_presence_area.end_point.y = 2.0
        set_task_command.work_presence_area.end_point.z = 2.0

        #廃棄候補エリアリスト作成.
        set_task_command.candidate_discharge_location = CandidateDischargeLocationAreaList()

        #廃棄候補エリア作成.
        area_list = CandidateDischargeLocationArea()
        area_list.start_point.x = -2.0
        area_list.start_point.y = -2.0
        area_list.start_point.z = 0.0
        area_list.end_point.x = 2.0
        area_list.end_point.y = 2.0
        area_list.end_point.z = 2.0
        set_task_command.candidate_discharge_location.candidate_discharge_location_area_list.append(area_list)

        print(set_task_command)
        set_task_command_list.task_command_list.append(set_task_command)

        #ここまで.
        srv = SystemManagementRequest()
        srv.task_info_list = set_task_command_list
        ret = super().execute(srv)
        print('reslt ', ret.task_result_list.task_result)

class TestMover():
    def __init__(self):
        ################################### 初期値設定 ####################################

        self.rate = rospy.Rate(10)  # 10hz

        ########## PI制御のパラメータ初期化（偏差、時間、偏差の積分） ##########
        self.e_pre_x = 0
        self.T_pre_x = time.time()
        self.ie_x = 0

        self.e_pre_y = 0
        self.T_pre_y = time.time()
        self.ie_y = 0

        self.e_pre_z = 0
        self.T_pre_z = time.time()
        self.ie_z = 0

        ########## 状態遷移のための変数 ##########
        # 連続して同じ処理をしないための変数（/signをsubscribeした時）
        self.init = ''

        # コマンドによる割り込み処理用の変数
        self.cancel = False # True:マーカー補正可能 False:マーカー補正不可（コマンド処理実行時）

        # フィードバック用の変数（/cmd_velをpublishした時）
        self.bool = True # True:成功 False:失敗

        ########## マーカー関連の変数 ##########
        # マーカー補正したときのマーカーID（後退する量を決めるための変数）
        self.pre_id = 0

        # マーカー補正を繰り返ししないための変数（マーカー補正終了時）
        self.adjust = False

        # yaw→x→yの順にマーカー補正するための変数
        self.void_x = 0
        self.void_y = 0
        self.void_z = 0

        # マーカーの2段階補正を切り替えるための変数
        self.change = False

        # 姿勢分布中心の計算のための変数
        self.elr_ttl = 0.0
        self.cnt_z = 0
        self.centerdeg_z = 0.0

        ##########################################################################

        ########################### パラメーター設定 ###############################

        ########## 台形加速 ##########
        # 目標速度
        self.tar_vel_x = 0.3 # [m]
        self.tar_vel_y = 0.3

        # 目標加速・減速時間
        self.acc_time_x = 1 # [s]
        self.acc_time_y = 1

        # cmd_velのpublish
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #######################################################################

    # 前後移動速度指定付き
    def pub_x_vel(self, data, vel):
        self.tar_vel_x = vel
        self.pub_x(data)
        self.tar_vel_x = 0.3

    # 前後移動
    def pub_x(self, data):
        # 目的の距離と速度を設定
        dist_x = data # [m]
        
        # Twist 型のデータ
        t = Twist()
        t.angular.z = 0

        # 等速時間の計算
        tar_time = (abs(dist_x) - self.tar_vel_x) / self.tar_vel_x

        # 速度の向き設定
        if (dist_x > 0):
            self.tar_vel_x = self.tar_vel_x
        if (dist_x < 0):
            self.tar_vel_x = -self.tar_vel_x

        # 開始の時刻を保存
        start_time = time.time()
        # 経過した時刻を取得
        end_time = time.time()

        # 等速時間があるとき
        if (tar_time >= 0):
            # 加速
            while (end_time - start_time <= self.acc_time_x):
                t.linear.x = (self.tar_vel_x / self.acc_time_x) * (end_time - start_time)
                self.cmd_pub.publish(t)
                end_time = time.time()

            # 等速
            while (self.acc_time_x <= end_time - start_time <= tar_time + self.acc_time_x):
                t.linear.x = self.tar_vel_x
                self.cmd_pub.publish(t)
                end_time = time.time()

            # 減速
            while (end_time - start_time <= tar_time + (self.acc_time_x + self.acc_time_x)):
                t.linear.x = (self.tar_vel_x / self.acc_time_x) * ((tar_time + (self.acc_time_x + self.acc_time_x)) - (end_time - start_time))
                self.cmd_pub.publish(t)
                end_time = time.time()
        
        # 等速時間がないとき
        if (tar_time < 0):
            acc_time = dist_x / (self.tar_vel_x / self.acc_time_x)
            # 加速
            while (end_time - start_time <= acc_time):
                t.linear.x = (self.tar_vel_x / self.acc_time_x) * (end_time - start_time)
                self.cmd_pub.publish(t)
                end_time = time.time()

            # 減速
            while (end_time - start_time <= acc_time + acc_time):
                t.linear.x = (self.tar_vel_x / self.acc_time_x) * ((acc_time + acc_time) - (end_time - start_time))
                self.cmd_pub.publish(t)
                end_time = time.time()

        self.tar_vel_x = abs(self.tar_vel_x)
        t.linear.x = 0

    def rotate(self, angle, speed, duration):
        vel_msg = Twist()
        
        # Set the angular velocity in the z-axis
        vel_msg.angular.z = speed

        # Rotate for a certain duration to achieve the desired angle
        end_time = rospy.Time.now() + rospy.Duration(duration)
        
        while rospy.Time.now() < end_time:
            self.cmd_pub.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot after rotation
        vel_msg.angular.z = 0
        self.cmd_pub.publish(vel_msg) 

def halt():
    cmd = SystemManagementHalt()
    #終了処理
    print("halt")
    rospy.sleep(0.1)
    cmd.execute()
    rospy.sleep(0.1)

def main(data = None):
    cmd = SystemManagementTaskCommand()

    testmover = TestMover()

    #ピック
    print("pick")
    cmd.execute(PICK_COMMAND)

     #机までの移動
    testmover.pub_x_vel(-0.4, 0.2)

    #ダウン
    print("down")
    cmd.execute(DOWN_COMMAND)

    rospy.sleep(1)
    testmover.pub_x_vel(-0.8, 0.2)
    rospy.sleep(1)
    testmover.rotate(1.57, 0.2, (5.23)*1.51)
    rospy.sleep(1)
    testmover.pub_x_vel(1.97, 0.2)
    rospy.sleep(1)
    testmover.rotate(-1.57, -0.2, (5.23)*1.51)
    rospy.sleep(1)
    testmover.pub_x_vel(0.2551, 0.2)

    #リリース
    print("Place")
    cmd.execute(PLACE_COMMAND)

    #戻る
    testmover.pub_x_vel(-0.2551, 0.3)
    testmover.rotate(-1.57, -0.3, 5.23)
    testmover.pub_x_vel(1.975, 0.3)
    testmover.rotate(1.57, 0.3, 5.23)
    testmover.pub_x_vel(1.250, 0.3)

    return Empty()


if __name__ == '__main__':
    rospy.init_node("system_management_node")
    args = sys.argv
    print("args:", args)
    try:
        if 2 == len(args):
            #システム終了指令.
            if 'halt' == args[1]:
                halt()
            else:
                main()
        else:
            main()
    except rospy.ROSInterruptException:
        pass