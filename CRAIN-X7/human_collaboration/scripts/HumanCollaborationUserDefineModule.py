#!/usr/bin/env python3
# coding: UTF-8

import rospy
from rospy.topics import Publisher, Subscriber
from std_msgs.msg import String
from HumanCollaborationStateHeader import *
from TransitionModule import Transition
from HumanCollaborationToolModule import HumanCollaborationTool
from HumanCollaborationStateHeader import *
from HumanCollaborationCommunicationModule import *
from geometry_msgs.msg import TransformStamped
from MovePlannerModule import MovePlanner

#定数の定義
HAND_SET_VALUE = 30.0
HAND_VELOCITY = 0.01
DEFAULT_VELOCITY = 1.0
RETRY_COUNT_MAX = 3
PREPAR_WAIT = 3
PAUSED_WAIT = 0.5
NON_OPERATING_WAIT = 1
COFFEE_WAIT_TIME = 60.0

PUSH_1_VAL_1 = -36.383397234359
PUSH_VAL_2 = -10.729666037856
PUSH_VAL_3 = -81.298815016057
PUSH_VAL_4 = -84.890637153258
PUSH_VAL_5 = -20.039084293141
PUSH_VAL_6 = -7.9488153791884
PUSH_VAL_7 = -21.005835980866
PUSH_1_VEL =  1.0
PUSH_2_VAL_1 = -41.383397234359
PUSH_2_VEL =  0.5
PUSH_3_VAL_1 = -55.383397234359
PUSH_3_VEL =  0.07
PUSH_4_VAL_1 = -36.383397234359
PUSH_4_VEL =  0.07
APPROACH_VAL_1 = -76.640625
APPROACH_VAL_2 = -32.431640625
APPROACH_VAL_3 = -47.8125
APPROACH_VAL_4 = -114.521484375
APPROACH_VAL_5 = 71.71875
APPROACH_VAL_6 = 44.912109375
APPROACH_VAL_7 = -120.14648437499999
APPROACH_VEL =  1.0
PICK_VAL_1 = -84.19921875
PICK_VAL_2 = -34.27734375
PICK_VAL_3 = -44.912109375
PICK_VAL_4 = -107.75390625
PICK_VAL_5 = 74.970703125
PICK_VAL_6 = 41.484375
PICK_VAL_7 = -118.037109375
PICK_VEL = 0.11
UP_VAL_1 = -57.83203125
UP_VAL_2 = -13.447265625
UP_VAL_3 = -55.1953125
UP_VAL_4 = -133.505859375
UP_VAL_5 = 51.767578125
UP_VAL_6 = 52.734375
UP_VAL_7 = -111.181640625
UP_VEL = 0.11
CARRY_1_VAL_1 = -87.626953125
CARRY_1_VAL_2 = 7.3828125
CARRY_1_VAL_3 = -33.837890625
CARRY_1_VAL_4 = -156.708984375
CARRY_1_VAL_5 = -20.654296875
CARRY_1_VAL_6 = 68.02734375
CARRY_1_VAL_7 = -79.013671875
CARRY_1_VEL = 0.11
CARRY_2_VAL_1 = -47.548828125
CARRY_2_VAL_2 = 2.63671875
CARRY_2_VAL_3 = 2.4609375
CARRY_2_VAL_4 = -158.291015625
CARRY_2_VAL_5 = -5.009765625
CARRY_2_VAL_6 = 72.24609375
CARRY_2_VAL_7 = -85.341796875
CARRY_2_VEL = 0.05
CARRY_3_VAL_1 = 34.541015625
CARRY_3_VAL_2 = 2.900390625
CARRY_3_VAL_3 = 21.708984375
CARRY_3_VAL_4 = -154.072265625
CARRY_3_VAL_5 = 1.142578125
CARRY_3_VAL_6 = 66.005859375
CARRY_3_VAL_7 = -90.439453125
CARRY_3_VEL = 0.05
CARRY_4_VAL_1 = 47.109375
CARRY_4_VAL_2 = 2.900390625
CARRY_4_VAL_3 = 21.708984375
CARRY_4_VAL_4 = -154.072265625
CARRY_4_VAL_5 = 1.142578125
CARRY_4_VAL_6 = 66.005859375
CARRY_4_VAL_7 = -90.439453125
CARRY_4_VEL = 0.05
CARRY_5_VAL_1 = 47.109375
CARRY_5_VAL_2 = -35.595703125
CARRY_5_VAL_3 = 18.984375
CARRY_5_VAL_4 = -114.521484375
CARRY_5_VAL_5 = -5.361328125
CARRY_5_VAL_6 = 59.67773437500001
CARRY_5_VAL_7 = -97.734375
CARRY_5_VEL = 0.11
RELEASE_1_VAL_1 = 47.109375
RELEASE_1_VAL_2 = -40.250399699499
RELEASE_1_VAL_3 = 18.984375
RELEASE_1_VAL_4 = -114.521484375
RELEASE_1_VAL_5 = -5.361328125
RELEASE_1_VAL_6 = 59.67773437500001
RELEASE_1_VAL_7 = -97.734375
RELEASE_1_VEL = 0.01
RELEASE_2_VAL_1 = 34.892578125
RELEASE_2_VAL_2 = -21.796875
RELEASE_2_VAL_3 = 37.353515625
RELEASE_2_VAL_4 = -118.30078125
RELEASE_2_VAL_5 = -2.548828125
RELEASE_2_VAL_6 = 45.791015625
RELEASE_2_VAL_7 = -98.173828125
RELEASE_2_VEL = 0.2
RELEASE_3_VAL_1 = -0.087890625
RELEASE_3_VAL_2 = -12.568359375
RELEASE_3_VAL_3 = 27.24609375
RELEASE_3_VAL_4 = -140.888671875
RELEASE_3_VAL_5 = -10.283203125
RELEASE_3_VAL_6 = 81.474609375
RELEASE_3_VAL_7 = -97.734375
RELEASE_3_VEL = 1.0

###########################################
#MovePlanner共有クラス.                   #
###########################################
class ShareMovePlanner:
    move_plannner = MovePlanner()
    @classmethod
    def get_move_planner(cls):
        return cls.move_plannner
    
###########################################
#初期処理クラス.                          #
###########################################
class INITIALISE(HumanCollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinalServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0

    def execute(self, userdata):
        HumanCollaborationCurrentData.SetCurrentState(self)
        
        if self.tran.get_event().is_workend():
            task_command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
            task_command = task_command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        if self.preempt('INIITIALISE is being preempted!!!'):
            return 'preempted'
        
        #初期処理を行う.
        return self.initialise()

    def initialise(self):
        ###ロボットごとの初期動作.
        #ここから.

        while not HumanCollaborationTool.is_shutdown():
          #初期位置移動.
          ret = self.init_pose()
          if 'succeeded' == ret:
                break
          elif 'retry' == ret:
            continue
          else:
            return ret

        #ここまで.
        return 'succeeded'

    #初期位置移動.
    def init_pose(self):
        self.counter = 0

        #3回以上連続で失敗したら異常終了する.
        while not HumanCollaborationTool.is_shutdown():
            self.counter += 1
            HumanCollaborationTool.loginfo('Initialise_gripper')
            rc = self.planner.gripper_open()
            ret =self.return_btos(rc)
            if 'succeeded' == ret:
                #正常完了の場合はエラーカウンターをリセット.
                self.counter = 0
                break
            else:
                if self.counter >= 3:
                    return 'aborted'
                else:
                    continue

        #3回以上連続で失敗したら異常終了する.
        while not HumanCollaborationTool.is_shutdown():
            self.counter += 1
            HumanCollaborationTool.loginfo('Initialise_pose')
            rc = self.planner.initial_pose()
            ret =self.return_btos(rc)
            if 'succeeded' == ret:
                #正常完了の場合はエラーカウンターをリセット.
                self.counter = 0
                break
            else:
                if self.counter >= 3:
                    return 'aborted'
                else:
                    continue

        return 'succeeded'


###########################################
#スタート前処理クラス.                    #
###########################################
class BEFORE_START(HumanCollaborationState):
    def __init__(self, label, taskfin:TaskFinalServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin

    def execute(self, userdata):
        HumanCollaborationCurrentData.SetCurrentState(self)
        if self.tran.get_event().is_pause():
            HumanCollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        HumanCollaborationTool.loginfo('Before Start ...')
        if self.preempt('State BEFORE_START is being preempted!!!'):
            return 'preempted'
      
        return self.before_start()

    def before_start(self):
        ###ロボットごとのスタート前動作.
        #ここから.

        #ここまで.
        return 'succeeded'

###########################################
#準備中処理クラス.                        #
###########################################
class PREPAR_WORK(HumanCollaborationState):
    def __init__(self, label, taskfin:TaskFinalServer, tran:Transition, area, disc, workd, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin
        self.area = area
        self.disc = disc
        self.workd = workd

    def execute(self, userdata):
        HumanCollaborationCurrentData.SetCurrentState(self)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
      
        HumanCollaborationTool.loginfo('Preparing ...')
        if self.preempt('State Prepar is being preempted!!!'):
            return 'preempted'
        
        #準備処理.
        return self.prepar()

    def prepar(self):
        ###ロボットごとの準備動作.
        #ここから.
    
        self.counter = 0

        #周辺環境範囲設定.
        while not HumanCollaborationTool.is_shutdown():
            ret = self.PeripheralEnvironmentAreaSet()
            if 'succeeded' == ret:
                break
            elif 'retry' == ret:
                continue
            else:
                return ret

        #ワーク検知.
        while not HumanCollaborationTool.is_shutdown():
            ret = self.WorkDetect()
            if 'succeeded' == ret:
                break
            elif 'retry' == ret:
                continue
            else:
                return ret
          
        #排出位置検出.
        while not HumanCollaborationTool.is_shutdown():
            ret = self.DischargeDetect()
            if 'succeeded' == ret:
                break
            elif 'retry' == ret:
                continue
            else:
                return ret
        
        #ここまで.
        return 'succeeded'
    
    #周辺環境範囲を設定する.
    def PeripheralEnvironmentAreaSet(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        return 'succeeded'

    #排出位置を検出する.
    def DischargeDetect(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
    
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        HumanCollaborationTool.loginfo("waiting discharge_position")
        HumanCollaborationTool.wait_for_service('discharge_position_detect_server')
        HumanCollaborationTool.loginfo("discharge_position comes up")

        event = self.check_event()
        if(None != event ):
            return event
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State DischargeDetect is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            #排出位置を探索.
            self.set_discharge_area()
            rc = self.disc.execute(None)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc) 
        else:
            return 'aborted'

    #ワークを検知する.
    def WorkDetect(self):
        HumanCollaborationTool.loginfo("waiting work_det_service")
        HumanCollaborationTool.wait_for_service('work_det_service')
        HumanCollaborationTool.loginfo("work_det_service comes up")

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event

        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State Work Detect is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            #ワーク検知範囲の設定.
            self.set_presence_area()
            rc = self.workd.execute(None)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc) 
        else:
            return 'aborted'

    #排出位置を設定する.
    def set_discharge_area(self):
        #現在のコマンドリストを取得.
        command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
        task_command = command_list.get(0)
        discharge_area_list = task_command.get_DischargeLocationAreaList()
        print(discharge_area_list)
        discharge_area = discharge_area_list[0]
        print(discharge_area)
        self.disc.set_discharge_area(discharge_area.start_point.x,
                                     discharge_area.start_point.y,
                                     discharge_area.start_point.z,
                                     discharge_area.end_point.x,
                                     discharge_area.end_point.y,
                                     discharge_area.end_point.z,)

    #ワーク検知範囲を設定する.
    def set_presence_area(self):
        #現在のコマンドリストを取得.
        command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
        task_command = command_list.get(0)
        presence_area = task_command.work_presence_area
        self.workd.set_presence_area(task_command.task_command_id,
                                     task_command.work_type_id,
                                     presence_area.start_point.x,
                                     presence_area.start_point.y,
                                     presence_area.start_point.z,
                                     presence_area.end_point.x,
                                     presence_area.end_point.y,
                                     presence_area.end_point.z,)
    
    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            HumanCollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None
        
###########################################
#自律動作処理クラス.                      #
###########################################
class OPERATING_WORK(HumanCollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinalServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0
        self.work_label = None

    def execute(self, userdata):
        HumanCollaborationCurrentData.SetCurrentState(self)
        #現在のコマンドリストを取得.
        command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
        HumanCollaborationTool.loginfo(command_list.get(0).task_command_id)
        
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        #自律動作.
        return self.operating_work()

    #自律動作処理.
    def operating_work(self):
        ###ロボットごとの自律動作.
        #ここから.

        #現在のコマンドリストを取得.
        command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
        command_id = command_list.get(0).task_command_id

        #ハンド開ける1.
        if (None == self.work_label) or ('HandOpen1' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'HandOpen1'
                ret = self.hand_open()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #初期位置に戻る1.
        if (None == self.work_label) or ('InitPose1' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'InitPose1'
                ret = self.init_pose()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #ハンドセット位置1.
        if (None == self.work_label) or ('HnadSet1' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'HandSet1'
                ret = self.hand_set()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #ボタンを押す1.
        if (None == self.work_label) or ('Push1' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Push1'
                ret = self.joint_manip(PUSH_1_VAL_1, PUSH_VAL_2, PUSH_VAL_3, PUSH_VAL_4, PUSH_VAL_5, PUSH_VAL_6, PUSH_VAL_7, PUSH_1_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #ボタンを押す2.
        if (None == self.work_label) or ('Push2' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Push2'
                ret = self.joint_manip(PUSH_2_VAL_1, PUSH_VAL_2, PUSH_VAL_3, PUSH_VAL_4, PUSH_VAL_5, PUSH_VAL_6, PUSH_VAL_7, PUSH_2_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #ボタンを押す3.
        if (None == self.work_label) or ('Push3' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Push3'
                ret = self.joint_manip(PUSH_3_VAL_1, PUSH_VAL_2, PUSH_VAL_3, PUSH_VAL_4, PUSH_VAL_5, PUSH_VAL_6, PUSH_VAL_7, PUSH_3_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #ボタンを押す4.
        if (None == self.work_label) or ('Push4' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Push4'
                ret = self.joint_manip(PUSH_4_VAL_1, PUSH_VAL_2, PUSH_VAL_3, PUSH_VAL_4, PUSH_VAL_5, PUSH_VAL_6, PUSH_VAL_7, PUSH_4_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #ハンド開ける2.
        if (None == self.work_label) or ('HandOpen2' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'HandOpen2'
                ret = self.hand_open()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #初期位置に戻る2.
        if (None == self.work_label) or ('InitPose2' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'InitPose2'
                ret = self.init_pose()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                 continue
                else:
                    return ret

        #アプローチ.
        if (None == self.work_label) or ('Approach' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Approach'
                ret = self.joint_manip(APPROACH_VAL_1, APPROACH_VAL_2, APPROACH_VAL_3, APPROACH_VAL_4, APPROACH_VAL_5, APPROACH_VAL_6, APPROACH_VAL_7, APPROACH_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #ハンドセット位置1.
        if (None == self.work_label) or ('HnadSet2' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'HandSet2'
                ret = self.hand_set()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #コーヒー待ち.
        if (None == self.work_label) or ('CoffeeWait' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'CoffeeWait'
                #作業途中で状態遷移するイベントの確認.
                event = self.check_event()
                if(None != event):
                    return event

                HumanCollaborationTool.loginfo('CoffeeWait')
                HumanCollaborationTool.wait_time(COFFEE_WAIT_TIME)
                self.work_label = None
                break
            
        #つかみ位置.
        if (None == self.work_label) or ('Pick' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Pick'
                ret = self.joint_manip(PICK_VAL_1, PICK_VAL_2, PICK_VAL_3, PICK_VAL_4, PICK_VAL_5, PICK_VAL_6, PICK_VAL_7, PICK_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #ハンド閉める.
        if (None == self.work_label) or ('HandClose' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'HandClose'
                ret = self.hand_close()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #持ち上げ位置.
        if (None == self.work_label) or ('Up' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Up'
                ret = self.joint_manip(UP_VAL_1, UP_VAL_2, UP_VAL_3, UP_VAL_4, UP_VAL_5, UP_VAL_6, UP_VAL_7, UP_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #運ぶ1.
        if (None == self.work_label) or ('Carry1' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Carry1'
                ret = self.joint_manip(CARRY_1_VAL_1, CARRY_1_VAL_2, CARRY_1_VAL_3, CARRY_1_VAL_4, CARRY_1_VAL_5, CARRY_1_VAL_6, CARRY_1_VAL_7, CARRY_1_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #運ぶ2.
        if (None == self.work_label) or ('Carry2' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Carry2'
                ret = self.joint_manip(CARRY_2_VAL_1, CARRY_2_VAL_2, CARRY_2_VAL_3, CARRY_2_VAL_4, CARRY_2_VAL_5, CARRY_2_VAL_6, CARRY_2_VAL_7, CARRY_2_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #運ぶ3.
        if (None == self.work_label) or ('Carry3' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Carry3'
                ret = self.joint_manip(CARRY_3_VAL_1, CARRY_3_VAL_2, CARRY_3_VAL_3, CARRY_3_VAL_4, CARRY_3_VAL_5, CARRY_3_VAL_6, CARRY_3_VAL_7, CARRY_3_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #運ぶ4.
        if (None == self.work_label) or ('Carry4' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Carry4'
                ret = self.joint_manip(CARRY_4_VAL_1, CARRY_4_VAL_2, CARRY_4_VAL_3, CARRY_4_VAL_4, CARRY_4_VAL_5, CARRY_4_VAL_6, CARRY_4_VAL_7, CARRY_4_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #運ぶ5.
        if (None == self.work_label) or ('Carry5' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Carry5'
                ret = self.joint_manip(CARRY_5_VAL_1, CARRY_5_VAL_2, CARRY_5_VAL_3, CARRY_5_VAL_4, CARRY_5_VAL_5, CARRY_5_VAL_6, CARRY_5_VAL_7, CARRY_5_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
            
        #離す1.
        if (None == self.work_label) or ('Release1' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Release1'
                ret = self.joint_manip(RELEASE_1_VAL_1, RELEASE_1_VAL_2, RELEASE_1_VAL_3, RELEASE_1_VAL_4, RELEASE_1_VAL_5, RELEASE_1_VAL_6, RELEASE_1_VAL_7, RELEASE_1_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #ハンド開ける3.
        if (None == self.work_label) or ('HandOpen3' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'HandOpen3'
                ret = self.hand_open()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #離す2.
        if (None == self.work_label) or ('Release2' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Release2'
                ret = self.joint_manip(RELEASE_2_VAL_1, RELEASE_2_VAL_2, RELEASE_2_VAL_3, RELEASE_2_VAL_4, RELEASE_2_VAL_5, RELEASE_2_VAL_6, RELEASE_2_VAL_7, RELEASE_2_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #離す3.
        if (None == self.work_label) or ('Release3' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'Release3'
                ret = self.joint_manip(RELEASE_3_VAL_1, RELEASE_3_VAL_2, RELEASE_3_VAL_3, RELEASE_3_VAL_4, RELEASE_3_VAL_5, RELEASE_3_VAL_6, RELEASE_3_VAL_7, RELEASE_3_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #初期位置に戻る3.
        if (None == self.work_label) or ('InitPose3' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'InitPose3'
                ret = self.init_pose()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #ここまで.
        self.work_label = None
        return 'succeeded'

    #ハンド開ける.
    def hand_open(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State HAND_OPEN is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            HumanCollaborationTool.loginfo('HandOpen')
            rc = self.planner.gripper_open()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
    
    #ハンド閉める.
    def hand_close(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State HAND_CLOSE is being preempted!!!'):
                return 'preempted'
            
            self.counter += 1
            HumanCollaborationTool.loginfo('HandClose')
            rc = self.planner.gripper_close()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
    
    #ハンドセット位置.
    def hand_set(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State HAND_OPEN is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            HumanCollaborationTool.loginfo('HandOpen')
            rc = self.planner.gripper_value(HAND_SET_VALUE, HAND_SET_VALUE, HAND_VELOCITY)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #初期位置移動.
    def init_pose(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            HumanCollaborationTool.loginfo('InitPose')
            rc = self.planner.initial_pose()
           #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
        
    def joint_manip(self, val_1, val_2, val_3, val_4, val_5, val_6, val_7, vel):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State NORMAL_MANIP is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            HumanCollaborationTool.loginfo('Manipulate at ({},{},{},{},{},{},{}) in scale velocity {}'.format(val_1, val_2, val_3, val_4, val_5, val_6, val_7, vel))
            rc = self.planner.joint_value(val_1, val_2, val_3, val_4, val_5, val_6, val_7, vel)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            HumanCollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        elif self.tran.get_event().is_workend():
            command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None

###########################################
#非自律動作処理クラス.                    #
###########################################
class NON_OPERATING_WORK(HumanCollaborationState):
    def __init__(self, label, taskfin:TaskFinalServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin

    def execute(self, userdata):
        HumanCollaborationCurrentData.SetCurrentState(self)
        #現在のコマンドリストを取得.
        command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
        HumanCollaborationTool.loginfo(command_list.get(0).task_command_id)
        
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        #非自律動作処理.
        return self.non_operating_work()
    
    #非自律動作処理.
    def non_operating_work(self):
        ###ロボットごとの非自律動作.
        #ここから.

        HumanCollaborationTool.wait_time(NON_OPERATING_WAIT)

        #初期位置に戻る.
        if (None == self.work_label) or ('InitPose1' == self.work_label):
            while not HumanCollaborationTool.is_shutdown():
                self.work_label = 'InitPose1'
                ret = self.init_pose()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #ここまで.
        self.work_label = None
        return 'succeeded'
    
    #初期位置移動.
    def init_pose(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            HumanCollaborationTool.loginfo('InitPose')
            rc = self.planner.initial_pose()
           #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            HumanCollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        elif self.tran.get_event().is_workend():
            command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None

###########################################
#一時停止中処理クラス.                        #
###########################################
class PAUSED_WORK(HumanCollaborationState):
    def __init__(self, label, taskfin:TaskFinalServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin
        self.counter = 0

    def execute(self, userdata):
        HumanCollaborationCurrentData.SetCurrentState(self)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event

        HumanCollaborationTool.loginfo('Preparing ...')
        if self.preempt('State Prepar is being preempted!!!'):
            return 'preempted'
        
        #一時停止処理.
        return self.paused()

    def paused(self):
        ###ロボットごとの一時停止動作.
        #ここから.

        HumanCollaborationTool.wait_time(PAUSED_WAIT)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        #初期位置移動.
        while not HumanCollaborationTool.is_shutdown():
          ret = self.init_pose()
          if 'succeeded' == ret:
                break
          elif 'retry' == ret:
            continue
          else:
            return ret

        #ここまで.
        return 'succeeded'

    #初期位置移動.
    def init_pose(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            HumanCollaborationTool.loginfo('init_pose')
            rc = self.planner.initial_pose()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_workend():
            command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None

###########################################
#終了処理クラス.                          #
###########################################        
class FINALISE(HumanCollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinalServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0

    def execute(self, userdata):
        HumanCollaborationCurrentData.SetCurrentState(self)

        if self.tran.get_event().is_workend():
            task_command_list = HumanCollaborationCurrentData.GetCurrentCommandList()
            task_command = task_command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        if self.preempt('INIITIALISE_POSE is being preempted!!!'):
            return 'preempted'
        
        #終了処理を行う.
        return self.finalise()

    #終了処理.
    def finalise(self):
        ###ロボットごとの終了動作.
        #ここから.

        #初期位置移動.
        while not HumanCollaborationTool.is_shutdown():
            ret = self.init_pose()
            if 'succeeded' == ret:
                break
            elif 'retry' == ret:
                continue
            else:
                return ret

        #終了する.
        HumanCollaborationTool.signal_shutdown("sys_manage_halt")

        #ここまで.
        return 'succeeded'
    
    def init_pose(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            HumanCollaborationTool.loginfo('finalise')
            rc = self.planner.initial_pose()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'