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
from human_collaboration.msg import TargetArea
from system_management.msg import *
from system_management.srv import *
from std_msgs.msg import Empty
import sys
import os

# 相対パスでhuman_collaboration/scriptsのパスを追加
sys.path.append(os.path.join(os.path.dirname(__file__), '../../human_collaboration/scripts'))
# インポート
from EnumerateModule import EnumCommandReceiveState

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

class SystemManagementServer(SystemManagementBase):
    def __init__(self, service_name, service_class, callback_impl):
        self.server = rospy.Service(service_name, service_class, callback_impl)

    def service_delete(self, msg=''):
        self.server.shutdown(msg)

    def execute(self, object):
        pass

#作業結果コマンド受信クラス.
class TaskResultServer(SystemManagementServer):
    def __init__(self):
        super().__init__('share_task_result', ShareTaskResult, self.recv_task_result)
        self.task_failed = 0

    def service_delete(self):
        super().service_delete('share_task_result deleted')

    #作業結果コマンド受信.
    def recv_task_result(self, data):
        print("TaskResult:", data.task_result.task_result)
        if(False == data.task_result.task_result):
            self.task_failed = 1

        return Empty()
    
    def get_task_failed(self):
        return self.task_failed
    
    def reset_task_result(self):
        self.task_failed = 0

#作業完了コマンド受信クラス.
class TaskCompleteServer(SystemManagementServer):
    def __init__(self):
        super().__init__('share_task_complete', ShareTaskComplete, self.recv_task_complete)
        self.task_complete = 0

    def service_delete(self):
        super().service_delete('share_task_complete deleted')

    #作業完了コマンド受信.
    def recv_task_complete(self, data):
        print("TaskComplete")
        self.task_complete = 1
        return Empty()
    
    def get_task_complete(self):
        return self.task_complete
    
    def reset_task_complete(self):
        self.task_complete = 0

#システム終了指令.
class SystemManagementHaltPublisher(SystemManagementPublisher):
    def __init__(self,):
        super().__init__('sys_manage_halt', SystemManagementHalt)

    def execute(self):
        msg = SystemManagementHalt()
        msg.empty = Empty()
        super().execute(msg)

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
        set_task_command.work_presence_area = TargetArea()
        set_task_command.work_presence_area.start_point.x = -2.0
        set_task_command.work_presence_area.start_point.y = -2.0
        set_task_command.work_presence_area.start_point.z = 0.0
        set_task_command.work_presence_area.end_point.x = 2.0
        set_task_command.work_presence_area.end_point.y = 2.0
        set_task_command.work_presence_area.end_point.z = 2.0

        #廃棄候補エリアリスト作成.
        set_task_command.candidate_discharge_location = CandidateDischargeLocationAreaList()

        #廃棄候補エリア作成.
        area_list = TargetArea()
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
        print('reslt ', ret.command_receive_result)
        return ret.command_receive_result

def wait(result:TaskResultServer, comp:TaskCompleteServer):
    while ((False == result.get_task_failed()) and (False == comp.get_task_complete()) and (not rospy.is_shutdown())):
        rospy.sleep(0.05)
    return 

def halt():
    cmd = SystemManagementHaltPublisher()
    
    #終了処理
    print("halt")
    rospy.sleep(0.1)
    cmd.execute()
    rospy.sleep(0.1)

def main(data = None):
    cmd = SystemManagementTaskCommand()
    result = TaskResultServer()
    complete = TaskCompleteServer()

    #カップを運ぶ.
    print("CupCarry")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(1)):
        print("command_retry")
        rospy.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        result.service_delete()
        complete.service_delete()
        return Empty()
    
    result.service_delete()
    complete.service_delete()
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