#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import threading

import rospy
import numpy as np
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
from pymodbus.client.sync import ModbusSerialClient as ModbusClient


class DualArmManualDemo(object):
    def __init__(self):
        rospy.loginfo("Initializing DualArmManualDemo...")

        self.initialize_moveit()
        self.initialize_modbus()   # 需要左臂夹爪Modbus时打开

        # ============================================================
        # 这里直接自定义左右臂“视觉坐标点”
        # 格式: [x, y, z]
        # direction 取 0 / 1 / 2 / 3
        # ============================================================
        self.left_cam_point = np.array([0.142, -0.138, 0.517], dtype=float)
        self.left_direction = 0

        self.right_cam_point = np.array([0.020, -0.070, 0.535], dtype=float)
        self.right_direction = 2

        rospy.loginfo("DualArmManualDemo initialized.")

    # ============================================================
    # 初始化
    # ============================================================
    def initialize_moveit(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.left_group = moveit_commander.MoveGroupCommander("L_arm")
        self.right_group = moveit_commander.MoveGroupCommander("R_arm")

        self.init_group(self.left_group, "L_arm")
        self.init_group(self.right_group, "R_arm")

        # ========= 左相机 -> 左机械臂基坐标系（按你的标定修改）=========
        self.left_R = np.array([
            [0.966943,  0.000576,  0.254993],
            [-0.239565, -0.340506, 0.909211],
            [0.087350, -0.940242, -0.329112]
        ])
        self.left_T = np.array([0.085252, 0.01847, 0.308683])   # 45°
        # self.left_T = np.array([0.153252, -0.021530, 0.318683])  # 直抓

        # ========= 右相机 -> 右机械臂基坐标系（按你的标定修改）=========
        self.right_R = np.array([
            [0.96305149, -0.01489413, -0.26890516],
            [0.25163360, -0.30607811,  0.91814853],
            [-0.09598101, -0.95188989, -0.29102112]
        ])
        # self.right_T = np.array([0.65805384, -0.02382935, 0.30006910]) # 直抓
        self.right_T = np.array([0.63605384, -0.02382935, 0.31006910]) #15度

        rospy.loginfo("MoveIt initialized.")

    def init_group(self, group, name):
        group.set_planner_id("RRTConnectkConfigDefault")
        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_planning_time(3.0)
        group.allow_replanning(True)
        group.set_num_planning_attempts(10)
        group.set_max_velocity_scaling_factor(1.0)
        group.set_max_acceleration_scaling_factor(1.0)
        rospy.loginfo("%s initialized.", name)

    def initialize_modbus(self):
        self.modbus_client = ModbusClient(
            method='rtu',
            port='/dev/ttyUSB4',
            baudrate=9600,
            timeout=1,
            parity='N',
            stopbits=1,
            bytesize=8
        )

        if self.modbus_client.connect():
            rospy.loginfo("Modbus连接已建立。")
        else:
            rospy.logerr("无法连接到Modbus服务器。")
            sys.exit("Modbus初始化失败。")

    # ============================================================
    # Modbus 控制
    # ============================================================
    def send_motor_command(self, slave_id, address, value):
        try:
            response = None
            for _ in range(3):
                response = self.modbus_client.write_register(address, value, unit=slave_id)
                rospy.sleep(0.1)

            if response is None or response.isError():
                rospy.logerr("发送命令到电机 %s 失败", str(slave_id))
                return False
            else:
                rospy.loginfo("命令已发送到电机 %s", str(slave_id))
                return True
        except Exception as e:
            rospy.logerr("发送命令到电机 %s 时出错: %s", str(slave_id), str(e))
            return False

    # ============================================================
    # MoveIt 工具：普通 Pose 规划
    # ============================================================
    def _plan_pose(self, group, pose_target):
        group.set_pose_target(pose_target)
        plan_ret = group.plan()

        success = False
        plan = None

        if isinstance(plan_ret, tuple):
            # 兼容 Noetic 常见返回: (success, plan, planning_time, error_code)
            if len(plan_ret) >= 2:
                success = bool(plan_ret[0])
                plan = plan_ret[1]
        else:
            plan = plan_ret
            try:
                success = hasattr(plan, "joint_trajectory") and len(plan.joint_trajectory.points) > 0
            except Exception:
                success = plan is not None

        return success, plan

    def _execute_pose_target(self, group, arm_name, pose_target,
                             vel_scale=None, acc_scale=None, wait_sec=0.0):
        if vel_scale is not None:
            group.set_max_velocity_scaling_factor(vel_scale)
        if acc_scale is not None:
            group.set_max_acceleration_scaling_factor(acc_scale)

        ok, plan = self._plan_pose(group, pose_target)
        if not ok or plan is None:
            rospy.logwarn("[%s] Pose规划失败。目标点: [%.4f, %.4f, %.4f]",
                          arm_name,
                          pose_target.position.x,
                          pose_target.position.y,
                          pose_target.position.z)
            group.clear_pose_targets()
            return False

        exec_ok = group.execute(plan, wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        if not exec_ok:
            rospy.logwarn("[%s] 轨迹执行失败。", arm_name)
            return False

        if wait_sec > 0:
            rospy.sleep(wait_sec)

        return True

    # ============================================================
    # MoveIt 工具：笛卡尔直线规划
    # 从当前位姿直线走到 pose_target
    # ============================================================
    def _execute_cartesian_target(self, group, arm_name, pose_target,
                                eef_step=0.005,
                                avoid_collisions=True,
                                min_fraction=0.90,
                                vel_scale=None,
                                acc_scale=None,
                                wait_sec=0.0):
        if vel_scale is not None:
            group.set_max_velocity_scaling_factor(vel_scale)
        if acc_scale is not None:
            group.set_max_acceleration_scaling_factor(acc_scale)

        waypoint = Pose()
        waypoint.position.x = pose_target.position.x
        waypoint.position.y = pose_target.position.y
        waypoint.position.z = pose_target.position.z
        waypoint.orientation = pose_target.orientation

        waypoints = [waypoint]

        try:
            plan, fraction = group.compute_cartesian_path(
                waypoints,
                eef_step,
                avoid_collisions
            )
        except Exception as e:
            rospy.logwarn("[%s] 笛卡尔路径规划异常: %s", arm_name, str(e))
            return False

        has_points = (
            plan is not None and
            hasattr(plan, "joint_trajectory") and
            len(plan.joint_trajectory.points) > 0
        )

        if (not has_points) or (fraction < min_fraction):
            rospy.logwarn("[%s] 笛卡尔路径规划失败，fraction=%.3f", arm_name, fraction)
            return False

        rospy.loginfo("[%s] 笛卡尔路径规划成功，fraction=%.3f", arm_name, fraction)

        exec_ok = group.execute(plan, wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        if not exec_ok:
            rospy.logwarn("[%s] 笛卡尔轨迹执行失败。", arm_name)
            return False

        if wait_sec > 0:
            rospy.sleep(wait_sec)

        return True

    # ============================================================
    # 命名位姿
    # ============================================================
    def move_to_home(self, group, arm_name):
        try:
            if arm_name == "left":
                named_target = "L_home"
            elif arm_name == "right":
                named_target = "R_home"
            else:
                rospy.logerr("[%s] 未知机械臂名称，无法回位。", arm_name)
                return False

            group.set_named_target(named_target)
            ok = group.go(wait=True)
            group.stop()
            group.clear_pose_targets()

            if not ok:
                rospy.logwarn("[%s] move_to_home 执行失败，目标位姿=%s", arm_name, named_target)
            else:
                rospy.loginfo("[%s] 已回到 %s", arm_name, named_target)

            return ok

        except Exception as e:
            rospy.logerr("[%s] move_to_home 异常: %s", arm_name, str(e))
            return False

    # ============================================================
    # 相机坐标 -> 机械臂坐标
    # ============================================================
    def cam_to_arm(self, arm_name, cam_point):
        if arm_name == "left":
            R_mat = self.left_R
            T_vec = self.left_T
        elif arm_name == "right":
            R_mat = self.right_R
            T_vec = self.right_T
        else:
            raise ValueError("未知 arm_name: {}".format(arm_name))

        goal_arm = np.dot(cam_point, R_mat.T) + T_vec
        return goal_arm

    # ============================================================
    # 姿态选择
    # ============================================================
    def get_quaternion_by_direction(self, direction):
        quaternions = {
            0: Quaternion(-0.166, 0.022746 ,-0.13645 ,0.97638), #zuo
            1: Quaternion(-0.17215, -0.022603, -0.12858, 0.97638),
            2: Quaternion(-0.17216, 0.022666, 0.12854, 0.97638),
            3: Quaternion(-0.17365, 0.00072894, 0.0041768, 0.9848),
        }
        return quaternions.get(direction, quaternions[0])

    # ============================================================
    # 执行流程
    # 左臂：到预夹持点(普通规划) -> 到夹持点(优先笛卡尔，失败退回普通规划) -> 保持15s -> 回位
    # 右臂：到预打叶点(普通规划) -> 到打叶点(优先笛卡尔，失败退回普通规划) -> 等待10s -> 回位
    # ============================================================
    def execute_plan(self, group, arm_name, goal, quaternion, motor_slave=None):
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        # 最终目标点
        target_offset = np.array([0.0, -0.00, 0.0])
        target_disp = rotation.apply(target_offset)
        pose_target_final = Pose(
            Point(goal[0] + target_disp[0],
                  goal[1] + target_disp[1],
                  goal[2] + target_disp[2]),
            quaternion
        )
        target_offset1 = np.array([0.0, -0.00, 0.0])
        target_disp1 = rotation.apply(target_offset1)
        pose_target_final1 = Pose(
            Point(goal[0] + target_disp1[0],
                  goal[1] + target_disp1[1],
                  goal[2] + target_disp1[2]),
            quaternion
        )
        # 预接近点：沿局部 y 方向退 9cm
        pre_offset = np.array([0.0, -0.09, 0.0])
        pre_disp = rotation.apply(pre_offset)
        pose_target_pre = Pose(
            Point(goal[0] + pre_disp[0],
                  goal[1] + pre_disp[1],
                  goal[2] + pre_disp[2]),
            quaternion
        )
        
        pre_offset1 = np.array([0.0, -0.05, 0.0])
        pre_disp1 = rotation.apply(pre_offset1)
        pose_target_pre1 = Pose(
            Point(goal[0] + pre_disp1[0],
                  goal[1] + pre_disp1[1],
                  goal[2] + pre_disp1[2]),
            quaternion
        )       
        

        if arm_name == "left":
            # 1. 先到预夹持点：普通Pose规划
            ok = self.send_motor_command(slave_id=motor_slave, address=0, value=25)
            ok = self._execute_pose_target(
                group, arm_name, pose_target_pre,
                vel_scale=1.0, acc_scale=1.0, wait_sec=0.5
            )
            if not ok:
                rospy.logwarn("[left] 到预夹持点失败。")
                return False

            # 2. 从预夹持点到夹持点：优先笛卡尔
            ok = self._execute_cartesian_target(
                group, arm_name, pose_target_final,
                eef_step=0.01,
                avoid_collisions=True,
                min_fraction=0.90,
                vel_scale=1,
                acc_scale=1,
                wait_sec=0.5
            )

            rospy.sleep(2.0)
            # # # 3. 笛卡尔失败则退回普通Pose规划
            if not ok:
                rospy.logwarn("[left] 笛卡尔到夹持点失败，尝试普通Pose规划。")
                ok = self._execute_pose_target(
                    group, arm_name, pose_target_final,
                    vel_scale=1, acc_scale=1, wait_sec=0.5
                )

            if not ok:
                rospy.logwarn("[left] 到夹持点失败，尝试回位。")
                self.move_to_home(group, arm_name)
                return False
            
            rospy.sleep(3.0)

            # # 左臂Modbus夹持示例，需要时取消注释
            ok = self.send_motor_command(slave_id=motor_slave, address=0, value=45)
            # rospy.sleep(0.5)
            # if not ok:
            #     rospy.logwarn("[left] Modbus夹持命令发送失败。")
            #     self.move_to_home(group, arm_name)
            #     return False

            # rospy.loginfo("[left] 已到夹持点，保持15秒...")
            # rospy.sleep(15.0)

            # ok = self.send_motor_command(slave_id=motor_slave, address=0, value=39)
            # rospy.sleep(1.5)
            # rospy.loginfo("[right] 右臂流程完成。")
            # back_ok = self.move_to_home(self.left_group, "left")
            # if not back_ok:
            #     rospy.logwarn("[left] 回位失败。")
            # rospy.sleep(3.5)
            # ok = self.send_motor_command(slave_id=motor_slave, address=0, value=25)

            # rospy.loginfo("[left] 左臂流程完成。")
            return True

        elif arm_name == "right":
            # 1. 先到预打叶点：普通Pose规划
            ok = self._execute_pose_target(
                group, arm_name, pose_target_pre1,
                vel_scale=1, acc_scale=1, wait_sec=0.5
            )
            if not ok:
                rospy.logwarn("[right] 到预打叶点失败。")
                return False

            # ok = self._execute_cartesian_target(
            #     group, arm_name, pose_target_final1,
            #     eef_step=0.01,
            #     avoid_collisions=True,
            #     min_fraction=0.90,
            #     vel_scale=0.3,
            #     acc_scale=0.3,
            #     wait_sec=0.5
            # )

            # 3. 笛卡尔失败则退回普通Pose规划
            # if not ok:
            #     rospy.logwarn("[right] 笛卡尔到打叶点失败，尝试普通Pose规划。")
            rospy.sleep(3.0)
            ok = self._execute_pose_target(
                group, arm_name, pose_target_final1,
                vel_scale=0.3, acc_scale=0.3, wait_sec=0.5
            )

            if not ok:
                rospy.logwarn("[right] 到打叶点失败，尝试回位。")
                self.move_to_home(group, arm_name)
                return False

            rospy.loginfo("[right] 已到达打叶点，等待10秒...")
            rospy.sleep(10.0)
            #ok = self.send_motor_command(slave_id=motor_slave, address=0, value=25)
            

            back_ok = self.move_to_home(group, arm_name)
            
            if not back_ok:
                rospy.logwarn("[right] 回位失败。")
                return False
            
            #back_ok = self.move_to_home(self.left_group, "left")
            rospy.sleep(3.0)
            ok = self.send_motor_command(slave_id=motor_slave, address=0, value=25)
            back_ok = self.move_to_home(self.left_group, "left")
            
            if not back_ok:
                rospy.logwarn("[left] 回位失败。")
            #rospy.sleep(3.5)
            #ok = self.send_motor_command(slave_id=motor_slave, address=0, value=25)           
            
          
            
            return True

        else:
            rospy.logerr("未知机械臂 arm_name=%s", arm_name)
            return False

    # ============================================================
    # 左右臂任务
    # ============================================================
    def run_left_arm(self):
        cam_point = self.left_cam_point.copy()
        direction = self.left_direction
        quaternion = self.get_quaternion_by_direction(direction)
        # goal_arm = self.cam_to_arm("left", cam_point)
        goal_arm=np.array([0.30657, 0.49624, 0.44642], dtype=float)
        self.execute_plan(self.left_group, "left", goal_arm, quaternion, motor_slave=4)

    def run_right_arm(self):
        cam_point = self.right_cam_point.copy()
        direction = self.right_direction
        quaternion = self.get_quaternion_by_direction(direction)
        # goal_arm = self.cam_to_arm("right", cam_point)
        goal_arm=np.array([0.53773,0.48312, 0.20045], dtype=float)
        self.execute_plan(self.right_group, "right", goal_arm, quaternion, motor_slave=4)

    # def run_both_arms(self):
    #     left_thread = threading.Thread(target=self.run_left_arm)
    #     right_thread = threading.Thread(target=self.run_right_arm)

    #     left_thread.start()
    #     right_thread.start()

    #     left_thread.join()
    #     right_thread.join()

    #     rospy.loginfo("双臂任务执行完成。")
    def run_both_arms(self):
        rospy.loginfo("===== 左臂开始 =====")
        self.run_left_arm()

        rospy.sleep(3.0)

        rospy.loginfo("===== 右臂开始 =====")
        self.run_right_arm()

        rospy.loginfo("双臂任务执行完成。")

if __name__ == '__main__':
    rospy.init_node('dual_arm_manual_demo')
    demo = DualArmManualDemo()

    rospy.sleep(1.0)

    # 直接执行左右臂
    demo.run_both_arms()

    rospy.spin()