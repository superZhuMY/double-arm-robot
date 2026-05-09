import sys
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_matrix
from scipy.spatial.transform import Rotation as R
import moveit_commander
import tf
import math
from moveit_exe.msg import object_poseAction, object_poseFeedback, object_poseResult
from moveit_msgs.msg import Constraints, JointConstraint
class IntegratedActionServer(object):
    def __init__(self):
        # self.server = actionlib.SimpleActionServer(
        #     'execute_pose',
        #     object_poseAction,
        #     execute_cb=self.execute_cb,
        #     auto_start=False
        # )
        
        self.T_cam_to_ee = self.quaternion_to_rotation_matrix()
        self.listener = tf.TransformListener()
        self.initialize_moveit()
        self.execute_cb([-0.219,0.044,0.834])
        # self.server.start()
    def get_initial_quaternion(self):
        # 获取机械臂当前姿态
        current_pose = self.move_group.get_current_pose().pose
        return current_pose.orientation
    def initialize_moveit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm_group")
        self.move_group.set_planner_id("RRTstar")
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.001)
        self.move_group.set_planning_time(3)
        self.move_group.allow_replanning(True)
        self.start_pose = self.move_group.get_current_pose().pose
        self.R=np.array([
    [ 0.7495618 , -0.01202532,  0.07330147],
    [-0.04212047,  0.02184922,  0.88085764],
    [ 0.0370993 , -0.97072637, -0.02285164]
])
        self.T=np.array([-0.16222035, -0.0056684, 0.49488549])

    def execute_cb(self, goal):


        quaternions = {
            1: Quaternion(0,0,0,1),
            2: Quaternion(0.380847961,0,0,0.924639810),
            3: Quaternion(0.35187713, -0.14716398, 0.35321087, 0.85426652),
        }
        # quaternions=self.get_initial_quaternion()

        # 依次尝试每个目标位置
        # for target in sorted_data:
        #     positions = target["positions"]
        #     rospy.loginfo("Attempting to plan path to positions: %s", positions)

        #     for attempt in range(2):  # 每个目标尝试两次
        #         if self.execute_plan(positions, quaternions[1]):
        #             return  # 如果成功，则返回

        #         rospy.loginfo("Attempt %d failed for positions: %s", attempt + 1, positions)
        #         self.move_to_start()
        goal1=np.dot(goal, self.R.T) +self.T
        print(goal1)
        goal1[0]=goal1[0]+0.132*math.tan(math.asin((goal1[0]+0.14234)/goal1[1]))+0.01
        goal1[2]=goal1[2]+0.02
        print(0.132*math.tan(math.asin((goal1[0]+0.14234)/goal1[1])))
        self.execute_plan(goal1, quaternions[1])
        # rospy.loginfo("All targets failed.")
        # result = object_poseResult()
        # result.result = 0
        # self.server.set_aborted(result)

    def execute_plan(self, goal, quaternion):
        # movements = np.array([0, -0.15, 0])
        # rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        # displacement=rotation.apply(movements)
        # waypoints = [self.move_group.get_current_pose().pose]
        # pose_target = Pose(Point(goal[0] + displacement[0], goal[1] + displacement[1], goal[2] + displacement[2]), quaternion)
        pose_target1 = Pose(Point(goal[0], goal[1], goal[2]), quaternion)
        
        # waypoints = [self.move_group.get_current_pose().pose]
        # waypoints.extend([pose_target1])
        # self.move_group.set_start_state_to_current_state()
        
        # start_time = rospy.Time.now().to_sec()
        
        # if(self.execute_cartesian_path(waypoints,0.01)):
        #     elapsed_time = rospy.Time.now().to_sec() - start_time
        #     print(elapsed_time)
        #     rospy.sleep(4)
        #     return        
        
        
        


        
        
        self.move_group.set_pose_target(pose_target1)
        plan = self.move_group.plan()  
        if plan[0]: 
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(5)
            self.move_group.set_pose_target(pose_target1)
            plan = self.move_group.plan() 
            
            
    def execute_cartesian_path(self, waypoints,speed):     
        maxtries = 20
        attempts = 0
        fraction = 0.0
        self.move_group.set_start_state_to_current_state()
        print(waypoints)
        
        while attempts < maxtries:
            attempts += 1
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, speed, False)
            
            if fraction < 1.0:
                rospy.logwarn(f"Attempt {attempts}: Incomplete planning, skipping waypoint.")
                if attempts == maxtries:
                    rospy.logwarn("Max attempts reached, skipping waypoint.")
                continue
            
            self.move_group.execute(plan, wait=True)
            print("zhixingchenggong")
            self.move_group.stop()        
            self.move_group.clear_pose_targets()
            return 1 
      


        
        
        
        

    def move_to_start(self):
        rospy.loginfo("Returning to start position.")
        self.move_group.set_pose_target(self.start_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def quaternion_to_rotation_matrix(self):
        q = [0.021406236993629982,-0.006979501046515124,0.017238838713636875,0.9995978601531315]
        rotation_matrix = quaternion_matrix(q)[:3, :3]
        T_cam_to_ee = np.eye(4)
        T_cam_to_ee[:3, :3] = rotation_matrix
        # T_cam_to_ee[:3, 3] = [-0.028239910736809412,  0.08457268458322861,  -0.03908552380538743]
        T_cam_to_ee[:3, 3] = [-0.028239910736809412,  0.08457268458322861,  -0.05908552380538743]
        return T_cam_to_ee

    def transform_point(self, point_camera, target_frame='base_link', source_frame_ee='pick'):
        try:
            self.listener.waitForTransform(target_frame, source_frame_ee, rospy.Time(0), rospy.Duration(4.0))
            (trans_ee_to_base, rot_ee_to_base) = self.listener.lookupTransform(target_frame, source_frame_ee, rospy.Time(0))   
            # 构建末端执行器到机械臂基座的变换矩阵
            T_ee_to_base = tf.transformations.quaternion_matrix(rot_ee_to_base)
            T_ee_to_base[:3, 3] = trans_ee_to_base
            # 将相机坐标系中的点转换为齐次坐标
            P_camera = np.array([point_camera[0], point_camera[1], point_camera[2], 1.0])
            # 转换到末端执行器坐标系
            P_ee = self.T_cam_to_ee @ P_camera
            # 转换到机械臂基座坐标系
            P_base = T_ee_to_base @ P_ee
            return P_base[:3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Could not get transform from {} to {}".format(source_frame_ee, target_frame))
            return None

if __name__ == '__main__':
    rospy.init_node('integrated_action_server')
    server = IntegratedActionServer()
    rospy.spin()