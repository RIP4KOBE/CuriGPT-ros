#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json
from srv import ExecuteGroupPose, ExecuteGroupPoseRequest


def transform_coordinates(bounding_box):
    # Placeholder for coordinate transformation logic
    # This should convert bounding_box coordinates to robot base coordinates
    # You might need actual transformation logic here depending on your setup
    return bounding_box  # This is a placeholder

def publish_grasp_pose_to_service(grasp_pose, log_info):

    # Wait for the service to be available
    rospy.wait_for_service('/panda_left/execute_create_ptp_cartesian_trajectory')

    try:
        # Create a service client
        service_client = rospy.ServiceProxy('/panda_left/execute_create_ptp_cartesian_trajectory', ExecuteGroupPose)

        # Create the service request
        service_request = ExecuteGroupPoseRequest()

        # Fill in the request
        service_request.header.seq = 0
        service_request.header.stamp = rospy.Time.now()  # Or rospy.Time(0) for current time
        service_request.header.frame_id = ''  # Set if needed
        service_request.group_name = ''  # Set if needed
        service_request.goal_type = 0  # Set if needed

        # Set the position and orientation from the grasp_pose
        service_request.goal.position = grasp_pose.position
        service_request.goal.orientation = grasp_pose.orientation

        service_request.tolerance = 25
        service_request.constraint = ''  # Set if needed

        # Call the service
        service_response = service_client(service_request)

        # Handle the response
        if not service_response.SUCCEEDED:
            rospy.loginfo(log_info)
        else:
            rospy.loginfo("Service call failed.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def softhand_grasp(synergy_control_parameters):
    # define softhand publisher
    softhand_grasp_pub = rospy.Publisher('/qbhand2m_chain/control/qbhand2m2_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)

    traj_l = JointTrajectory()
    traj_l.header.stamp = rospy.Time.now()
    traj_l.header.frame_id = " "
    traj_l.joint_names = ["qbhand2m2_manipulation_joint", "qbhand2m2_synergy_joint"]
    point = JointTrajectoryPoint()
    point.positions = [synergy_control_parameters[1], synergy_control_parameters[0]]
    point.time_from_start = rospy.Duration(1)
    traj_l.points.append(point)

    for i in range(50):
        softhand_grasp_pub.publish(traj_l)
        rospy.loginfo("softhand grasp execution...")
        # 短暂休眠，以确保消息被发送并且不会过载接收方
        rospy.sleep(0.1)  # 休眠100毫秒

    # rospy.sleep(10)

def grasp_and_place(arg1, arg2):

    rospy.init_node('grasp_and_place_node')
    rospy.loginfo("Starting grasp_and_place operation")
    rate = rospy.Rate(1)

    # Transform the coordinates from the bounding box to robot base coordinates
    grasp_point = transform_coordinates(arg1)
    place_point = transform_coordinates(arg2)

    # define pre-grasp pose
    pre_grasp_pose = Pose()
    pre_grasp_pose.position.x = 0.452
    pre_grasp_pose.position.y = 0.020
    pre_grasp_pose.position.z = 0.957
    pre_grasp_pose.orientation.x = 0.861
    pre_grasp_pose.orientation.y = -0.239
    pre_grasp_pose.orientation.z = 0.385
    pre_grasp_pose.orientation.w = -0.232

    # define grasp pose
    grasp_pose = Pose()
    grasp_pose.position.x = grasp_point[0]
    grasp_pose.position.y = grasp_point[1]
    grasp_pose.position.z = grasp_point[2]
    grasp_pose.orientation.x = 0.861
    grasp_pose.orientation.y = -0.239
    grasp_pose.orientation.z = 0.385
    grasp_pose.orientation.w = -0.232

    # define place pose
    place_pose = Pose()
    place_pose.position.x = place_point[0]
    place_pose.position.y = place_point[1]
    place_pose.position.z = place_point[2]
    place_pose.orientation.x = 0.861
    place_pose.orientation.y = -0.239
    place_pose.orientation.z = 0.385
    place_pose.orientation.w = -0.232

    # define softhand grasp synergy parameters
    close_synergy_control_parameters = [1.0, 0.0]
    open_synergy_control_parameters = [0.0, 0.0]

    # move to pre-grasp pose
    publish_grasp_pose_to_service(pre_grasp_pose, "Moving to pre-grasp pose...")

    # move to grasp pose
    publish_grasp_pose_to_service(grasp_pose, "Moving to grasp pose...")
    # rospy.sleep(3)

    # close softhand
    softhand_grasp(close_synergy_control_parameters)

    # move to place pose
    publish_grasp_pose_to_service(place_pose, "Moving to post-grasp pose...")
    # rospy.sleep(3)

    # open softhand
    softhand_grasp(open_synergy_control_parameters)

    # move back
    publish_grasp_pose_to_service(pre_grasp_pose, "Moving to pre-grasp pose...")

    rospy.loginfo("Grasp and place operation completed")

def grasp_and_give(arg1):

    rospy.init_node('grasp_and_give_node')
    rospy.loginfo("Starting grasp_and_give operation")
    rate = rospy.Rate(1)

    # Transform the coordinates from the bounding box to robot base coordinates
    grasp_point = transform_coordinates(arg1)

    # define pre-grasp pose
    pre_grasp_pose = Pose()
    pre_grasp_pose.position.x = 0.452
    pre_grasp_pose.position.y = 0.020
    pre_grasp_pose.position.z = 0.957
    pre_grasp_pose.orientation.x = 0.861
    pre_grasp_pose.orientation.y = -0.239
    pre_grasp_pose.orientation.z = 0.385
    pre_grasp_pose.orientation.w = -0.232

    # define grasp pose
    grasp_pose = Pose()
    grasp_pose.position.x = grasp_point[0]
    grasp_pose.position.y = grasp_point[1]
    grasp_pose.position.z = grasp_point[2]
    grasp_pose.orientation.x = 0.861
    grasp_pose.orientation.y = -0.239
    grasp_pose.orientation.z = 0.385
    grasp_pose.orientation.w = -0.232

    # define give pose
    give_pose = Pose()
    give_pose.position.x = 0.452
    give_pose.position.y = 0.452
    give_pose.position.z = 0.452
    give_pose.orientation.x = 0.861
    give_pose.orientation.y = -0.239
    give_pose.orientation.z = 0.385
    give_pose.orientation.w = -0.232

    # define softhand grasp synergy parameters
    close_synergy_control_parameters = [1.0, 0.0]
    open_synergy_control_parameters = [0.0, 0.0]

    # move to pre-grasp pose
    publish_grasp_pose_to_service(pre_grasp_pose, "Moving to pre-grasp pose...")

    # move to grasp pose
    publish_grasp_pose_to_service(grasp_pose, "Moving to grasp pose...")
    # rospy.sleep(3)

    # close softhand
    softhand_grasp(close_synergy_control_parameters)

    # move to give pose
    publish_grasp_pose_to_service(give_pose, "Moving to give pose...")
    # rospy.sleep(3)

    # open softhand
    softhand_grasp(open_synergy_control_parameters)

    # move back
    publish_grasp_pose_to_service(pre_grasp_pose, "Moving back to pre-grasp pose...")
    rospy.loginfo("Grasp and give operation completed")

def grasp_handover_place(arg1, arg2):

    rospy.init_node('grasp_handover_place_node')
    rospy.loginfo("Starting grasp_handover_place operation")
    rate = rospy.Rate(1)

    # Transform the coordinates from the bounding box to robot base coordinates
    grasp_point = transform_coordinates(arg1)
    place_point = transform_coordinates(arg2)

    # define pre-grasp pose
    pre_grasp_pose = Pose()
    pre_grasp_pose.position.x = 0.452
    pre_grasp_pose.position.y = 0.020
    pre_grasp_pose.position.z = 0.957
    pre_grasp_pose.orientation.x = 0.861
    pre_grasp_pose.orientation.y = -0.239
    pre_grasp_pose.orientation.z = 0.385
    pre_grasp_pose.orientation.w = -0.232

    # define grasp pose
    grasp_pose = Pose()
    grasp_pose.position.x = grasp_point[0]
    grasp_pose.position.y = grasp_point[1]
    grasp_pose.position.z = grasp_point[2]
    grasp_pose.orientation.x = 0.861
    grasp_pose.orientation.y = -0.239
    grasp_pose.orientation.z = 0.385
    grasp_pose.orientation.w = -0.232

    # define place pose
    place_pose = Pose()
    place_pose.position.x = place_point[0]
    place_pose.position.y = place_point[1]
    place_pose.position.z = place_point[2]
    place_pose.orientation.x = 0.861
    place_pose.orientation.y = -0.239
    place_pose.orientation.z = 0.385
    place_pose.orientation.w = -0.232

    # define softhand grasp synergy parameters
    close_synergy_control_parameters = [1.0, 0.0]
    open_synergy_control_parameters = [0.0, 0.0]

    # move to pre-grasp pose
    publish_grasp_pose_to_service(pre_grasp_pose, "Moving to pre-grasp pose...")

    # move to grasp pose
    publish_grasp_pose_to_service(grasp_pose, "Moving to grasp pose...")
    # rospy.sleep(3)

    # close softhand
    softhand_grasp(close_synergy_control_parameters)

    # move to place pose
    publish_grasp_pose_to_service(place_pose, "Moving to post-grasp pose...")
    # rospy.sleep(3)

    # open softhand
    softhand_grasp(open_synergy_control_parameters)

    # move back
    publish_grasp_pose_to_service(pre_grasp_pose, "Moving to pre-grasp pose...")

    rospy.loginfo("Grasp and place operation completed")



if __name__ == '__main__':
    grasp_and_place([0.1, 0.2, 0.3], [0.4, 0.5, 0.6])
    rospy.spin()
    rospy.loginfo("Shutting down grasp_and_place operation")