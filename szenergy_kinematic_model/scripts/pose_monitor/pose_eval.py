'''
Created on Jun 12, 2020

@author: kyberszittya
'''

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

import datetime
import math

class PoseEvaluator(object):
    
    def euclideanError(self, p0, p1):
        dx = (p0.pose.position.x - p1.pose.position.x)**2
        dy = (p0.pose.position.y - p1.pose.position.y)**2
        return math.sqrt(dx + dy)
    
    def __init__(self):
        self.gps_pose = PoseStamped()
        self.gps_nova_pose = PoseStamped()
        self.gps_file = open("./data/eval_{0}.csv".format(datetime.datetime.now().strftime("%Y-%m-%d_%H_%M_%S")), 'w')
        self.gps_file.write("gps_x;gps_y;gps_yaw;ndt_x;ndt_y;ndt_yaw;euclidean_error;yaw_error;ref_error\n".format())
        self.sub_pose_gnss = rospy.Subscriber("/gnss_pose", PoseStamped, self.cbPoseStampedGnssPose)
        self.sub_pose_gnss = rospy.Subscriber("/gps/nova/current_pose", PoseStamped, self.cbPoseStampedNova)
        self.sub_pose_ndt = rospy.Subscriber("/ndt_pose", PoseStamped, self.cbPoseStampedNdtPose)
        
        
    def cbPoseStampedGnssPose(self, data):
        self.gps_pose = data
        
    def cbPoseStampedNova(self, data):
        self.gps_ref = data
          
    def cbPoseStampedNdtPose(self, data):
        self.ndt_pose = data
        error = self.euclideanError(self.gps_pose, self.ndt_pose)
        ref_error = self.euclideanError(self.gps_ref, self.ndt_pose)
        o1 = self.gps_pose.pose.orientation
        o2 = self.ndt_pose.pose.orientation
        _, _, yaw = euler_from_quaternion([o1.x, o1.y, o1.z, o1.w])
        _, _, yaw_ndt = euler_from_quaternion([o2.x, o2.y, o2.z, o2.w])
        yaw_error = abs(yaw - yaw_ndt)
        self.gps_file.write("{0};{1};{2};{3};{4};{5};{6};{7};{8}\n".format(
            self.gps_pose.pose.position.x,
            self.gps_pose.pose.position.y,
            yaw,
            self.ndt_pose.pose.position.x,
            self.ndt_pose.pose.position.y,
            yaw_ndt,            
            error,
            yaw_error,
            ref_error
            )
        )
        
    def close(self):
        self.gps_file.close()
        

def main():
    rospy.init_node("pose_evaluator")
    peval = PoseEvaluator()
    rospy.spin()
    peval.close()
    
if __name__=="__main__":
    main()