import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
TOPIC  = '/scaled_joint_trajectory_controller/joint_trajectory'  # requiere UR sim oficial con controladores

class Palletizer(Node):
    def __init__(self):
        super().__init__('palletizer')
        self.home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.over_pick = [-0.7, -1.3, 1.6, -1.8, -1.6, 0.0]
        self.pick      = [-0.7, -1.5, 1.8, -2.0, -1.6, 0.0]

        self.grid_left  = [[-1.1,-1.5,1.7,-1.7,-1.6,0.0],[-1.2,-1.5,1.7,-1.7,-1.6,0.0],[-1.3,-1.5,1.7,-1.7,-1.6,0.0],
                           [-1.1,-1.4,1.6,-1.7,-1.6,0.0],[-1.2,-1.4,1.6,-1.7,-1.6,0.0],[-1.3,-1.4,1.6,-1.7,-1.6,0.0],
                           [-1.1,-1.3,1.5,-1.7,-1.6,0.0],[-1.2,-1.3,1.5,-1.7,-1.6,0.0],[-1.3,-1.3,1.5,-1.7,-1.6,0.0]]
        self.grid_right = [[-p[0],p[1],p[2],p[3],p[4],p[5]] for p in self.grid_left]

        self.pub = self.create_publisher(JointTrajectory, TOPIC, 10)
        self.create_subscription(String, '/detected_object_color', self.color_cb, 10)
        self.create_subscription(PoseStamped, '/detected_pick_pose', self.pose_cb, 10)

        self.color=None; self.idx={'red':0,'blue':0}

    def send(self, pos, t):
        jt = JointTrajectory(); jt.joint_names = JOINTS
        pt = JointTrajectoryPoint(); pt.positions = pos; pt.time_from_start.sec=int(t)
        jt.points=[pt]; self.pub.publish(jt)

    def color_cb(self, m): self.color=m.data

    def pose_cb(self, ps):
        if self.color not in ['red','blue']: return
        self.get_logger().info(f'Picking {self.color} near ({ps.pose.position.x:.3f},{ps.pose.position.y:.3f})')
        self.send(self.over_pick,3); self.send(self.pick,3)
        # Aquí llamarías /ATTACHLINK (hazlo desde otra terminal por ahora)
        self.send(self.over_pick,2)
        grid = self.grid_left if self.color=='red' else self.grid_right
        i = self.idx[self.color] % len(grid)
        self.send(grid[i],4)
        # Aquí llamarías /DETACHLINK
        self.send(self.home,4)
        self.idx[self.color]+=1

def main():
    rclpy.init(); n=Palletizer(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
