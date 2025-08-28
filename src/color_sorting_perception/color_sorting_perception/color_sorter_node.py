import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2, numpy as np

def linmap(v,a,b,c,d):
    return c + (v-a)*(d-c)/float(b-a) if b!=a else (c+d)/2.0

class ColorSorter(Node):
    def __init__(self):
        super().__init__('color_sorter')
        self.declare_parameter('image_topic', '/overhead/overhead_cam/image')
        self.declare_parameter('u_min', 0); self.declare_parameter('u_max', 639)
        self.declare_parameter('v_min', 0); self.declare_parameter('v_max', 479)
        self.declare_parameter('x_min', -0.20); self.declare_parameter('x_max', 0.20)
        self.declare_parameter('y_min', -0.20); self.declare_parameter('y_max', 0.20)
        self.declare_parameter('pick_z', 0.90)
        self.declare_parameter('min_area', 300)

        self.bridge = CvBridge()
        self.pub_color = self.create_publisher(String, '/detected_object_color', 10)
        self.pub_pose  = self.create_publisher(PoseStamped, '/detected_pick_pose', 10)
        self.pub_dbg   = self.create_publisher(Image, '/color_sorting/debug_image', 10)
        self.sub = self.create_subscription(Image, self.get_parameter('image_topic').value, self.cb, 10)
        self.get_logger().info('Color sorter running.')

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        red = cv2.inRange(hsv, (0,80,50), (10,255,255)) | cv2.inRange(hsv,(170,80,50),(180,255,255))
        blue= cv2.inRange(hsv, (95,80,50),(135,255,255))

        def pick(mask):
            cnts,_=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts: return None,0,(0,0)
            c=max(cnts,key=cv2.contourArea); a=cv2.contourArea(c)
            M=cv2.moments(c); 
            return (c,a,(int(M['m10']/M['m00']),int(M['m01']/M['m00']))) if M['m00']!=0 else (None,0,(0,0))

        rc,ra,rcxy=pick(red); bc,ba,bcxy=pick(blue)

        min_area=int(self.get_parameter('min_area').value)
        color=None; cxy=(0,0)
        if ra>=ba and ra>=min_area: color='red'; cxy=rcxy
        elif ba>=min_area: color='blue'; cxy=bcxy

        if color:
            u_min=self.get_parameter('u_min').value; u_max=self.get_parameter('u_max').value
            v_min=self.get_parameter('v_min').value; v_max=self.get_parameter('v_max').value
            x_min=self.get_parameter('x_min').value; x_max=self.get_parameter('x_max').value
            y_min=self.get_parameter('y_min').value; y_max=self.get_parameter('y_max').value
            z=self.get_parameter('pick_z').value
            u,v=cxy
            xw=linmap(u,u_min,u_max,x_min,x_max)
            yw=linmap(v,v_min,v_max,y_max,y_min)
            ps=PoseStamped(); ps.header=msg.header; ps.header.frame_id='world'
            ps.pose.position.x=float(xw); ps.pose.position.y=float(yw); ps.pose.position.z=float(z)
            ps.pose.orientation.w=1.0
            self.pub_color.publish(String(data=color))
            self.pub_pose.publish(ps)

        dbg = img.copy()
        if rc is not None: cv2.drawContours(dbg,[rc],-1,(0,0,255),2)
        if bc is not None: cv2.drawContours(dbg,[bc],-1,(255,0,0),2)
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg,'bgr8'))

def main():
    rclpy.init(); n=ColorSorter(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
