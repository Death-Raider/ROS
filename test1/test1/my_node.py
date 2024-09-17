#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

# uncomment when building
# from test1.rotation import Rotation
# from test1.transformManager import TransfromManager
# from test1.wheels import Wheel

# comment out when building 
from rotation import Rotation
from transformManager import TransfromManager
from wheels import Wheel


class TransformPublisher(Node):
    def __init__(self,node_name):
        super().__init__(node_name)

        self.transform_manager_static = TransfromManager(node_cls = self, type = 'static')
        self.transform_manager_dynamic = TransfromManager(node_cls = self, type = 'dynamic')
        self.rotation = Rotation()
        self.wheels = [Wheel(str(i),f"wheel_{i}") for i in range(6)] # create 6 wheels
        self.wheel_rot_angles = np.zeros((6,3))
        self.init_body()

        self.timer = self.create_timer(0.1, self.rotate_wheels_sync)

    def init_body(self):
        # Set the body position of rover
        # B(5,0)
        self.transform_manager_static.set_transform('world','body', [1., 0., 0., 0., 0., 0., 1.], self.get_timestamp() )
        self.transform_manager_static.broadcast_transform()
        # Set the wheels around the body of the rover in lines assuming B is origin
        # w0(1,-1), w1(0,-1), w2(-1,-1), w3(1,1), w4(0,1), w5(-1,1),
        w = 0
        for i in range(-1,3,2):
            for j in range(3):
                pos_w = [1.0-j, float(i), 0., 0., 0., 0., 1.]
                self.wheels[w].pos = pos_w
                self.transform_manager_static.set_transform('body',f'wheel_{w}', pos_w, self.get_timestamp() )
                self.transform_manager_static.broadcast_transform()
                w+=1

    def get_timestamp(self):
        return self.get_clock().now().to_msg()

    def rotate_wheels_sync(self,angular_speed=0.1,direction=1,):
        for i in range(6):
            self.wheels[i].pos[3:] = self.rotation.euler_to_quaternion(*self.wheel_rot_angles[i]) # rotate in xz plane (about y axis)
            self.wheel_rot_angles[i][1] += direction*angular_speed
            self.transform_manager_dynamic.set_transform('body',f'wheel_{i}',self.wheels[i].pos, self.get_timestamp())
            self.transform_manager_dynamic.broadcast_transform()

    # def publish_dynamic_transform(self):
    #     self.transform_manager_dynamic.set_transform('body','obj', self.dynamic_pos, self.get_timestamp() )
    #     self.transform_manager_dynamic.broadcast_transform()
    #     self.update_pos() # updated self.dynamic_pos and self.angle
        
    # def update_pos(self):
    #     new_ang = self.rotation.euler_to_quaternion(*self.angle)
    #     self.angle[0] += 0.1
    #     self.angle[1] += 0.2
    #     self.angle[2] += 0.3
    #     self.dynamic_pos = [1.,0.,0.,*new_ang]

def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher('transform_publisher')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
