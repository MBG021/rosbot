#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty
from rclpy.time import Time

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i':(1,0),
    'o':(1,-1),
    'j':(0,1),
    'l':(0,-1),
    'u':(1,1),
    ',':(-1,0),
    '.':(-1,1),
    'm':(-1,-1),
}

speedBindings = {
    'q':(1.1,1.1),
    'z':(.9,.9),
    'w':(1.1,1),
    'x':(.9,1),
    'e':(1,1.1),
    'c':(1,.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        # Cambiamos el tipo de mensaje a TwistStamped
        self.pub = self.create_publisher(TwistStamped, '/tricycle_controller/cmd_vel', 10)

        self.speed = 0.2
        self.turn = 1.0
        self.target_speed = 0.0
        self.target_turn = 0.0
        self.control_speed = 0.0
        self.control_turn = 0.0
        self.status = 0

    def send_velocity(self, control_speed, control_turn):
        # Creamos el mensaje TwistStamped
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()  # Marca de tiempo actual
        twist_stamped.header.frame_id = "chasis"  # Podemos definir el frame_id aquí

        # Llenamos el mensaje Twist con los datos de velocidad
        twist_stamped.twist.linear.x = control_speed
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = control_turn

        # Publicamos el mensaje TwistStamped
        self.pub.publish(twist_stamped)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = TeleopNode()
    
    print(msg)
    print(vels(node.speed, node.turn))
    
    try:
        while True:
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                node.target_speed = node.speed * x
                node.target_turn = node.turn * th
            elif key in speedBindings.keys():
                node.speed = node.speed * speedBindings[key][0]
                node.turn = node.turn * speedBindings[key][1]
                print(vels(node.speed, node.turn))
            elif key == ' ' or key == 'k':
                node.target_speed = 0
                node.target_turn = 0
                node.control_speed = 0
                node.control_turn = 0
            else:
                if key == '\x03':  # Control-C to quit
                    break
            
            # Gradualmente ajustamos las velocidades
            if node.target_speed > node.control_speed:
                node.control_speed = min(node.target_speed, node.control_speed + 0.02)
            elif node.target_speed < node.control_speed:
                node.control_speed = max(node.target_speed, node.control_speed - 0.02)
            
            if node.target_turn > node.control_turn:
                node.control_turn = min(node.target_turn, node.control_turn + 0.1)
            elif node.target_turn < node.control_turn:
                node.control_turn = max(node.target_turn, node.control_turn - 0.1)
            
            node.send_velocity(node.control_speed, node.control_turn)

            rclpy.spin_once(node, timeout_sec=0.1)

    except Exception as e:
        print(e)

    finally:
        twist_stamped = TwistStamped()
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.angular.z = 0.0
        node.pub.publish(twist_stamped)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
