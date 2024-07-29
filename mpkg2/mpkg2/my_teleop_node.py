import sys
import rclpy                        
from rclpy.node import Node          
from geometry_msgs.msg import Twist  
from rclpy.executors import ExternalShutdownException   
import argparse
from typing import List
from pythonosc import dispatcher
from pythonosc import osc_server
import threading



class MyTeleop(Node):   

    def printdata(self,address: str, *osc_arguments: List[str]):
        print(address + "  " + str(osc_arguments[0])) 
        if(address=="/angle"):
            self.rec_a=osc_arguments[0]
        if(address=="/enter" and osc_arguments[0]=="Enter"):
            self.rec_e="w"
            print(self.rec_a)
            self.angle=float(self.rec_a)
            if(self.angle<0):
                self.angle = self.angle+8*(self.angle/5+1)-12
            print(self.angle)
            server.shutdown()

    def __init__(self):   # コンストラクタ
        global dispatcher
        global server
        super().__init__('my_teleop_node')   

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vel = Twist() 

        #default
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.count=0
        self.rec_a=0
        self.rec_e=None
        ###
        parser = argparse.ArgumentParser()
        #publisher_IP?
        parser.add_argument("--ip",
            default="192.168.11.25", help="The ip to listen on")
        # port number?
        parser.add_argument("--port",
            type=int, default=7000, help="The port to listen on")
        args = parser.parse_args()
        # ?
        dispatcher = dispatcher.Dispatcher()
        dispatcher.map("/*", self.printdata)

        server = osc_server.ThreadingOSCUDPServer(
            (args.ip, args.port), dispatcher)
        print("Serving on {}".format(server.server_address))
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()

        #show
        #self.angle=float(input('input angle 5 or 10 or 15'))
        self.angle=float(self.rec_a)
        if(self.angle<0):
            self.angle = self.angle+8*(self.angle/5+1)-12
            print(self.angle)
        self.key=self.rec_e
        
    
    def timer_callback(self):  # タイマーのコールバック関数
        self.key=self.rec_e
        if self.key == 'w':
            self.vel.linear.x = 0.20
            self.count+=1

            if(self.count>90): #1 sec go straight
                self.vel.angular.z +=-(self.angle)*0.000055
        elif self.key == 's':
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        else:
            print('入力キーが違います．')
        if(self.count>180): #after 3 sec stop
            self.get_logger().info('stop')
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.timer.cancel()    
            
        self.publisher.publish(self.vel)  # 速度指令メッセージのパブリッシュ
        self.get_logger().info(f'並進速度={self.vel.linear.x} 角速度={self.vel.angular.z}')
        self.get_logger().info(f'count={self.count}')
        

def main():  # main関数
    rclpy.init()
    node = MyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')
    except ExternalShutdownException:
        print('ExternalShutdownException')
        sys.exit(1)
    finally:
        rclpy.try_shutdown()    
