import sys
import rclpy                        
from rclpy.node import Node          
from geometry_msgs.msg import Twist  
from rclpy.executors import ExternalShutdownException   


class MyTeleop(Node):  
    def __init__(self):   # コンストラクタ
        super().__init__('my_teleop_node')   

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vel = Twist() 

        #default
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.count=0

        #show
        self.angle=float(input('input angle 5 or 10 or 15'))
        self.key=input('if you want to go, press w. else s')

    def timer_callback(self):  # タイマーのコールバック関数
        #key = input('w, sキー入力後にEnterキーを押下 <<')  
        # キー取得
        # キーの値により並進速度や角速度を変更
        #angle=float(input('input angle 0.1-0.3'))
        #key=input('w de zenshin s ha stop')

        if self.key == 'w':
            self.vel.linear.x = 0.05

            if(self.count>10): #1 sec go straight
                self.vel.angular.z +=self.angle*0.002
        elif self.key == 's':
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        else:
            print('入力キーが違います．')
        if(self.count>30): #after 3 sec stop
            self.get_logger().info('stop')
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.timer.cancel()    
            
        self.publisher.publish(self.vel)  # 速度指令メッセージのパブリッシュ
        self.get_logger().info(f'並進速度={self.vel.linear.x} 角速度={self.vel.angular.z}')
        self.count+=1
        


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
