import rclpy
import socket
import struct

from rclpy.lifecycle  import LifecycleNode
from rclpy.lifecycle  import TransitionCallbackReturn
from rclpy.qos        import QoSProfile

from uvms_msg_pkg.msg import HalInertialnaviMsg
from uvms_msg_pkg.msg import HalDvlMsg
from uvms_msg_pkg.msg import HalDepthsensorMsg
from uvms_msg_pkg.msg import HalMainthrusterMsg
from uvms_msg_pkg.msg import HalAuxithrusterMsg
from uvms_msg_pkg.msg import HalBatteryMsg
from uvms_msg_pkg.msg import HalTailservoMsg
from uvms_msg_pkg.msg import HalWingservoMsg
# from uvms_msg_pkg.msg import HalArmcabinmotorMsg
# from uvms_msg_pkg.msg import HalArmmotorMsg
# from uvms_msg_pkg.msg import HalArmcontrollerMsg
# from uvms_msg_pkg.msg import HalAntennaMsg

class BspCommNode(LifecycleNode):
    def __init__(self):
        super().__init__('bsp_comm_node')

        # 节点消息订阅参数
        self.inertialnavi_sub   = None
        self.inertialnavi_data  = None
        self.dvl_sub            = None
        self.dvl_data           = None
        self.depthsensor_sub    = None
        self.depthsensor_data   = None
        self.mainthruster_sub   = None
        self.mainthruster_data  = None
        self.auxithruster_sub   = None
        self.auxithruster_data  = None
        self.battery_sub        = None
        self.battery_data       = None
        self.tailservo_sub      = None
        self.tailservo_data     = None
        self.wingservo_sub      = None
        self.wingservo_data     = None
        # self.armcabinmotor_sub  = None
        # self.armcabinmotor_data = None
        # self.armmotor_sub       = None
        # self.armmotor_data      = None
        # self.armcontroller_sub  = None
        # self.armcontroller_data = None
        # self.antenna_sub        = None
        # self.antenna_data       = None
        
        # UDP通信参数
        self.udp_socket = None
        self.udp_timer  = None
        self.udp_ip     = '192.168.1.30'  
        self.udp_port   = 5000      
        
        self.node_active     = False
        self.udpcomm_enabled = False
        
        # 通信数据帧参数
        self.frame_header     = 0x55AA
        self.inertialnavi_id  = 0x01
        self.dvl_id           = 0x02
        self.depthsensor_id   = 0x03
        self.mainthruster_id  = 0x04
        self.auxithruster_id  = 0x05
        self.battery_id       = 0x06
        self.tailservo_id     = 0x07
        self.wingservo_id     = 0x08
        # self.armcabinmotor_id = 0x09
        # self.armmotor_id      = 0x10
        # self.armcontroller_id = 0x11
        # self.antenna_id       = 0x12

        # self.get_logger().info('bsp_comm_node lifecycle node created.')

    def on_configure(self, state):
        # self.get_logger().info('Configuring bsp_comm_node...')

        try:
            qos_profile = QoSProfile(depth=10)

            self.inertialnavi_sub   = self.create_subscription(HalInertialnavi,'/hal/inertialnavi',self.inertialnavi_callback,qos_profile)
            self.dvl_sub            = self.create_subscription(HalDvl,'/hal/dvl',self.dvl_callback,qos_profile)
            self.depthsensor_sub    = self.create_subscription(HalDepthsensor,'/hal/depthsensor',self.depthsensor_callback,qos_profile)
            self.mainthruster_sub   = self.create_subscription(HalMainthruster,'/hal/mainthruster',self.mainthruster_callback,qos_profile)
            self.auxithruster_sub   = self.create_subscription(HalAuxithruster,'/hal/auxithruster',self.auxithruster_callback,qos_profile)
            self.battery_sub        = self.create_subscription(HalBattery,'/hal/battery',self.battery_callback,qos_profile)
            self.tailservo_sub      = self.create_subscription(HalTailservo,'/hal/tailservo',self.tailservo_callback,qos_profile)
            self.wingservo_sub      = self.create_subscription(HalWingservo,'/hal/wingservo',self.wingservo_callback,qos_profile)
            # self.armcabinmotor_sub  = self.create_subscription(HalArmcabinmotor,'/hal/armcabinmotor',self.armcabinmotor_callback,qos_profile)
            # self.armmotor_sub       = self.create_subscription(HalArmmotor,'/hal/armmotor',self.armmotor_callback,qos_profile)
            # self.armcontroller_sub  = self.create_subscription(HalArmcontroller,'/hal/armcontroller',self.armcontroller_callback,qos_profile)
            # self.antenna_sub        = self.create_subscription(HalAntenna,'/hal/antenna',self.antenna_callback,qos_profile)

            self.inertialnavi_data  = None
            self.dvl_data           = None
            self.depthsensor_data   = None
            self.mainthruster_data  = None
            self.auxithruster_data  = None
            self.battery_data       = None
            self.tailservo_data     = None
            self.wingservo_data     = None
            # self.armcabinmotor_data = None
            # self.armmotor_data      = None
            # self.armcontroller_data = None
            # self.antenna_data       = None
            
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_timer  = self.create_timer(0.02, self.udpcomm_callback)    

            # self.get_logger().info('Inertial subscriber created.')
            # self.get_logger().info(f'UDP socket created, target = {self.udp_ip}:{self.udp_port}')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            # self.get_logger().error(f'Failed to configure: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        # self.get_logger().info('Activating bsp_comm_node...')
        self.node_active     = True
        self.udpcomm_enabled = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        # self.get_logger().info('Deactivating bsp_comm_node...')
        self.node_active     = False
        self.udpcomm_enabled = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        # self.get_logger().info('Cleaning up bsp_comm_node...')
        self.node_active       = False
        self.udpcomm_enabled   = False
        
        self.inertialnavi_sub   = None
        self.inertialnavi_data  = None
        self.dvl_sub            = None
        self.dvl_data           = None
        self.depthsensor_sub    = None
        self.depthsensor_data   = None
        self.mainthruster_sub   = None
        self.mainthruster_data  = None
        self.auxithruster_sub   = None
        self.auxithruster_data  = None
        self.battery_sub        = None
        self.battery_data       = None
        self.tailservo_sub      = None
        self.tailservo_data     = None
        self.wingservo_sub      = None
        self.wingservo_data     = None
        # self.armcabinmotor_sub  = None
        # self.armcabinmotor_data = None
        # self.armmotor_sub       = None
        # self.armmotor_data      = None
        # self.armcontroller_sub  = None
        # self.armcontroller_data = None
        # self.antenna_sub        = None
        # self.antenna_data       = None
                
        if self.udp_socket is not None:
            self.udp_socket.close()
            self.udp_socket = None
            
        if self.udp_timer is not None:
            self.udp_timer.cancel()
            self.udp_timer = None
            
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        # self.get_logger().info('Shutting down bsp_comm_node...')
        self.node_active     = False
        self.udpcomm_enabled = False
        
        if self.udp_socket is not None:
            self.udp_socket.close()
            self.udp_socket = None
            
        if self.udp_timer is not None:
            self.udp_timer.cancel()
            self.udp_timer = None
            
        return TransitionCallbackReturn.SUCCESS

    # ========================
    # 各节点消息订阅函数  
    # ========================
    def inertialnavi_callback(self, msg: HalInertialnavi):
        if not self.node_active:
            return
        self.inertialnavi_data = msg
        
    def dvl_callback(self, msg: HalDvl):
        if not self.node_active:
            return
        self.dvl_data = msg
        
    def depthsensor_callback(self, msg: HalDepthsensor):
        if not self.node_active:
            return
        self.depthsensor_data = msg
        
    def mainthruster_callback(self, msg: HalMainthruster):
        if not self.node_active:
            return
        self.mainthruster_data = msg
  
    def auxithruster_callback(self, msg: HalAuxithruster):
        if not self.node_active:
            return
        self.auxithruster_data = msg
        
    def battery_callback(self, msg: HalBattery):
        if not self.node_active:
            return
        self.battery_data = msg
        
    def tailservo_callback(self, msg: HalTailservo):
        if not self.node_active:
            return
        self.tailservo_data = msg
        
    def wingservo_callback(self, msg: HalWingservo):
        if not self.node_active:
            return
        self.wingservo_data = msg
        
    # def armcabinmotor_callback(self, msg: HalArmcabinmotor)
        # if not self.node_active:
            # return
        # self.armcabinmotor_data = msg
        
    # def armmotor_callback(self, msg: HalArmmotor)
        # if not self.node_active:
            # return
        # self.armmotor_data = msg
        
    # def armcontroller_callback(self, msg: HalArmcontroller)
        # if not self.node_active:
            # return
        # self.armcontroller_data = msg
        
    # def antenna_callback(self, msg: HalAntenna)
        # if not self.node_active:
            # return
        # self.antenna_data = msg

    # ========================
    # 各节点消息打包函数  
    # ========================
    def inertialnavi_pack(self, msg: HalInertialnavi) -> bytes:
        payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        return payload
        
    def dvl_pack(self, msg: HalDvl) -> bytes:
        payload = struct.pack('<qfff',msg.timestamp,msg.velocity_x,msg.velocity_y,msg.velocity_z)
        return payload
        
    def depthsensor_pack(self, msg: HalDepthsensor) -> bytes:
        payload = struct.pack('<qfHfHf',msg.timestamp,msg.depth_1,msg.temp_1,msg.depth_2,msg.temp_2,msg.depth_avg)
        return payload
        
    def mainthruster_pack(self, msg: HalHalMainthruster) -> bytes:
        payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        return payload
        
    def auxithruster_pack(self, msg: HalAuxithruster) -> bytes:
        payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        return payload        

    def battery_pack(self, msg: HalBattery) -> bytes:
        payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        return payload 

    def tailservo_pack(self, msg: HalTailservo) -> bytes:
        payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        return payload 

    # def wingservo_pack(self, msg: HalWingservo) -> bytes:
        # payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        # return payload 

    # def armcabinmotor_pack(self, msg: HalArmcabinmotor) -> bytes:
        # payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        # return payload 

    # def armmotor_pack(self, msg: HalArmmotor) -> bytes:
        # payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        # return payload
        
    # def armcontroller_pack(self, msg: HalArmcontroller) -> bytes:
        # payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        # return payload
        
    # def antenna_pack(self, msg: HalAntenna) -> bytes:
        # payload = struct.pack('<qfffddfff',msg.timestamp_ms,msg.yaw,msg.pitch,msg.roll,msg.latitude,msg.longitude,msg.velocity_east,msg.velocity_north,msg.velocity_up)
        # return payload 

    # ========================
    # 通信数据打包函数  
    # ========================        
    def build_packet(self, msg_id: int, payload: bytes) -> bytes:
        packet = struct.pack('<HBH',self.frame_header,msg_id,len(payload)) + payload
        return packet

    # ========================
    # UDP通信函数  
    # ========================         
    def udpcomm_callback(self):
        if not self.node_active:
            return
        if not self.udpcomm_enabled:
            return
        if self.inertial_data is None:
            return
        if self.udp_socket is None:
            return

        try:
            inertialnavi_msg = self.inertialnavi_data
            inertialnavi_payload = self.inertial_pack(inertialnavi_msg)
            inertialnavi_packet = self.build_packet(self.inertialnavi_id,inertialnavi_payload)
            
            dvl_msg = self.dvl_data
            dvl_payload = self.dvl_pack(dvl_msg)
            dvl_packet = self.build_packet(self.dvl_id,dvl_payload)    
              
            depthsensor_msg = self.depthsensor_data
            depthsensor_payload = self.depthsensor_pack(depthsensor_msg)
            depthsensor_packet = self.build_packet(self.depthsensor_id,depthsensor_payload)
            
            mainthruster_msg = self.mainthruster_data
            mainthruster_payload = self.mainthruster_pack(mainthruster_msg)
            mainthruster_packet = self.build_packet(self.mainthruster_id,mainthruster_payload)
            
            auxithruster_msg = self.auxithruster_data
            auxithruster_payload = self.auxithruster_pack(auxithruster_msg)
            auxithruster_packet = self.build_packet(self.auxithruster_id,auxithruster_payload)
            
            battery_msg = self.battery_data
            battery_payload = self.battery_pack(battery_msg)
            battery_packet = self.build_packet(self.battery_id,battery_payload)  
            
            tailservo_msg = self.tailservo_data
            tailservo_payload = self.tailservo_pack(tailservo_msg)
            tailservo_packet = self.build_packet(self.tailservo_id,tailservo_payload)  
            
            wingservo_msg = self.wingservo_data
            wingservo_payload = self.wingservo_pack(wingservo_msg)
            wingservo_packet = self.build_packet(self.wingservo_id,wingservo_payload)  
            
            # armcabinmotor_msg = self.armcabinmotor_data
            # armcabinmotor_payload = self.armcabinmotor_pack(armcabinmotor_msg)
            # armcabinmotor_packet = self.build_packet(self.armcabinmotor_id,armcabinmotor_payload)  
            
            # armmotor_msg = self.armmotor_data
            # armmotor_payload = self.armmotor_pack(armmotor_msg)
            # armmotor_packet = self.build_packet(self.armmotor_id,armmotor_payload)
            
            # armcontroller_msg = self.armcontroller_data
            # armcontroller_payload = self.armcontroller_pack(armcontroller_msg)
            # armcontroller_packet = self.build_packet(self.armcontroller_id,armcontroller_payload)  
            
            # antenna_msg = self.antenna_data
            # antenna_payload = self.antenna_pack(antenna_msg)
            # antenna_packet = self.build_packet(self.antenna_id,antenna_payload)                   
                  
            self.udp_socket.sendto(inertialnavi_packet,(self.udp_ip, self.udp_port))
            self.udp_socket.sendto(dvl_packet,(self.udp_ip, self.udp_port))
            self.udp_socket.sendto(depthsensor_packet,(self.udp_ip, self.udp_port))
            self.udp_socket.sendto(mainthruster_packet,(self.udp_ip, self.udp_port))
            self.udp_socket.sendto(auxithruster_packet,(self.udp_ip, self.udp_port))
            self.udp_socket.sendto(battery_packet,(self.udp_ip, self.udp_port))
            self.udp_socket.sendto(tailservo_packet,(self.udp_ip, self.udp_port))
            self.udp_socket.sendto(wingservo_packet,(self.udp_ip, self.udp_port))
            # self.udp_socket.sendto(armcabinmotor_packet,(self.udp_ip, self.udp_port))
            # self.udp_socket.sendto(armmotor_packet,(self.udp_ip, self.udp_port))
            # self.udp_socket.sendto(armcontroller_packet,(self.udp_ip, self.udp_port))
            # self.udp_socket.sendto(antenna_packet,(self.udp_ip, self.udp_port))

        except Exception as e:
            self.get_logger().error(f'UDP send failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BspCommNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
