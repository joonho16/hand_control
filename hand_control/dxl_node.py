#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.node import Node
from dynamixel_sdk import *
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import JointState

class DxlControlNode(Node):
    def __init__(self):
        super().__init__('dxl_control_node')

        # --- [1] 다이나믹셀 설정 ---
        self.port_name = '/dev/ttyUSB0'
        self.baudrate = 4000000
        self.protocol_version = 2.0
        
        self.dxl_ids = [31, 32, 33, 34, 35, 36, 37, 38]
        self.aa_ids = [31, 33, 35, 37]
        self.fe_ids = [32, 34, 36, 38]
        
        self.joint_name_map = {
            31: "finger1_AA", 32: "finger1_FE",
            33: "finger2_AA", 34: "finger2_FE",
            35: "finger3_AA", 36: "finger3_FE",
            37: "finger4_AA", 38: "finger4_FE"
        }
        self.name_to_id_map = {v: k for k, v in self.joint_name_map.items()}
        
        # [수정됨] 각 ID별 제어 범위 개별 설정 (Dictionary 사용)
        # type: 'AA' 또는 'FE'
        # AA인 경우: 'min', 'max' 사용
        # FE인 경우: 'open', 'close' 사용
        self.joint_limits = {
            # Finger 1 (Index)
            31: {'type': 'AA', 'min': -100, 'max': 1000},
            32: {'type': 'FE', 'open': 0,    'close': -4560},
            
            # Finger 2 (Middle)
            33: {'type': 'AA', 'min': -2000, 'max': 2000},
            34: {'type': 'FE', 'open': 0,    'close': -4560}, 
            
            # Finger 3 (Ring)
            35: {'type': 'AA', 'min': -2000, 'max': 2000},
            36: {'type': 'FE', 'open': 0,    'close': -4560},
            
            # Finger 4 (Pinky)
            37: {'type': 'AA', 'min': -2000, 'max': 2000},
            38: {'type': 'FE', 'open': 0,    'close': -4560},
        }

        # 컨트롤 테이블 주소
        self.addr_operating_mode = 11
        self.addr_homing_offset = 20
        self.addr_torque_enable = 64
        self.addr_hardware_error = 70
        self.addr_goal_current = 102
        self.addr_goal_position = 116
        self.addr_present_position = 132
        
        self.len_present_position = 4
        self.len_goal_position = 4
        
        # 값 설정
        self.mode_current_based_position = 5
        self.torque_enable = 1
        self.torque_disable = 0
        self.default_current_limit = 1000 
        self.homing_current_limit = 150

        # 상태 플래그
        self.is_homing = False
        self.is_resetting = False
        self.is_torque_on = False
        self.is_teleop_enabled = False

        # --- [2] SDK 초기화 ---
        self.portHandler = PortHandler(self.port_name)
        self.packetHandler = PacketHandler(self.protocol_version)
        
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.addr_present_position, self.len_present_position)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.addr_goal_position, self.len_goal_position)

        # --- [3] 포트 열기 & 초기 설정 ---
        if self.open_port() and self.set_baudrate():
            self.get_logger().info("Port Opened. Setting up...")
            self.disable_all_torque(close_port=False)
            self.set_operating_mode(self.mode_current_based_position)
            self.enable_all_torque()
            
            for dxl_id in self.dxl_ids:
                if not self.groupSyncRead.addParam(dxl_id):
                    self.get_logger().error(f"[ID:{dxl_id}] GroupSyncRead addParam failed")
        else:
            self.get_logger().error("Failed to setup Dynamixel Port.")
            
        # --- [4] ROS 통신 설정 ---
        self.create_subscription(Bool, 'torque_cmd', self.torque_callback, 10)
        self.create_subscription(JointState, 'goal_joint_states', self.goal_joint_state_callback, 10)
        self.create_subscription(String, 'hand_cmd', self.command_callback, 10)
        self.create_subscription(Bool, 'hand_home', self.home_callback, 10)
        self.create_subscription(Bool, 'hand_reset', self.reset_callback, 10)
        self.create_subscription(Bool, 'teleop_enable', self.teleop_enable_callback, 10)
        
        self.status_pub = self.create_publisher(String, 'hand_status', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.timer = self.create_timer(0.05, self.sync_read_callback)
        self.error_timer = self.create_timer(1.0, self.monitor_errors)

    def open_port(self):
        return self.portHandler.openPort()

    def set_baudrate(self):
        return self.portHandler.setBaudRate(self.baudrate)

    def set_operating_mode(self, mode):
        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.addr_operating_mode, mode)

    def split_to_bytes(self, value):
        return [
            DXL_LOBYTE(DXL_LOWORD(value)),
            DXL_HIBYTE(DXL_LOWORD(value)),
            DXL_LOBYTE(DXL_HIWORD(value)),
            DXL_HIBYTE(DXL_HIWORD(value))
        ]

    def send_batch_positions(self, id_pos_dict):
        self.groupSyncWrite.clearParam()
        
        for dxl_id, raw_pos in id_pos_dict.items():
            if raw_pos < 0:
                raw_pos += 4294967296
            
            param_goal_pos = self.split_to_bytes(raw_pos)
            if not self.groupSyncWrite.addParam(dxl_id, param_goal_pos):
                self.get_logger().warn(f"[ID:{dxl_id}] SyncWrite AddParam Failed")
        
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            pass
            
        self.groupSyncWrite.clearParam()

    # [수정됨] 개별 설정을 불러와서 계산하는 로직
    def get_raw_pos_from_abstract(self, dxl_id, abstract_val):
        target_pos = 0
        config = self.joint_limits.get(dxl_id) # 해당 ID의 설정 가져오기

        if config is None:
            return 0
            
        if config['type'] == 'FE':
            # abstract_val: 0.0 (open) ~ 1.0 (close)
            ratio = max(0.0, min(1.0, abstract_val))
            open_pos = config['open']
            close_pos = config['close']
            target_pos = int(open_pos + (close_pos - open_pos) * ratio)
            
        elif config['type'] == 'AA':
            # abstract_val: -1.0 (min) ~ 1.0 (max)
            ratio = max(-1.0, min(1.0, abstract_val))
            ratio_0_1 = (ratio + 1.0) / 2.0
            min_pos = config['min']
            max_pos = config['max']
            target_pos = int(min_pos + (max_pos - min_pos) * ratio_0_1)
            
        return target_pos

    def monitor_errors(self):
        if self.is_homing or self.is_resetting:
            return

        error_details = []
        for dxl_id in self.dxl_ids:
            error_val, result, _ = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, self.addr_hardware_error)
            if result == COMM_SUCCESS and error_val != 0:
                errors = []
                if error_val & 0x01: errors.append("Input Voltage")
                if error_val & 0x04: errors.append("Overheating")
                if error_val & 0x08: errors.append("Encoder")
                if error_val & 0x10: errors.append("Shock")
                if error_val & 0x20: errors.append("Overload")
                error_msg = f"ID{dxl_id}:{'|'.join(errors)}"
                error_details.append(error_msg)
        
        if error_details:
            full_err_msg = f"ERROR! {', '.join(error_details)}"
            self.get_logger().error(full_err_msg)
            self.status_pub.publish(String(data=full_err_msg))

    def teleop_enable_callback(self, msg):
        self.is_teleop_enabled = msg.data
        state_str = "ENABLED" if self.is_teleop_enabled else "DISABLED"
        self.get_logger().info(f"Teleoperation Mode {state_str}")

    def home_callback(self, msg):
        if not self.is_torque_on:
            self.get_logger().warn("Homing Request Ignored: Torque is OFF.")
            return

        if msg.data and not self.is_homing and not self.is_resetting:
            self.get_logger().info("Starting Homing Sequence...")
            t = threading.Thread(target=self.run_homing_sequence, daemon=True)
            t.start()

    def reset_callback(self, msg):
        if msg.data and not self.is_resetting and not self.is_homing:
            self.get_logger().info("Starting Reset Sequence...")
            t = threading.Thread(target=self.run_reset_sequence, daemon=True)
            t.start()

    def run_reset_sequence(self):
        self.is_resetting = True
        try:
            self.get_logger().info("Rebooting all Dynamixels...")
            for dxl_id in self.dxl_ids:
                self.packetHandler.reboot(self.portHandler, dxl_id)
                time.sleep(0.05)
            
            self.get_logger().info("Waiting for reboot...")
            time.sleep(1.0)

            self.get_logger().info("Re-configuring...")
            self.disable_all_torque(close_port=False)
            self.set_operating_mode(self.mode_current_based_position)
            self.enable_all_torque()

            self.get_logger().info("Reset Complete. Homing...")
            self.is_resetting = False 
            self.run_homing_sequence() 

        except Exception as e:
            self.get_logger().error(f"Reset Error: {e}")
            self.is_resetting = False

    def run_homing_sequence(self):
        self.is_homing = True
        try:
            # 1. AA 정렬 (중앙으로)
            self.get_logger().info("Homing Step 1: Aligning AA...")
            for dxl_id in self.aa_ids:
                self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.addr_goal_current, self.default_current_limit)
            
            # AA는 일단 0으로 보냄 (Homing 중에는 0이 중앙이라고 가정하거나, Homing Offset이 이미 적용되어 있다면 0으로 이동)
            # 여기서는 Homing process이므로 절대위치 0보다는, 센서 기준 정렬이 필요할 수 있으나
            # 기존 코드 로직 유지 (단순 0 이동)
            aa_target = {dxl_id: 0 for dxl_id in self.aa_ids}
            self.send_batch_positions(aa_target)
            time.sleep(1.0)

            # 2. FE 펴기 (하드 스톱까지 밀기)
            self.get_logger().info("Homing Step 2: Extending FE...")
            for dxl_id in self.fe_ids:
                self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.addr_goal_current, self.homing_current_limit)
            
            # 펴는 방향으로 충분히 큰 값 전송
            fe_target = {dxl_id: 50000 for dxl_id in self.fe_ids}
            self.send_batch_positions(fe_target)
            time.sleep(3.0)

            # 3. 영점 설정
            self.get_logger().info("Homing Step 3: Zeroing...")
            for dxl_id in self.fe_ids:
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.addr_torque_enable, 0)
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.addr_homing_offset, 0)
                time.sleep(0.05)
                
                dxl_present_pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.addr_present_position)
                if dxl_present_pos > 0x7FFFFFFF: dxl_present_pos -= 4294967296
                
                new_offset = -dxl_present_pos
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.addr_homing_offset, new_offset & 0xFFFFFFFF)
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.addr_torque_enable, 1)
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.addr_goal_position, 0)
                self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, self.addr_goal_current, self.default_current_limit)

            self.get_logger().info("Homing Complete!")

        except Exception as e:
            self.get_logger().error(f"Homing Error: {e}")
        finally:
            self.is_homing = False

    def sync_read_callback(self):
        if self.is_homing or self.is_resetting:
            return

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            pass

        status_msg_list = []
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        for dxl_id in self.dxl_ids:
            if self.groupSyncRead.isAvailable(dxl_id, self.addr_present_position, self.len_present_position):
                present_pos = self.groupSyncRead.getData(dxl_id, self.addr_present_position, self.len_present_position)
                if present_pos > 0x7FFFFFFF:
                    present_pos -= 4294967296
                
                status_msg_list.append(f"{dxl_id}:{present_pos}")
                
                joint_name = self.joint_name_map.get(dxl_id, f"id_{dxl_id}")
                joint_state_msg.name.append(joint_name)
                
                # [수정됨] 개별 설정값을 사용하여 현재 위치(0.0~1.0) 역산
                normalized_pos = 0.0
                config = self.joint_limits.get(dxl_id)
                
                if config:
                    if config['type'] == 'FE':
                        open_pos = config['open']
                        close_pos = config['close']
                        # Div by Zero 방지
                        if close_pos != open_pos:
                            normalized_pos = (present_pos - open_pos) / (close_pos - open_pos)
                            
                    elif config['type'] == 'AA':
                        min_pos = config['min']
                        max_pos = config['max']
                        if max_pos != min_pos:
                            ratio = (present_pos - min_pos) / (max_pos - min_pos)
                            normalized_pos = (ratio * 2.0) - 1.0

                joint_state_msg.position.append(float(normalized_pos))

        if status_msg_list:
            full_str = ", ".join(status_msg_list)
            self.status_pub.publish(String(data=full_str))
            
        if joint_state_msg.name:
            self.joint_state_pub.publish(joint_state_msg)

    def torque_callback(self, msg):
        if msg.data is True:
            self.enable_all_torque()
        else:
            self.disable_all_torque(close_port=False)

    def goal_joint_state_callback(self, msg):
        if not self.is_teleop_enabled:
            return

        if self.is_homing or self.is_resetting:
            return

        target_batch = {}
        for i, name in enumerate(msg.name):
            try:
                target_id = self.name_to_id_map.get(name)
                if target_id is not None:
                    abstract_val = msg.position[i]
                    raw_pos = self.get_raw_pos_from_abstract(target_id, abstract_val)
                    target_batch[target_id] = raw_pos
            except Exception as e:
                pass
        
        if target_batch:
            self.send_batch_positions(target_batch)

    def command_callback(self, msg):
        if self.is_homing or self.is_resetting:
            return
        try:
            data_str = msg.data
            parts = data_str.split(':')
            if len(parts) == 2:
                target_id = int(parts[0])
                val_str = parts[1]
                if target_id in self.dxl_ids:
                    try:
                        abstract_val = float(val_str)
                        raw_pos = self.get_raw_pos_from_abstract(target_id, abstract_val)
                        self.send_batch_positions({target_id: raw_pos})
                    except ValueError:
                        pass
        except ValueError:
            pass

    def enable_all_torque(self):
        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.addr_torque_enable, self.torque_enable)
        self.is_torque_on = True

    def disable_all_torque(self, close_port=True):
        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.addr_torque_enable, self.torque_disable)
        self.is_torque_on = False
        if close_port:
            self.portHandler.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = DxlControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Shutting down DXL Node...')
    finally:
        node.disable_all_torque()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()