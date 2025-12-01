import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState  # [추가] JointState 수신용
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5 import uic

# --- [1] ROS 워커 스레드 (통신 전용) ---
class RosWorker(QThread):
    received_status_signal = pyqtSignal(str)
    received_joint_state_signal = pyqtSignal(dict) # [추가] 관절 데이터(Dict) 전달용

    def __init__(self):
        super().__init__()
        self.node = None
        self.pub_cmd = None
        self.pub_torque = None
        self.pub_home = None
        self.pub_reset = None
        self.pub_teleop_enable = None # [추가] 텔레오퍼레이션 활성화 신호용
        self.sub_status = None
        self.sub_joint = None # 초기에는 구독 객체 없음

    def run(self):
        try:
            rclpy.init()
        except:
            if not rclpy.ok():
                return

        self.node = Node('qt_gui_node')
        
        # Subscriber (상태는 항상 수신)
        self.sub_status = self.node.create_subscription(String, 'hand_status', self.status_callback, 10)
        
        # [수정] JointState 구독은 여기서 하지 않고 start_teleop_subscription()에서 함
        
        # Publisher
        self.pub_cmd = self.node.create_publisher(String, 'hand_cmd', 10) 
        self.pub_torque = self.node.create_publisher(Bool, 'torque_cmd', 10)
        self.pub_home = self.node.create_publisher(Bool, 'hand_home', 10)
        self.pub_reset = self.node.create_publisher(Bool, 'hand_reset', 10)
        self.pub_teleop_enable = self.node.create_publisher(Bool, 'teleop_enable', 10) # [추가]
        
        try:
            rclpy.spin(self.node)
        except Exception:
            pass
        finally:
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def status_callback(self, msg):
        self.received_status_signal.emit(msg.data)

    # [추가] JointState 콜백
    def joint_state_callback(self, msg):
        joint_data = {}
        for i, name in enumerate(msg.name):
            try:
                joint_data[name] = msg.position[i]
            except IndexError:
                pass
        self.received_joint_state_signal.emit(joint_data)

    # [신규] 텔레오퍼레이션 구독 시작
    def start_teleop_subscription(self):
        if self.node and self.sub_joint is None:
            self.sub_joint = self.node.create_subscription(
                JointState, 'joint_states', self.joint_state_callback, 10
            )

    # [신규] 텔레오퍼레이션 구독 중지
    def stop_teleop_subscription(self):
        if self.node and self.sub_joint is not None:
            self.node.destroy_subscription(self.sub_joint)
            self.sub_joint = None

    def publish_command(self, dxl_id, val_str):
        if self.node is not None:
            msg = String()
            msg.data = f"{dxl_id}:{val_str}"
            self.pub_cmd.publish(msg)

    def publish_torque(self, state):
        if self.node is not None:
            msg = Bool()
            msg.data = state
            self.pub_torque.publish(msg)

    def publish_home(self):
        if self.node is not None:
            msg = Bool()
            msg.data = True
            self.pub_home.publish(msg)

    def publish_reset(self):
        if self.node is not None:
            msg = Bool()
            msg.data = True
            self.pub_reset.publish(msg)
            
    # [추가] 텔레오퍼레이션 활성화 신호 전송
    def publish_teleop_enable(self, enable):
        if self.node is not None:
            msg = Bool()
            msg.data = enable
            self.pub_teleop_enable.publish(msg)

# --- [2] 메인 윈도우 ---
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        try:
            pkg_path = get_package_share_directory('hand_control')
            ui_path = os.path.join(pkg_path, 'ui', 'hand_gui.ui')
            if not os.path.exists(ui_path):
                raise FileNotFoundError(f"UI file not found at: {ui_path}")
            uic.loadUi(ui_path, self)
        except Exception as e:
            print(f"CRITICAL ERROR: {e}")
            return

        self.is_torque_on = True
        self.is_teleop_mode = False # [추가] 텔레오퍼레이션 모드 플래그
        
        self.slider_map = {
            'finger1_AA': 31, 'finger1_FE': 32,
            'finger2_AA': 33, 'finger2_FE': 34,
            'finger3_AA': 35, 'finger3_FE': 36,
            'finger4_AA': 37, 'finger4_FE': 38
        }
        
        self.fe_ids = [32, 34, 36, 38]
        self.aa_ids = [31, 33, 35, 37]

        self.init_ui_connections()

        self.ros_thread = RosWorker()
        self.ros_thread.received_status_signal.connect(self.update_status_ui)
        self.ros_thread.received_joint_state_signal.connect(self.update_sliders_from_feedback) # [추가]
        self.ros_thread.start()

    def init_ui_connections(self):
        # 1. 토크 버튼
        if hasattr(self, 'btn_torque'):
            self.btn_torque.clicked.connect(self.toggle_torque)
            self.update_torque_button_ui()

        # 2. Go Home 버튼
        if hasattr(self, 'btn_gohome'):
            self.btn_gohome.clicked.connect(self.go_home)

        # 3. RESET 버튼
        if hasattr(self, 'pushButton'):
            self.pushButton.clicked.connect(self.reset_hand)
            self.pushButton.setStyleSheet("background-color: #FF5722; color: white; font-weight: bold;")

        # 4. Teleop CheckBox 연결
        if hasattr(self, 'checkBox'):
            self.checkBox.stateChanged.connect(self.toggle_teleop_mode)
        else:
            print("Warning: 'checkBox' not found in UI")

        # 5. 슬라이더 연결
        for name, dxl_id in self.slider_map.items():
            if hasattr(self, name):
                slider = getattr(self, name)
                
                if dxl_id in self.fe_ids:
                    slider.setRange(0, 100)
                    slider.setValue(0) 
                else:
                    slider.setRange(-100, 100)
                    slider.setValue(0) 

                slider.valueChanged.connect(lambda val, x=dxl_id: self.send_joint_command(x, val))

    def update_status_ui(self, data_str):
        if hasattr(self, 'label_status'):
            self.label_status.setText(f"Status: {data_str}")
            
            if "Error" in data_str or "Err" in data_str or "Overload" in data_str:
                self.label_status.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
            else:
                self.label_status.setStyleSheet("color: black;")

    # [수정] 텔레오퍼레이션 모드 토글 (체크박스) -> 구독 제어 및 DXL 노드에 신호 전송
    def toggle_teleop_mode(self, state):
        if state == Qt.Checked:
            self.is_teleop_mode = True
            self.set_sliders_enabled(False) # 슬라이더 비활성화 (조작 금지)
            self.ros_thread.start_teleop_subscription() # 피드백 구독 시작
            self.ros_thread.publish_teleop_enable(True) # [추가] DXL 노드에 "텔레오퍼레이션 허용" 신호 전송
            
            if hasattr(self, 'label_status'):
                self.label_status.setText("Mode: Teleoperation (Listening to JointStates)")
        else:
            self.is_teleop_mode = False
            self.set_sliders_enabled(True) # 슬라이더 활성화 (UI 조작)
            self.ros_thread.stop_teleop_subscription() # 피드백 구독 중지
            self.ros_thread.publish_teleop_enable(False) # [추가] DXL 노드에 "텔레오퍼레이션 무시" 신호 전송
            
            if hasattr(self, 'label_status'):
                self.label_status.setText("Mode: UI Control")

    # [추가] 슬라이더 활성화/비활성화 헬퍼
    def set_sliders_enabled(self, enabled):
        for name in self.slider_map.keys():
            if hasattr(self, name):
                slider = getattr(self, name)
                slider.setEnabled(enabled)

    # [추가] ROS 피드백을 받아 슬라이더 업데이트 (Teleop 모드일 때만)
    def update_sliders_from_feedback(self, joint_data):
        if not self.is_teleop_mode:
            return

        for name, pos_float in joint_data.items():
            # 이름이 내 슬라이더 목록에 있는지 확인
            if name in self.slider_map:
                dxl_id = self.slider_map[name]
                slider = getattr(self, name)
                
                # 정규화된 값(0.0~1.0 또는 -1.0~1.0)을 슬라이더 값(0~100 또는 -100~100)으로 변환
                slider_val = int(pos_float * 100)
                
                # [중요] 슬라이더 값을 코드로 바꿀 때 valueChanged 이벤트가 발생해서 
                # 다시 명령을 보내는 무한 루프를 막기 위해 시그널 차단
                slider.blockSignals(True)
                slider.setValue(slider_val)
                slider.blockSignals(False)

    def send_joint_command(self, dxl_id, value):
        # 텔레오퍼레이션 모드면 UI 조작 명령을 보내지 않음 (혹시 모를 에러 방지)
        if self.is_teleop_mode:
            return

        val_to_send = ""
        # 100분율 -> 소수점 변환
        ratio = value / 100.0
        val_to_send = f"{ratio:.2f}"
            
        self.ros_thread.publish_command(dxl_id, val_to_send)

    def toggle_torque(self):
        self.is_torque_on = not self.is_torque_on
        self.ros_thread.publish_torque(self.is_torque_on)
        self.update_torque_button_ui()

    def go_home(self):
        if self.is_teleop_mode:
            QMessageBox.warning(self, "Warning", "Please disable Teleoperation mode first.")
            return

        QMessageBox.information(self, "Homing", "Starting Homing Sequence...\nPlease wait about 5 seconds.")
        self.ros_thread.publish_home()
        self.reset_sliders()

    def reset_hand(self):
        if self.is_teleop_mode:
            QMessageBox.warning(self, "Warning", "Please disable Teleoperation mode first.")
            return

        reply = QMessageBox.question(
            self, 'Reboot & Reset',
            "Are you sure you want to reboot all motors and homing?\n(It takes about 5 seconds)",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.ros_thread.publish_reset()
            self.reset_sliders()
            QMessageBox.information(self, "Resetting", "Reset sequence started.\nPlease wait a moment...")

    def reset_sliders(self):
        for name, dxl_id in self.slider_map.items():
             if hasattr(self, name):
                slider = getattr(self, name)
                slider.blockSignals(True)
                if dxl_id in self.fe_ids:
                    slider.setValue(0) 
                else:
                    slider.setValue(0)
                slider.blockSignals(False)

    def update_torque_button_ui(self):
        if not hasattr(self, 'btn_torque'):
            return
        if self.is_torque_on:
            self.btn_torque.setText("Torque ON")
            self.btn_torque.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        else:
            self.btn_torque.setText("Torque OFF")
            self.btn_torque.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")

    def closeEvent(self, event):
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass
            
        self.ros_thread.quit()
        if self.ros_thread.isRunning():
            self.ros_thread.wait(1000)
            if self.ros_thread.isRunning():
                self.ros_thread.terminate()
        
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()