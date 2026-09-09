import sys
import os
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from sensor_msgs.msg import JointState  # [추가] JointState 수신용
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QMainWindow,
    QMessageBox,
)
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, Qt
from PyQt5 import uic


class SineCommandDialog(QDialog):
    def __init__(self, joints, fe_ids, parent=None):
        super().__init__(parent)
        self.fe_ids = set(fe_ids)
        self.setWindowTitle("Sine Position Command")

        self.joint_combo = QComboBox()
        for name, dxl_id in joints:
            self.joint_combo.addItem(f"{name} (ID {dxl_id})", dxl_id)

        self.center_spin = QDoubleSpinBox()
        self.center_spin.setDecimals(3)
        self.center_spin.setSingleStep(0.05)

        self.amplitude_spin = QDoubleSpinBox()
        self.amplitude_spin.setDecimals(3)
        self.amplitude_spin.setSingleStep(0.05)
        self.amplitude_spin.setValue(0.25)

        self.frequency_spin = QDoubleSpinBox()
        self.frequency_spin.setRange(0.01, 20.0)
        self.frequency_spin.setDecimals(2)
        self.frequency_spin.setSingleStep(0.1)
        self.frequency_spin.setValue(1.0)
        self.frequency_spin.setSuffix(" Hz")

        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.0, 3600.0)
        self.duration_spin.setDecimals(1)
        self.duration_spin.setSingleStep(1.0)
        self.duration_spin.setValue(5.0)
        self.duration_spin.setSuffix(" s")
        self.duration_spin.setSpecialValueText("Continuous")

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        layout = QFormLayout(self)
        layout.addRow("Joint", self.joint_combo)
        layout.addRow("Center", self.center_spin)
        layout.addRow("Amplitude", self.amplitude_spin)
        layout.addRow("Frequency", self.frequency_spin)
        layout.addRow("Duration", self.duration_spin)
        layout.addRow(buttons)

        self.joint_combo.currentIndexChanged.connect(
            self.update_position_range)
        self.center_spin.valueChanged.connect(
            self.update_amplitude_range)
        self.update_position_range()

    def update_position_range(self, _index=None):
        if self.joint_combo.currentData() in self.fe_ids:
            minimum, maximum = 0.0, 1.0
        else:
            minimum, maximum = -1.0, 1.0

        self.center_spin.setRange(minimum, maximum)
        self.center_spin.setValue((minimum + maximum) / 2.0)
        self.update_amplitude_range()

    def update_amplitude_range(self, _value=None):
        maximum = self.center_spin.maximum()
        minimum = self.center_spin.minimum()
        center = self.center_spin.value()
        max_amplitude = max(
            0.0, min(center - minimum, maximum - center))
        self.amplitude_spin.setRange(0.0, max_amplitude)

    def get_config(self):
        return {
            'dxl_id': int(self.joint_combo.currentData()),
            'center': self.center_spin.value(),
            'amplitude': self.amplitude_spin.value(),
            'frequency': self.frequency_spin.value(),
            'duration': self.duration_spin.value(),
        }

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
        self.pub_cmd = self.node.create_publisher(
            Float32MultiArray, 'hand_cmd', 10)
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

    def publish_command(self, command_values):
        if self.node is not None and self.pub_cmd is not None:
            msg = Float32MultiArray()
            msg.data = [float(value) for value in command_values]
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
        self.command_ids = [31, 32, 33, 34, 35, 36, 37, 38]
        self.command_index = {
            dxl_id: index for index, dxl_id in enumerate(self.command_ids)
        }
        self.command_values = [0.0] * len(self.command_ids)
        self.id_to_slider_name = {
            dxl_id: name for name, dxl_id in self.slider_map.items()
        }
        
        self.fe_ids = [32, 34, 36, 38]
        self.aa_ids = [31, 33, 35, 37]

        self.sine_active = False
        self.sine_config = None

        self.init_ui_connections()

        self.ros_thread = RosWorker()
        self.ros_thread.received_status_signal.connect(self.update_status_ui)
        self.ros_thread.received_joint_state_signal.connect(self.update_sliders_from_feedback) # [추가]
        self.ros_thread.start()

        self.sine_timer = QTimer(self)
        self.sine_timer.setTimerType(Qt.PreciseTimer)
        self.sine_timer.setInterval(10)
        self.sine_timer.timeout.connect(self.update_sine_command)

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

        # 4. 사인파 위치 명령
        if hasattr(self, 'pushButton_2'):
            self.pushButton_2.setText("Sine Command")
            self.pushButton_2.clicked.connect(self.toggle_sine_command)

        # 5. Teleop CheckBox 연결
        if hasattr(self, 'checkBox'):
            self.checkBox.stateChanged.connect(self.toggle_teleop_mode)
        else:
            print("Warning: 'checkBox' not found in UI")

        # 6. 슬라이더 연결
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
            self.stop_sine_command(return_to_center=False)
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
                self.command_values[self.command_index[dxl_id]] = float(pos_float)
                
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

        # 100분율 -> 소수점 변환
        ratio = value / 100.0
        self.command_values[self.command_index[dxl_id]] = ratio
        self.ros_thread.publish_command(self.command_values)

    def toggle_sine_command(self):
        if self.sine_active:
            self.stop_sine_command(return_to_center=True)
            return

        if self.is_teleop_mode:
            QMessageBox.warning(
                self, "Warning",
                "Please disable Teleoperation mode first.")
            return

        joints = list(self.slider_map.items())
        dialog = SineCommandDialog(joints, self.fe_ids, self)
        if dialog.exec_() != QDialog.Accepted:
            return

        self.sine_config = dialog.get_config()
        self.sine_start_time = time.monotonic()
        self.sine_active = True
        self.set_sliders_enabled(False)
        self.pushButton_2.setText("Stop Sine")
        self.sine_timer.start()
        self.update_sine_command()

    def update_sine_command(self):
        if not self.sine_active or self.sine_config is None:
            return

        elapsed = time.monotonic() - self.sine_start_time
        duration = self.sine_config['duration']
        if duration > 0.0 and elapsed >= duration:
            self.stop_sine_command(return_to_center=True)
            return

        dxl_id = self.sine_config['dxl_id']
        center = self.sine_config['center']
        amplitude = self.sine_config['amplitude']
        frequency = self.sine_config['frequency']
        position = center + amplitude * math.sin(
            2.0 * math.pi * frequency * elapsed)

        self.set_sine_position(dxl_id, position, publish=True)

    def set_sine_position(self, dxl_id, position, publish):
        minimum = 0.0 if dxl_id in self.fe_ids else -1.0
        position = max(minimum, min(1.0, position))
        self.command_values[self.command_index[dxl_id]] = position

        slider_name = self.id_to_slider_name[dxl_id]
        slider = getattr(self, slider_name)
        slider.blockSignals(True)
        slider.setValue(int(round(position * 100.0)))
        slider.blockSignals(False)

        if publish:
            self.ros_thread.publish_command(self.command_values)

    def stop_sine_command(self, return_to_center=True):
        if not self.sine_active:
            return

        self.sine_timer.stop()
        if return_to_center and self.sine_config is not None:
            self.set_sine_position(
                self.sine_config['dxl_id'],
                self.sine_config['center'],
                publish=True)

        self.sine_active = False
        self.sine_config = None
        if not self.is_teleop_mode:
            self.set_sliders_enabled(True)
        if hasattr(self, 'pushButton_2'):
            self.pushButton_2.setText("Sine Command")

    def toggle_torque(self):
        self.is_torque_on = not self.is_torque_on
        if not self.is_torque_on:
            self.stop_sine_command(return_to_center=False)
        self.ros_thread.publish_torque(self.is_torque_on)
        self.update_torque_button_ui()

    def go_home(self):
        if self.is_teleop_mode:
            QMessageBox.warning(self, "Warning", "Please disable Teleoperation mode first.")
            return

        self.stop_sine_command(return_to_center=False)
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
            self.stop_sine_command(return_to_center=False)
            self.ros_thread.publish_reset()
            self.reset_sliders()
            QMessageBox.information(self, "Resetting", "Reset sequence started.\nPlease wait a moment...")

    def reset_sliders(self):
        for name, dxl_id in self.slider_map.items():
             if hasattr(self, name):
                slider = getattr(self, name)
                self.command_values[self.command_index[dxl_id]] = 0.0
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
        self.stop_sine_command(return_to_center=False)
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
