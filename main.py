import sys
import math
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QLabel, QPushButton, QTabWidget, QFrame
from PyQt5.QtWidgets import QVBoxLayout, QComboBox, QLineEdit, QLabel, QPushButton, QWidget,QMessageBox

from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtCore import QUrl
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np

def estimate_fly_time(battery_voltage, battery_capacity, battery_efficiency, motor_count, thrust_to_weight_ratio, power_consumption_per_kg, total_weight):
        # 計算最大起飛重量
    max_takeoff_weight = thrust_to_weight_ratio * total_weight
        
        # 根據您的公式計算平均電流
    avg_current = (max_takeoff_weight * power_consumption_per_kg) / battery_voltage
        
        # 計算電池總能量（mAh轉換成Ah）
    total_battery_energy = battery_voltage * (battery_capacity / 1000) * (battery_efficiency / 100)
        
        # 計算飛行器的總平均功率消耗（W）
    total_motor_power = motor_count * avg_current * battery_voltage
        
        # 估算飛行時間（小時）
    estimated_fly_time_hr = total_battery_energy / total_motor_power
        
        # 轉換成分鐘
    estimated_fly_time_min = estimated_fly_time_hr * 60
        
    return estimated_fly_time_min, max_takeoff_weight

def naca4(series, c, N):
    series_str = str(series)
    if len(series_str) != 4:
        print("Invalid NACA series")
        return

    m = int(series_str[0]) / 100.0
    p = int(series_str[1]) / 10.0
    tt = int(series_str[2:]) / 100.0

    x = np.linspace(0, c, N)
    yt = 5 * tt * c * (0.2969 * np.sqrt(x / c) - 0.1260 * (x / c) - 0.3516 * (x / c)**2 + 0.2843 * (x / c)**3 - 0.1036 * (x / c)**4)

    # 計算凸度
    yc = np.zeros_like(x)
    if m > 0 and p > 0:
        yc_upper = m / p**2 * (2 * p * (x / c) - (x / c)**2)
        yc_lower = m / (1 - p)**2 * ((1 - 2 * p) + 2 * p * (x / c) - (x / c)**2)
        
        for i in range(len(x)):
            if x[i] <= p * c:
                yc[i] = yc_upper[i]
            else:
                yc[i] = yc_lower[i]
    else:
        yc = np.zeros_like(x)

    # 上下表面
    y_upper = yc + yt
    y_lower = yc - yt

    return x, y_upper, y_lower, yc


def naca5(series, c, N):
    series_str = str(series)
    if len(series_str) != 5:
        print("Invalid NACA series")
        return

    m = int(series_str[0]) / 100.0
    p = int(series_str[1:3]) / 100.0
    tt = int(series_str[3:5]) / 100.0
    
    x = np.linspace(0, c, N)
    
    # 厚度分佈
    yt = 5 * tt * c * (0.2969*np.sqrt(x/c) - 0.1260*(x/c) - 0.3516*(x/c)**2 + 0.2843*(x/c)**3 - 0.1015*(x/c)**4)
    
    # 凸度和凸度導數
    if p > 0:
        yc = np.where(x <= p*c, m*x/(p**2) * (2*p - x/c), m*(c - x)/(1-p)**2 * (1 + x/c - 2*p))
        dyc_dx = np.where(x <= p*c, 2*m/(p**2)*(p - x/c), 2*m/(1-p)**2*(p - x/c))
    else:
        yc = np.zeros_like(x)
        dyc_dx = np.zeros_like(x)
    
    theta = np.arctan(dyc_dx)
    
    # 上下翼面
    xu = x - yt * np.sin(theta)
    yu = yc + yt * np.cos(theta)
    xl = x + yt * np.sin(theta)
    yl = yc - yt * np.cos(theta)
    
    return x, yc, xu, yu, xl, yl

class LiftCalculator(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 主佈局和QTabWidget
        main_layout = QVBoxLayout()
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)

        # 第一個頁籤：簡易升力計算
        self.tab1 = QFrame()
        self.tab1_layout = QVBoxLayout()
        self.initLiftTab()
        self.tab1.setLayout(self.tab1_layout)
        self.tab_widget.addTab(self.tab1, '簡易升力計算')

        # 第二個頁籤：NACA4/5翼型
        self.tab2 = QFrame()
        self.tab2_layout = QVBoxLayout()
        self.initNACATab()
        self.tab2.setLayout(self.tab2_layout)
        self.tab_widget.addTab(self.tab2, 'NACA4/5翼型')

        # 第三個頁籤：飛行時長估算
        self.tab3 = QFrame()
        self.tab3_layout = QVBoxLayout()
        self.initFlightTimeTab()
        self.tab3.setLayout(self.tab3_layout)
        self.tab_widget.addTab(self.tab3, '飛行時長估算')

        self.setLayout(main_layout)
        self.setWindowTitle('飛行工具箱_v1.1 Ardupilot.taipei')
        self.show()

    def initLiftTab(self):
        layout = QVBoxLayout()

        # blades_input
        self.blades_label = QLabel("槳葉數量:")
        self.blades_input = QLineEdit(self)
        self.blades_input.setText("2")  # 設置預設值為2片槳葉
        layout.addWidget(self.blades_label)
        layout.addWidget(self.blades_input)

        self.angle_label = QLabel("攻角 (角度):")
        self.angle_input = QLineEdit(self)
        self.angle_input.setText("0.1")  # 設置預設值
        layout.addWidget(self.angle_label)
        layout.addWidget(self.angle_input)

        self.rho_label = QLabel("空氣密度 (ρ):")
        self.rho_input = QLineEdit(self)
        self.rho_input.setText("1.225")  # 設置預設值
        layout.addWidget(self.rho_label)
        layout.addWidget(self.rho_input)

        self.area_label = QLabel("槳面積 (A):")
        self.area_input = QLineEdit(self)
        self.area_input.setText("1")  # 設置預設值
        layout.addWidget(self.area_label)
        layout.addWidget(self.area_input)

        self.rpm_label = QLabel("轉速 (RPM):")
        self.rpm_input = QLineEdit(self)
        self.rpm_input.setText("2600")  # 設置預設值
        layout.addWidget(self.rpm_label)
        layout.addWidget(self.rpm_input)

        self.length_label = QLabel("槳長 m (R):")
        self.length_input = QLineEdit(self)
        self.length_input.setText("0.5")  # 設置預設值
        layout.addWidget(self.length_label)
        layout.addWidget(self.length_input)

        # 計算按鈕
        self.calc_btn = QPushButton("計算升力", self)
        self.calc_btn.clicked.connect(self.calculate_lift)
        layout.addWidget(self.calc_btn)

        # 計算結果
        self.result_label = QLabel("升力 (L):")
        layout.addWidget(self.result_label)
        self.tab1.setLayout(layout)  # 設置tab1的佈局

        # 在最下面添加超連結
        self.link_label = QLabel('<a href="https://www.facebook.com/groups/ardupilot.taipei">Ardupilot.taipei</a>')
        self.link_label.setTextInteractionFlags(Qt.TextBrowserInteraction)
        self.link_label.setOpenExternalLinks(True)
        layout.addWidget(self.link_label)

        # 圖像展示
        self.image_label = QLabel(self)
        pixmap = QPixmap('./pic.png')  # 你的圖像路徑
        self.image_label.setPixmap(pixmap)
        layout.addWidget(self.image_label)  # 加入圖像
        
        # 將 layout 加入到 tab1_layout
        self.tab1_layout.addLayout(layout)
    def calculate_lift(self):
        try:
            angle = float(self.angle_input.text())
            rho = float(self.rho_input.text())
            area = float(self.area_input.text())
            rpm = float(self.rpm_input.text())
            length = float(self.length_input.text())
            blades = float(self.blades_input.text())  # 讀取槳葉數量

            # 計算升力係數，這裡可以用槳葉數量來做調整
            # 0.1 *(0.1*3)
            cl = 0.1 * angle * blades  # 假設升力係數與槳葉數量成正比，這是一個非常簡單的模型！

            # Calculate tip speed
            omega = rpm * 2 * math.pi / 60
            speed = omega * length

            # Calculate Lift in N
            lift = 0.5 * cl * rho * area * math.pow(speed, 2)

            # Convert Lift to Gram-force
            lift_gf = lift / 9.80665

            # Display result
            self.result_label.setText(f"升力 (L): {lift} N<br/>升力（克數）: {lift_gf} gf")

        except ValueError:
            self.result_label.setText("請輸入有效的數字。")

    def initNACATab(self):
        layout = QVBoxLayout()

        # 下拉選單
        self.comboBox = QComboBox(self)
        self.comboBox.addItem("NACA4")
        self.comboBox.addItem("NACA5")
        layout.addWidget(self.comboBox)

        # 數字輸入框
        self.lineEdit = QLineEdit(self)
        layout.addWidget(QLabel("請輸入編號："))
        layout.addWidget(self.lineEdit)

        # 繪圖
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        # 按鈕
        self.button = QPushButton("繪製", self)
        self.button.clicked.connect(self.calculate_wing)
        layout.addWidget(self.button)
        self.tab2.setLayout(layout)

        #self.setLayout(layout)
    def calculate_wing(self):
        series = self.lineEdit.text()
        
        NACA_type = self.comboBox.currentText()

        c = 1.0
        N = 1000

        if NACA_type == "NACA4":
            result = naca4(series, c, N)
            if result is None:
                QMessageBox.warning(self, "錯誤", "無效的 NACA4 系列")
                return
            x, y_upper, y_lower, yc = result

        elif NACA_type == "NACA5":
            result = naca5(series, c, N)
            if result is None:
                QMessageBox.warning(self, "錯誤", "無效的 NACA5 系列")
                return
            x, yc, xu, yu, xl, yl = result
            y_upper = yu
            y_lower = yl

        self.ax.clear()
        self.ax.plot(x, y_upper, label='Upper Surface')
        self.ax.plot(x, y_lower, label='Lower Surface')
        self.ax.plot(x, yc, label='Camber Line', linestyle='--')
        self.ax.plot([0, c], [0, 0], label='Chord Line', linestyle='-.')
        self.ax.axis('equal')
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_title('NACA ' + series)
        self.canvas.draw()




    


    def calculate_flight_time_and_weight(self):
        try:
            battery_voltage = float(self.battery_voltage_edit.text())
            battery_capacity = float(self.battery_capacity_edit.text())
            battery_efficiency = float(self.battery_efficiency_edit.text())
            motor_count = int(self.motor_count_edit.text())
            thrust_to_weight_ratio = float(self.thrust_to_weight_ratio_edit.text())
            power_consumption_per_kg = float(self.power_consumption_per_kg_edit.text())
            total_weight = float(self.total_weight_edit.text())
        except ValueError:
            self.result_label.setText("輸入無效，請確保所有欄位都填寫了有效的數字。")
            return

        estimated_fly_time_min, max_takeoff_weight = estimate_fly_time(
            battery_voltage, 
            battery_capacity, 
            battery_efficiency, 
            motor_count, 
            thrust_to_weight_ratio, 
            power_consumption_per_kg, 
            total_weight
        )

        self.result_label.setText(
            f"預估飛行時間：{estimated_fly_time_min:.2f} 分鐘\n最大起飛重量：{max_takeoff_weight:.2f} 公斤"
        )

    def initFlightTimeTab(self):
        layout = QVBoxLayout()

        # 電池電壓
        self.battery_voltage_edit = QLineEdit(self)
        layout.addWidget(QLabel("請輸入電池電壓（V）："))
        layout.addWidget(self.battery_voltage_edit)

        # 電池容量
        self.battery_capacity_edit = QLineEdit(self)
        layout.addWidget(QLabel("請輸入電池容量（mAh）："))
        layout.addWidget(self.battery_capacity_edit)

        # 電池效率
        self.battery_efficiency_edit = QLineEdit(self)
        layout.addWidget(QLabel("請輸入電池效率（%）："))
        layout.addWidget(self.battery_efficiency_edit)

        # 馬達數量
        self.motor_count_edit = QLineEdit(self)
        layout.addWidget(QLabel("請輸入馬達數量："))
        layout.addWidget(self.motor_count_edit)

        # 推重比
        self.thrust_to_weight_ratio_edit = QLineEdit(self)
        layout.addWidget(QLabel("請輸入推重比："))
        layout.addWidget(self.thrust_to_weight_ratio_edit)

        # 每公斤功耗
        self.power_consumption_per_kg_edit = QLineEdit(self)
        layout.addWidget(QLabel("請輸入每公斤功耗（W/kg）："))
        layout.addWidget(self.power_consumption_per_kg_edit)

        # 總重量
        self.total_weight_edit = QLineEdit(self)
        layout.addWidget(QLabel("請輸入總重量（kg）："))
        layout.addWidget(self.total_weight_edit)

        # 計算按鈕
        self.calc_btn = QPushButton("估算", self)
        self.calc_btn.clicked.connect(self.calculate_flight_time_and_weight)
        layout.addWidget(self.calc_btn)

        # 計算結果標籤
        self.result_label = QLabel("待計算...", self)
        layout.addWidget(self.result_label)

        # 設置佈局
        self.tab3.setLayout(layout)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = LiftCalculator()
    sys.exit(app.exec_())
