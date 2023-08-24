import sys
import math
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QLabel, QPushButton
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtCore import QUrl


class LiftCalculator(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
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

        # 在最下面添加超連結
        self.link_label = QLabel('<a href="https://www.facebook.com/groups/ardupilot.taipei">Ardupilot.taipei</a>')
        self.link_label.setTextInteractionFlags(Qt.TextBrowserInteraction)
        self.link_label.setOpenExternalLinks(True)
        layout.addWidget(self.link_label)
        # 圖像展示
        self.image_label = QLabel(self)
        pixmap = QPixmap('./pic.png')  # 你的圖像路徑
        self.image_label.setPixmap(pixmap)

        # 主布局
        main_layout = QHBoxLayout()
        main_layout.addLayout(layout)
        main_layout.addWidget(self.image_label)

        self.setLayout(main_layout)
        self.setWindowTitle('升力估算工具_v1.0 Ardupilot.taipei')
        self.show()

    def calculate_lift(self):
        try:
            angle = float(self.angle_input.text())
            rho = float(self.rho_input.text())
            area = float(self.area_input.text())
            rpm = float(self.rpm_input.text())
            length = float(self.length_input.text())
            blades = float(self.blades_input.text())  # 讀取槳葉數量

            # 計算升力係數，這裡可以用槳葉數量來做調整
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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = LiftCalculator()
    sys.exit(app.exec_())
