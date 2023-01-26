import numpy as np

import sys
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QMainWindow, QPushButton, QProgressBar, QVBoxLayout, QHBoxLayout, QComboBox, QListView, QSizePolicy
from PyQt5.QtGui import QFont, QStandardItemModel, QStandardItem, QIcon
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import serial
import serial.tools.list_ports

class SerialThread(QThread):
    data_received = pyqtSignal(list)
    error_occured = pyqtSignal(str)

    def __init__(self, port, baudrate):
        QThread.__init__(self)
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self.timeout = 0.1
    def run(self):
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate)
            self.serial_connection.timeout = self.timeout
            while not self.isInterruptionRequested():
                if self.serial_connection.read() == b'\xff':
                    data = self.serial_connection.read(15)
                    
                    self.data_received.emit(([x for x in data]))
        except Exception as e:
            self.error_occured.emit(str(e))
        finally:
            self.serial_connection.close()

class MainWindow(QMainWindow):
    def __init__(self, width, height):
        QMainWindow.__init__(self)
        self.width = width
        self.height = height
        self.freqCount = 15

        self.setWindowTitle("STM32 Spectrum Analyser")
        try:
            self.setWindowIcon(QIcon('logo.png'))
        except Exception as e: print(str(e))

        self.resize(width, height)
        
        # Set central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout containing freqBars with labels and menu
        self.mainLayout = QVBoxLayout()
        central_widget.setLayout(self.mainLayout)

        # Create frequency bars with chosen frequency labels
        freqs = [60,120,180,250,400,630,"1k","1.6k","2.5k","4k","6.3k", "8k", "10k", "16k", "20k"]
        self.createFreqBars(freqs)
        self.mainLayout.addLayout(self.freqsLayout)

        # Create menu layout
        self.createMenu()
        self.mainLayout.addLayout(self.menuLayout)

        # Prepare for thread
        self.thread = None

    # Creates frequency bars layout
    def createFreqBars(self, freqs):
        self.freqsLayout = QHBoxLayout(self)
        self.freqsLayout.setContentsMargins(50, 25, 50, 25)
        self.freqsBars = []
        for i, freq in enumerate(freqs):
            temp_bar =  QProgressBar(self)
            temp_bar.setOrientation(Qt.Vertical)
            temp_bar.setValue(int(np.sin(np.pi*i/ (self.freqCount - 1))*100))
            self.freqsBars.append(temp_bar)

            temp_label = QLabel(f"{freq}Hz")
            temp_label.show()
            temp_layout = QVBoxLayout()
            temp_layout.addWidget(temp_bar)
            temp_layout.addWidget(temp_label)

            self.freqsLayout.addLayout(temp_layout)

    # Creates menu layout
    def createMenu(self):
        # Create menu layout, containing a pushButton and a comboBox
        self.menuLayout = QHBoxLayout(self)
        self.menuLayout.setAlignment(Qt.AlignCenter)

        # Create button add at to Horizontal layout
        self.connectButton = QPushButton("Connect", self)
        self.connectButton.setFixedSize(int(0.4*self.width), int(0.2*self.height))
        self.connectButton.clicked.connect(self.connect_disconnect)
        self.connectButton.setFont(QFont('MS Sans Serif', 14))
        self.menuLayout.addWidget(self.connectButton)
        
        # Create a list of ports
        self.ports = [port for port, desc, hwid in sorted(serial.tools.list_ports.comports())]

        # Create a QListView
        list_view = QListView()

        # Add items to the list view
        model = QStandardItemModel()

        for i, port in enumerate(self.ports):
            item = QStandardItem(port)
            model.appendRow(item)
        list_view.setModel(model)

        # Create a QComboBox
        self.portList = QComboBox()
        self.portList.setModel(model)
        self.portList.setView(list_view)
        self.portList.setCurrentIndex(0)
        self.portList.setFixedSize(int(0.4*self.width), int(0.1*self.height))
        self.portList.setFont(QFont('MS Sans Serif', 14))
        self.menuLayout.addWidget(self.portList)

    # Connect / disconnect
    def connect_disconnect(self):
        if self.connectButton.text() == "Connect":
            port = self.portList.currentText()
            self.serial_thread = SerialThread(port, 115200)
            self.serial_thread.data_received.connect(self.handle_data)
            self.serial_thread.error_occured.connect(self.handle_error)
            self.serial_thread.start()
            self.connectButton.setText("Disconnect")
        else:
            self.serial_thread.requestInterruption()
            self.serial_thread.wait()
            self.connect_button.setText("Connect")
            self.exit()
    def handle_data(self, data):
        for freq_value, freq_bar in zip(data, self.freqsBars):
            freq_bar.setValue(int(freq_value*(100/128)))

    def handle_error(self, error_message):
        print(error_message)

app = QApplication(sys.argv)

width, height = 800, 400
window = MainWindow(width, height)
window.show()
sys.exit(app.exec_())
