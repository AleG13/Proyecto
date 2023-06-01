import time
from Adafruit_IO import Client
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QMainWindow, QApplication
import serial
from interfaz import Ui_MainWindow


from Adafruit_IO import Client


ADAFRUIT_IO_USERNAME = "AlejandroUVG"
ADAFRUIT_IO_KEY = "aio_ZxHg86cFKLA62aYtR7mMwbxFiSls"

aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.serial_port = None
        self.pushButton.clicked.connect(self.connect_port)
        self.pushButton_3.clicked.connect(self.modos)
        self.pushButton_2.clicked.connect(self.close_port)
        
        self.horizontalSlider.valueChanged.connect(self.slider_pwm)
        self.horizontalSlider_2.valueChanged.connect(self.slider_pwm2)
        self.horizontalSlider_3.valueChanged.connect(self.slider_pwm3)
        self.horizontalSlider_4.valueChanged.connect(self.slider_pwm4)

        

        self.timer2 = QTimer(self)
        self.timer2.timeout.connect(self.receive_data_adafruit)
        self.timer2.start(1)

        self.bandera = None
        self.valor0_anterior = None
        self.valor1_anterior = None
        self.valor2_anterior = None
        self.valor3_anterior = None

    def connect_port(self):
        port_name = self.comboBox.currentText()
        try:
            self.serial_port = serial.Serial(port_name, 9600)
        except serial.SerialException as e:
            self.statusbar.showMessage(str(e))

    def send_data(self, data):
        bytes_enviar = int(bin(data), 2).to_bytes((len(bin(data)) + 7) // 8, 'big')
        print(bytes_enviar)
        if self.serial_port is not None:
            self.serial_port.write(bytes_enviar)


    def receive_data_adafruit(self):
        modes = self.comboBox_2.currentText()
        if ( modes == 'ADAFRUIT'):

            valor0_actual = aio.receive('pot1').value
            valor1_actual = aio.receive('pot2').value
            valor2_actual = aio.receive('pot3').value
            valor3_actual = aio.receive('pot4').value
            self.send_data(7)
            self.send_data(7)
            self.send_data(1)
            self.send_data(int(valor0_actual))
            self.send_data(2)
            self.send_data(int(valor1_actual))
            self.send_data(3)
            self.send_data(int(valor2_actual))
            self.send_data(4)
            self.send_data(int(valor3_actual))
            
            
            

    def modos(self):
        modes = self.comboBox_2.currentText()
        if modes == 'MANUAL':
            self.send_data(5)
            self.send_data(5)
            
        if modes == 'EPROM':
            self.send_data(6)
            self.send_data(6)
            
        if modes == 'SERIAL':
            self.send_data(7)
            self.send_data(7)
            
        if modes == 'ADAFRUIT':
            self.send_data(7)
            self.send_data(7) 
            
                
            
    def slider_pwm(self,event):
        self.horizontalSlider.setValue(event)
        self.send_data(1)
        self.send_data(event)    
    def slider_pwm2(self,event):
        self.horizontalSlider_2.setValue(event)
        self.send_data(2)
        self.send_data(event) 
    def slider_pwm3(self,event):
        self.horizontalSlider_3.setValue(event)
        self.send_data(3)
        self.send_data(event)
    def slider_pwm4(self,event):
        self.horizontalSlider_4.setValue(event)
        self.send_data(4)
        self.send_data(event)
    def close_port(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None

if __name__ == "__main__":
    app = QApplication([])
    main_window = MainWindow()
    main_window.show()
    app.exec()

