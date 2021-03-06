# Proyecto 2 - Programación de microcontroladores
#
# Interfaz gráfica - Python
# Control de 4 servomotores con la computadora
#
# Gerardo Paz - 20173
# Javier Pérez - 20183
#
# Fecha de creación: 22/05/2022

import sys
import serial
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication

# valor de cada servo
s1 = 0 
s2 = 0 
s3 = 0 
s4 = 0

# codificación servos
s1_cod = 0
s2_cod = 1
s3_cod = 2
s4_cod = 3

class App(QMainWindow):
    def __init__(self):
        super().__init__() # Iniciar características del QMainWindow
        uic.loadUi("main_GUI.ui", self) # Agregar interfaz gráfica
        
        # configuración de terminal
        self.ser = serial.Serial(port="COM4", baudrate=9600, timeout=1) # terminal
        self.ser.close() # cerrar a menos que la utilice
                
        self.servo1.sliderReleased.connect(self.get_servo1) # S1 VAL
        
        self.servo2.sliderReleased.connect(self.get_servo2) # S2 VAL
        
        self.servo3.sliderReleased.connect(self.get_servo3) # S3 VAL
        
        self.servo4.sliderReleased.connect(self.get_servo4) # S4 VAL
        
    # ------------------------ SERVO 1 ------------------------------   
    def get_servo1(self): # SERVO 1
        x1 = self.servo1.value()
        s1 = int(x1*31/99)
        
        self.ser.open() # Abrimos puerto serial
        self.ser.write(chr(s1).encode()) # Enviamos valor de la perilla como caracter
        #a = self.ser.readline().decode() # Leemos respuesta del PIC
        self.ser.close() # Cerramos puerto serial
        
        print("Servo 1: " + str(s1))
        #print(str(ord(a))) # 5 posiciones del PIC + caracter en decimal (ver tabla ASCII)
    
        self.lbl_s1.setText(str(x1)) # leer valor de slider
        
    # ------------------------ SERVO 2 ------------------------------         
    def get_servo2(self): #SERVO 2
        x2 = self.servo2.value()
        s2 = int((x2*31/99)+32)
        
        self.ser.open() # Abrimos puerto serial
        self.ser.write(chr(s2).encode()) # Enviamos valor de la perilla como caracter
        #a = self.ser.readline().decode() # Leemos respuesta del PIC
        self.ser.close() # Cerramos puerto serial
        
        #print(str(ord(a)))
        print("Servo 2: " + str(s2))
        self.lbl_s2.setText(str(x2)) # leer valor de slider
        
    # ------------------------ SERVO 3 ------------------------------  
    def get_servo3(self): # SERVO 3
        x3 = self.servo3.value()
        s3 = int((x3*31/99)+64)
        
        self.ser.open() # Abrimos puerto serial
        self.ser.write(chr(s3).encode()) # Enviamos valor de la perilla como caracter
        #a = self.ser.readline().decode() # Leemos respuesta del PIC
        self.ser.close() # Cerramos puerto serial
        
        #print(str(ord(a)))
        print("Servo 3: " + str(s3))
        self.lbl_s3.setText(str(x3)) # leer valor de slider
        
    # ------------------------ SERVO 4 ------------------------------
    def get_servo4(self): # SERVO 4
        x4 = self.servo4.value()
        s4 = int((x4*31/99)+96)
        
        self.ser.open() # Abrimos puerto serial
        self.ser.write(chr(s4).encode()) # Enviamos valor de la perilla como caracter
        #a = self.ser.readline().decode() # Leemos respuesta del PIC
        self.ser.close() # Cerramos puerto serial

        #print(str(ord(a)))
        print("Servo 4: " + str(s4))
        self.lbl_s4.setText(str(x4)) # leer valor de slider
        
        
        
if __name__ == '__main__':
    # inicializar interfaz gráfica
    app = QApplication(sys.argv)
    GUI = App()
    GUI.show()
    sys.exit(app.exec_())