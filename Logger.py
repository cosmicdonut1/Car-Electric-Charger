# ###########################
# Pacemaker Technologies GmbH
# Herrgottwiesgasse 121
# A-8020 Graz  | Austria
# ###########################
# Creator: Kseniia Soboleva
# E-Mail:k.soboleva@pacemaker-technologies.at
#
# Information: Installs necessary libraries
#

import os
import sys
import math
import datetime
import time
import serial
import threading
import serial
import serial.tools.list_ports
from PyQt6.QtCore import Qt, QIODevice, QThread, pyqtSignal, QTimer, QMutex
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QLineEdit, QLabel, QPushButton, QCheckBox, QComboBox, QFormLayout, QMessageBox

class AverageFilter:
    def __init__(self):
        self.SIZE = 20
        self.buf = [0] * self.SIZE
        self.sum = 0
        self.count = 0

    def reset(self):
        self.buf = [0] * self.SIZE
        self.sum = 0
        self.count = 0

    def filtered(self, val):
        self.count += 1
        if self.count >= self.SIZE:
            self.count = 0
        self.sum -= self.buf[self.count]
        self.sum += val
        self.buf[self.count] = val
        return self.sum / self.SIZE

class MedianFilter:
    def __init__(self):
        self.SIZE = 20
        self.buffer = [0.0] * self.SIZE
        self._count = 0
    
    def filtered(self, newVal):
        self.buffer[self._count] = newVal
        if (self._count < self.SIZE - 1) and (self.buffer[self._count] > self.buffer[self._count + 1]):
            for i in range(self._count, self.SIZE - 1):
                if self.buffer[i] > self.buffer[i + 1]:
                    buff = self.buffer[i]
                    self.buffer[i] = self.buffer[i + 1]
                    self.buffer[i + 1] = buff
        else:
            if (self._count > 0) and (self.buffer[self._count - 1] > self.buffer[self._count]):
                for i in range(self._count, 0, -1):
                    if self.buffer[i] < self.buffer[i - 1]:
                        buff = self.buffer[i]
                        self.buffer[i] = self.buffer[i - 1]
                        self.buffer[i - 1] = buff
        self._count += 1
        if self._count >= self.SIZE:
            self._count = 0
        return self.buffer[self.SIZE // 2]
    
    def reset(self):
        for i in range(self._count, self.SIZE - 1):
            self.buffer[i] = 0.0
        self._count = 0
  
class EnergyMeter:
    def __init__(self):
        self.old_time = datetime.datetime.now().strftime("Date: %Y-%m-%d Time: %H:%M:%S.%f")[:-3]
        self.watt_hour = 0
        self.old_power = 0

    def compute(self, value):
        # Calculate powers Wh
        timestamp_ = datetime.datetime.now().strftime("Date: %Y-%m-%d Time: %H:%M:%S.%f")[:-3]
        curr_time = datetime.datetime.strptime(timestamp_, "Date: %Y-%m-%d Time: %H:%M:%S.%f")
        old_time = datetime.datetime.strptime(self.old_time, "Date: %Y-%m-%d Time: %H:%M:%S.%f")
        self.old_time = timestamp_
                        
        delta_t = curr_time - old_time
        seconds = delta_t.total_seconds()
        # Info calc formula
        # watt_sec += deltaTau*(new_val_power+old_val_power)/2;
        # watt_hour = watt_sec/3600
        # old_val_power=new_val_power;
        self.watt_hour += ((seconds * (value + self.old_power))/2)/3600
        self.old_power = value
        return self.watt_hour
    
    def reset(self):
        self.old_time = datetime.datetime.now().strftime("Date: %Y-%m-%d Time: %H:%M:%S.%f")[:-3]
        self.watt_hour = 0
        self.old_power = 0
        
class SerialThread(QThread):
    ReceiveCallback = pyqtSignal(bytes)
    SerialErrorCallback = pyqtSignal(bool)
    MsgboxCallback = pyqtSignal(str)

    def __init__(self, parent=None):
        super(SerialThread, self).__init__(parent)
        self.serial = None
        self.currSerialName = ""
        self.currSerialBaudRate = 0
        self.currSerialParity = serial.PARITY_NONE
        self.currSerialStopBits = serial.STOPBITS_ONE
        self.currSerialByteSize = serial.FIVEBITS
        self.currSerialXonXoff = False
        self.currSerialRtsCts = False
        self.currSerialDsrDtr = False
        self.abort = False
        self.error = False
        self.line = ''
        self.count = 0
        self.requestReconnect = False

    def connect(self, serName, serRate, serParity, serStopbits, serByteSize, serFlowControl):
        if sys.platform.startswith("linux"):
            serialName = "/dev/" + serName
        elif os.name == "nt":
            serialName = serName

        serialBaudRate = serRate
        serialParity = serial.PARITY_NONE
        serialStopBits = serial.STOPBITS_ONE
        serialByteSize = serial.FIVEBITS
        serialXonXoff = False
        serialRtsCts = False
        serialDsrDtr = False

        if serParity == 0:
            serialParity = serial.PARITY_NONE
        elif serParity == 1:
            serialParity = serial.PARITY_EVEN
        elif serParity == 2:
            serialParity = serial.PARITY_ODD
        elif serParity == 3:
            serialParity = serial.PARITY_MARK
        elif serParity == 4:
            serialParity = serial.PARITY_SPACE

        if serStopbits == 0:
            serialStopBits = serial.STOPBITS_ONE
        elif serStopbits == 1:
            serialStopBits = serial.STOPBITS_ONE_POINT_FIVE
        elif serStopbits == 2:
            serialStopBits = serial.STOPBITS_TWO

        if serByteSize == 5:
            serialByteSize = serial.FIVEBITS
        elif serByteSize == 6:
            serialByteSize = serial.SIXBITS
        elif serByteSize == 7:
            serialByteSize = serial.SEVENBITS
        elif serByteSize == 8:
            serialByteSize = serial.EIGHTBITS

        if serFlowControl == 1:
            serialXonXoff = False
            serialRtsCts = True
            serialDsrDtr = False
        elif serFlowControl == 2:
            serialXonXoff = False
            serialRtsCts = False
            serialDsrDtr = True
        elif serFlowControl == 3:
            serialXonXoff = True
            serialRtsCts = False
            serialDsrDtr = False

        self.currSerialName = serialName
        self.currSerialBaudRate = serialBaudRate
        self.currSerialParity = serialParity
        self.currSerialStopBits = serialStopBits
        self.currSerialByteSize = serialByteSize
        self.currSerialXonXoff = serialXonXoff
        self.currSerialRtsCts = serialRtsCts
        self.currSerialDsrDtr = serialDsrDtr
        
        try:
            self.serial = serial.Serial(port=serialName, 
                                        baudrate=serialBaudRate, 
                                        parity=serialParity,
                                        stopbits=serialStopBits,
                                        bytesize=serialByteSize, 
                                        xonxoff=serialXonXoff, 
                                        rtscts=serialRtsCts,
                                        dsrdtr=serialDsrDtr,
                                        timeout=None
                                        )
            
            self.serial.flushInput()
            self.serial.flushOutput()
                    
            if self.serial.isOpen():
                time.sleep(0.1)
                
        except Exception as e:
            if not self.error:
                self.MsgboxCallback.emit("Error opening COM port: " + str(e))
                self.SerialErrorCallback.emit(True)
                self.error = True

    def send(self, text):
        try:
            if not self.requestReconnect:
                if self.serial is not None:
                    if self.serial.isOpen():
                        self.serial.write(text.encode())
        except Exception as e:
            if not self.error:
                self.MsgboxCallback.emit("Send error COM port: " + str(e))
                self.SerialErrorCallback.emit(True)
                self.error = True

    def reconnect(self):
        self.requestReconnect = True
            
    def stop(self):
        self.count = 0
        self.abort = True
        self.wait()
        try:
            if self.serial is not None:
                if self.serial.isOpen():
                    self.serial.close()
                self.serial.__del__()
        except Exception as e:
            self.MsgboxCallback.emit("Close error closing COM port: " + str(e))

    def run(self):
        while True:
            if self.abort:
                return
            try:
                if self.serial is not None:
                    if self.serial.isOpen():
                        rx = self.serial.read(self.serial.inWaiting())
                        if rx == b'':
                            self.count += 1
                            # Timeout ~5-6 sec
                            if self.count >= 3500000:
                                if not self.error:
                                    self.MsgboxCallback.emit("Timeout exception COM port")
                                    self.SerialErrorCallback.emit(True)
                                    self.error = True
                        for char in rx:
                            self.count = 0
                            self.line += (chr(char))
                            if chr(char) == '\r':
                                self.ReceiveCallback.emit(self.line.encode())
                                self.line = ''
                                
                if self.requestReconnect:
                    if self.serial is not None:
                        if self.serial.isOpen():
                            self.serial.close()
                        self.serial.__del__()
                        self.serial = None

                    self.serial = serial.Serial(port=self.currSerialName, 
                                                baudrate=self.currSerialBaudRate, 
                                                parity=self.currSerialParity,
                                                stopbits=self.currSerialStopBits,
                                                bytesize=self.currSerialByteSize, 
                                                xonxoff=self.currSerialXonXoff, 
                                                rtscts=self.currSerialRtsCts,
                                                dsrdtr=self.currSerialDsrDtr,
                                                timeout=None
                                                )

                    self.serial.flushInput()
                    self.serial.flushOutput()
                    
                    if self.serial.isOpen():
                        time.sleep(0.1)

                    self.requestReconnect = False
                    self.count = 0
                        
            except Exception as e:
                if not self.error:
                    self.MsgboxCallback.emit("Error serial port: " + str(e))
                    self.SerialErrorCallback.emit(True)
                    self.error = True
      
        self.deleteLater()

class NeChargeMonitor(QMainWindow):
    def __init__(self):
        super(NeChargeMonitor, self).__init__()

        self.serialThread = None
        self.filename = ""
        # Constants for the new formula
        self.VOLTAGE = 3.3  # 3.3 volts
        self.SENSITIVITY = 15e-3  # 15 mV/A
        #
        self.voltage_N = 0.0
        self.voltage_0 = 0.0
        self.voltage_1 = 0.0
        self.voltage_2 = 0.0
        
        self.current_0 = 0.0
        self.current_1 = 0.0
        self.current_2 = 0.0
        
        self.power_0 = 0.0
        self.power_1 = 0.0
        self.power_2 = 0.0
        self.powers_sum = 0.0
        
        self.watt_hour_0 = 0.0
        self.watt_hour_1 = 0.0
        self.watt_hour_2 = 0.0
        
        self.voltage_average_filter_0 = AverageFilter()
        self.voltage_average_filter_1 = AverageFilter()
        self.voltage_average_filter_2 = AverageFilter()
        
        self.voltage_median_filter_0 = MedianFilter()
        self.voltage_median_filter_1 = MedianFilter()
        self.voltage_median_filter_2 = MedianFilter()

        self.current_average_filter_0 = AverageFilter()
        self.current_average_filter_1 = AverageFilter()
        self.current_average_filter_2 = AverageFilter()
        
        self.current_median_filter_0 = MedianFilter()
        self.current_median_filter_1 = MedianFilter()
        self.current_median_filter_2 = MedianFilter()

        self.energy_meter_0 = EnergyMeter()
        self.energy_meter_1 = EnergyMeter()
        self.energy_meter_2 = EnergyMeter()
        
        # Set window properties
        self.setWindowTitle("NeCharge")
        self.setFixedSize(900, 300)

        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Create main layout
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)

        # Create left, center, and right form layouts
        left_layout = QFormLayout()
        center_layout = QFormLayout()
        powers_w_layout = QFormLayout()
        right_layout = QFormLayout()

        # Add left, center, and right layouts to main layout
        main_layout.addLayout(left_layout)
        main_layout.addLayout(center_layout)
        main_layout.addLayout(powers_w_layout)
        main_layout.addLayout(right_layout)

        # Add 4 text boxes to left layout
        # Create the 4 Voltage label and text box pairs
        label1 = QLabel("N (V):")
        self.parameterInfo_1 = QLineEdit()
        self.parameterInfo_1.setFixedWidth(150)
        left_layout.addRow(label1, self.parameterInfo_1)

        label2 = QLabel("U1 (V):")
        self.parameterInfo_2 = QLineEdit()
        self.parameterInfo_2.setFixedWidth(150)
        left_layout.addRow(label2, self.parameterInfo_2)

        label3 = QLabel("U2 (V):")
        self.parameterInfo_3 = QLineEdit()
        self.parameterInfo_3.setFixedWidth(150)
        left_layout.addRow(label3, self.parameterInfo_3)

        label4 = QLabel("U3 (V):")
        self.parameterInfo_4 = QLineEdit()
        self.parameterInfo_4.setFixedWidth(150)
        left_layout.addRow(label4, self.parameterInfo_4)

        # Create the 3 Current label and text box pairs
        label5 = QLabel("I1 (A):")
        self.parameterInfo_5 = QLineEdit()
        self.parameterInfo_5.setFixedWidth(150)
        center_layout.addRow(label5, self.parameterInfo_5)

        label6 = QLabel("I2 (A):")
        self.parameterInfo_6 = QLineEdit()
        self.parameterInfo_6.setFixedWidth(150)
        center_layout.addRow(label6, self.parameterInfo_6)

        label7 = QLabel("I3 (A):")
        self.parameterInfo_7 = QLineEdit()
        self.parameterInfo_7.setFixedWidth(150)
        center_layout.addRow(label7, self.parameterInfo_7)

        # Create the 3 Power label and text box pairs
        label8 = QLabel("P1 (W):")
        self.parameterInfo_8 = QLineEdit()
        self.parameterInfo_8.setFixedWidth(150)
        powers_w_layout.addRow(label8, self.parameterInfo_8)

        label9 = QLabel("P2 (W):")
        self.parameterInfo_9 = QLineEdit()
        self.parameterInfo_9.setFixedWidth(150)
        powers_w_layout.addRow(label9, self.parameterInfo_9)

        label10 = QLabel("P3 (W):")
        self.parameterInfo_10 = QLineEdit()
        self.parameterInfo_10.setFixedWidth(150)
        powers_w_layout.addRow(label10, self.parameterInfo_10)

        # Create the 3 Power label and text box pairs
        label11 = QLabel("P1 (Wh):")
        self.parameterInfo_11 = QLineEdit()
        self.parameterInfo_11.setFixedWidth(150)
        powers_w_layout.addRow(label11, self.parameterInfo_11)

        label12 = QLabel("P2 (Wh):")
        self.parameterInfo_12 = QLineEdit()
        self.parameterInfo_12.setFixedWidth(150)
        powers_w_layout.addRow(label12, self.parameterInfo_12)

        label13 = QLabel("P3 (Wh):")
        self.parameterInfo_13 = QLineEdit()
        self.parameterInfo_13.setFixedWidth(150)
        powers_w_layout.addRow(label13, self.parameterInfo_13)

        label14 = QLabel("Sum P:")
        self.parameterInfo_14 = QLineEdit()
        self.parameterInfo_14.setFixedWidth(150)
        powers_w_layout.addRow(label14, self.parameterInfo_14)

        label21 = QLabel("Aver.Filt. I1 (A):")
        self.parameterInfo_21 = QLineEdit()
        self.parameterInfo_21.setFixedWidth(150)
        center_layout.addRow(label21, self.parameterInfo_21)

        label22 = QLabel("Aver.Filt. I2 (A):")
        self.parameterInfo_22 = QLineEdit()
        self.parameterInfo_22.setFixedWidth(150)
        center_layout.addRow(label22, self.parameterInfo_22)

        label23 = QLabel("Aver.Filt. I3 (A):")
        self.parameterInfo_23 = QLineEdit()
        self.parameterInfo_23.setFixedWidth(150)
        center_layout.addRow(label23, self.parameterInfo_23)

        label24 = QLabel("Med.Filt. I1 (A):")
        self.parameterInfo_24 = QLineEdit()
        self.parameterInfo_24.setFixedWidth(150)
        center_layout.addRow(label24, self.parameterInfo_24)

        label25 = QLabel("Med.Filt. I2 (A):")
        self.parameterInfo_25 = QLineEdit()
        self.parameterInfo_25.setFixedWidth(150)
        center_layout.addRow(label25, self.parameterInfo_25)

        label26 = QLabel("Med.Filt. I3 (A):")
        self.parameterInfo_26 = QLineEdit()
        self.parameterInfo_26.setFixedWidth(150)
        center_layout.addRow(label26, self.parameterInfo_26)

        label27 = QLabel("Aver.Filt. U1 (V):")
        self.parameterInfo_27 = QLineEdit()
        self.parameterInfo_27.setFixedWidth(150)
        left_layout.addRow(label27, self.parameterInfo_27)

        label28 = QLabel("Aver.Filt. U2 (V):")
        self.parameterInfo_28 = QLineEdit()
        self.parameterInfo_28.setFixedWidth(150)
        left_layout.addRow(label28, self.parameterInfo_28)
        
        label29 = QLabel("Aver.Filt. U3 (V):")
        self.parameterInfo_29 = QLineEdit()
        self.parameterInfo_29.setFixedWidth(150)
        left_layout.addRow(label29, self.parameterInfo_29)
        
        label30 = QLabel("Med.Filt. U1 (V):")
        self.parameterInfo_30 = QLineEdit()
        self.parameterInfo_30.setFixedWidth(150)
        left_layout.addRow(label30, self.parameterInfo_30)

        label31 = QLabel("Med.Filt. U2 (V):")
        self.parameterInfo_31 = QLineEdit()
        self.parameterInfo_31.setFixedWidth(150)
        left_layout.addRow(label31, self.parameterInfo_31)
        
        label32 = QLabel("Med.Filt. U3 (V):")
        self.parameterInfo_32 = QLineEdit()
        self.parameterInfo_32.setFixedWidth(150)
        left_layout.addRow(label32, self.parameterInfo_32)
        
        self.textLabelsEnable(False)

        # Add elements to right layout for choosing com port and config
        self.refreshPortsButton = QPushButton(f"Refresh List Ports")
        right_layout.addRow("", self.refreshPortsButton)
        self.refreshPortsButton.clicked.connect(self.refreshPorts)

        label15 = QLabel("Port name")
        com_ports = serial.tools.list_ports.comports()
        com_port_names = [port.device for port in com_ports]
        self.interfaceListPorts = QComboBox()
        self.interfaceListPorts.addItems(com_port_names)
        right_layout.addRow(label15, self.interfaceListPorts)

        label16 = QLabel("Baud Rates")
        self.interfaceBaudRates = QComboBox(self)
        self.interfaceBaudRates.addItems([
            '50', '75', '110', '134', '150',
            '200', '300', '600', '1200', '1800',
            '2400', '4800', '9600', '14400', '19200',
            '28800', '31250', '38400', '51200', '56000',
            '57600', '76800', '115200', '128000', '230400',
            '250000', '256000', '460800', '500000', '576000',
            '921600', '1000000', '1152000', '1500000', '2000000',
            '2500000', '3000000', '3500000', '4000000'
        ])
        self.interfaceBaudRates.setCurrentText('115200')
        self.interfaceBaudRates.setMinimumHeight(15)
        right_layout.addRow(label16, self.interfaceBaudRates)

        label17 = QLabel("Data Bits")
        self.interfaceDataBits = QComboBox(self)
        self.interfaceDataBits.addItems(['5 bit', '6 bit', '7 bit', '8 bit'])
        self.interfaceDataBits.setCurrentIndex(3)
        self.interfaceDataBits.setMinimumHeight(15)
        right_layout.addRow(label17, self.interfaceDataBits)

        label18 = QLabel("Parity")
        self.interfaceParity = QComboBox(self)
        self.interfaceParity.addItems(['No Parity', 
                                       'Even Parity', 
                                       'Odd Parity', 
                                       'Mark Parity', 
                                       'Space Parity'])
        self.interfaceParity.setCurrentIndex(0)
        self.interfaceParity.setMinimumHeight(15)
        right_layout.addRow(label18, self.interfaceParity)

        label19 = QLabel("Stop Bits")
        self.interfaceStopBits = QComboBox(self)
        self.interfaceStopBits.addItems(['One Stop', 
                                         'One And Half Stop', 
                                         'Two Stop'])
        self.interfaceStopBits.setCurrentIndex(0)
        self.interfaceStopBits.setMinimumHeight(15)
        right_layout.addRow(label19, self.interfaceStopBits)

        label20 = QLabel("Flow Control")
        self.interfaceFlowControl = QComboBox(self)
        self.interfaceFlowControl.addItems(['No Flow Control', 
                                            'Hardware Control (RTS/CTS)', 
                                            'Hardware Control (DSR/DTR)', 
                                            'Software Control'])
        self.interfaceFlowControl.setCurrentIndex(0)
        self.interfaceFlowControl.setMinimumHeight(15)
        right_layout.addRow(label20, self.interfaceFlowControl)

        # Add 2 buttons to right layout
        self.startButton = QPushButton(f"Start ")
        right_layout.addRow("", self.startButton)
        self.startButton.clicked.connect(self.startSerialCommunication)

        self.stopButton = QPushButton(f"Stop ")
        right_layout.addRow("", self.stopButton)
        self.stopButton.clicked.connect(self.stopSerialCommunication)

        # Add checkbox to right layout
        self.enableLog = QCheckBox("Enable log")
        right_layout.addRow("", self.enableLog)

        # Find the serial port to use
        self.previousSerialPort = self.interfaceListPorts.currentText()
        self.updateSerialPortsList()
        self.previousSerialPort = self.interfaceListPorts.currentText()

        self.timerRequest = QTimer()
        self.timerRequest.timeout.connect(self.timerRequestEvent)
        self.timerRequest.stop()
        
        self.timerReconnection = QTimer()
        self.timerReconnection.timeout.connect(self.timerReconnectionEvent)
        self.timerReconnection.stop()

    def refreshPorts(self):
        self.previousSerialPort = self.interfaceListPorts.currentText()
        self.updateSerialPortsList()
        self.previousSerialPort = self.interfaceListPorts.currentText()

    def startSerialCommunication(self):

        if hasattr(self, 'start_clicked_before'):
            self.msgbox("Start button was already pushed. Push Stop button to start new measure")
            return

        if self.isActiveLog():
            # Get the current time
            now = datetime.datetime.now()
            # Format the time as a string
            timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
            # Use the timestamp in the filename
            self.filename = f"log_{timestamp}.txt"
        else:
            self.filename = ""

        self.serialStartThread()

        if self.serialThread.isRunning():
            self.start_clicked_before = True
        else:
            self.start_clicked_before = False
            self.serialStopThread()
            return
        
        # reset all data
        self.voltage_N = 0.0
        self.voltage_0 = 0.0
        self.voltage_1 = 0.0
        self.voltage_2 = 0.0
        
        self.current_0 = 0.0
        self.current_1 = 0.0
        self.current_2 = 0.0
        
        self.power_0 = 0.0
        self.power_1 = 0.0
        self.power_2 = 0.0
        self.powers_sum = 0.0
        
        self.watt_hour_0 = 0.0
        self.watt_hour_1 = 0.0
        self.watt_hour_2 = 0.0
        
        self.voltage_average_filter_0.reset()
        self.voltage_average_filter_1.reset()
        self.voltage_average_filter_2.reset()
        
        self.voltage_median_filter_0.reset()
        self.voltage_median_filter_1.reset()
        self.voltage_median_filter_2.reset()
        
        self.current_average_filter_0.reset()
        self.current_average_filter_1.reset()
        self.current_average_filter_2.reset()
        
        self.current_median_filter_0.reset()
        self.current_median_filter_1.reset()
        self.current_median_filter_2.reset()

        self.energy_meter_0.reset()
        self.energy_meter_1.reset()
        self.energy_meter_2.reset()
        
        self.timerRequest.start(250)
        self.timerReconnection.start(1000 * 60 * 10)

        # Disable all serial controls
        self.serialControlEnable(False)
        # Disable log control
        self.logControlEnable(False)

    def stopSerialCommunication(self):
        if hasattr(self, 'start_clicked_before'):
            del self.start_clicked_before

        self.timerRequest.stop()
        self.timerReconnection.stop()

        self.serialStopThread()

        self.filename = ""

        self.textLabelsClear()
        # Enable all serial controls
        self.serialControlEnable(True)
        # Enable log control
        self.logControlEnable(True)

    def portName(self):
        return self.interfaceListPorts.currentText()

    def baudRate(self):
        return int(self.interfaceBaudRates.currentText())

    def dataBit(self):
        return int(self.interfaceDataBits.currentIndex() + 5)

    def parity(self):
        return self.interfaceParity.currentIndex()

    def stopBit(self):
        return self.interfaceStopBits.currentIndex()

    def flowControl(self):
        return self.interfaceFlowControl.currentIndex()

    def updateSerialPortsList(self):
        # Get the available serial ports
        com_ports = QSerialPortInfo.availablePorts()
        com_port_names = [port.portName() for port in com_ports]

        # Update the QComboBox with the available serial ports
        self.interfaceListPorts.clear()
        self.interfaceListPorts.addItems(com_port_names)

        # If the previous serial port is still available, select it
        if self.previousSerialPort in com_port_names:
            index = com_port_names.index(self.previousSerialPort)
            self.interfaceListPorts.setCurrentIndex(index)

    def serialControlEnable(self, flag):
        self.interfaceListPorts.setEnabled(flag)
        self.interfaceBaudRates.setEnabled(flag)
        self.interfaceDataBits.setEnabled(flag)
        self.interfaceParity.setEnabled(flag)
        self.interfaceStopBits.setEnabled(flag)
        self.interfaceFlowControl.setEnabled(flag)
        self.refreshPortsButton.setEnabled(flag)

    def serialStartThread(self):
        serialPortName = self.portName()
        serialBaudRate = self.baudRate()
        serialParity = self.parity()
        serialStopBits = self.stopBit()
        serialByteSize = self.dataBit()
        serialFlowControl = self.flowControl()

        if self.serialThread is not None:
            self.serialThread.stop()
            self.serialThread = None

        self.serialThread = SerialThread()
        self.serialThread.ReceiveCallback.connect(self.update, Qt.ConnectionType.QueuedConnection)
        self.serialThread.SerialErrorCallback.connect(self.error, Qt.ConnectionType.QueuedConnection)
        self.serialThread.MsgboxCallback.connect(self.msgbox, Qt.ConnectionType.QueuedConnection)
        self.serialThread.connect(serialPortName, serialBaudRate, serialParity, serialStopBits, serialByteSize, serialFlowControl)

        if not self.serialThread.isRunning():
            self.serialThread.start()

    def serialStopThread(self):
        if self.serialThread is not None:
            self.serialThread.stop()
            self.serialThread = None

    def logControlEnable(self, flag):
        self.enableLog.setEnabled(flag)

    def isActiveLog(self):
        return self.enableLog.isChecked()

    def logFileName(self):
        return self.filename

    def textLabelsEnable(self, flag):
        self.parameterInfo_1.setEnabled(flag)
        self.parameterInfo_2.setEnabled(flag)
        self.parameterInfo_3.setEnabled(flag)
        self.parameterInfo_4.setEnabled(flag)
        self.parameterInfo_5.setEnabled(flag)
        self.parameterInfo_6.setEnabled(flag)
        self.parameterInfo_7.setEnabled(flag)
        self.parameterInfo_8.setEnabled(flag)
        self.parameterInfo_9.setEnabled(flag)
        self.parameterInfo_10.setEnabled(flag)
        self.parameterInfo_11.setEnabled(flag)
        self.parameterInfo_12.setEnabled(flag)
        self.parameterInfo_13.setEnabled(flag)
        self.parameterInfo_14.setEnabled(flag)
        self.parameterInfo_21.setEnabled(flag)
        self.parameterInfo_22.setEnabled(flag)
        self.parameterInfo_23.setEnabled(flag)
        self.parameterInfo_24.setEnabled(flag)
        self.parameterInfo_25.setEnabled(flag)
        self.parameterInfo_26.setEnabled(flag)
        self.parameterInfo_27.setEnabled(flag)
        self.parameterInfo_28.setEnabled(flag)
        self.parameterInfo_29.setEnabled(flag)
        self.parameterInfo_30.setEnabled(flag)
        self.parameterInfo_31.setEnabled(flag)
        self.parameterInfo_32.setEnabled(flag)

    def textLabelsClear(self):
        # Clear all the text labels
        self.parameterInfo_1.setText("")
        self.parameterInfo_2.setText("")
        self.parameterInfo_3.setText("")
        self.parameterInfo_4.setText("")
        self.parameterInfo_5.setText("")
        self.parameterInfo_6.setText("")
        self.parameterInfo_7.setText("")
        self.parameterInfo_8.setText("")
        self.parameterInfo_9.setText("")
        self.parameterInfo_10.setText("")
        self.parameterInfo_11.setText("")
        self.parameterInfo_12.setText("")
        self.parameterInfo_13.setText("")
        self.parameterInfo_14.setText("")
        self.parameterInfo_21.setText("")
        self.parameterInfo_22.setText("")
        self.parameterInfo_23.setText("")
        self.parameterInfo_24.setText("")
        self.parameterInfo_25.setText("")
        self.parameterInfo_26.setText("")
        self.parameterInfo_27.setText("")
        self.parameterInfo_28.setText("")
        self.parameterInfo_29.setText("")
        self.parameterInfo_30.setText("")
        self.parameterInfo_31.setText("")
        self.parameterInfo_32.setText("")

    def msgbox(self, text):
        try:
            msg = QMessageBox(self)
            msg.setWindowModality(Qt.WindowModality.NonModal)
            msg.setWindowTitle("Information")
            msg.setText(text)
            msg.setStandardButtons(QMessageBox.StandardButton.Ok)
            msg.show()
            msg.exec()
        except:
            pass

    def error(self, flag):
        if flag:
            self.stopSerialCommunication()
            self.msgbox("There are errors in the operation of the COM port")
            self.refreshPorts()

    def update(self, line):

        if self.serialThread is None:
            return

        try:
            data = line.decode().strip("\r\n")
        except:
            data = ""

        try:
            if data != "":

                timestamp_ = datetime.datetime.now().strftime("Date: %Y-%m-%d Time: %H:%M:%S.%f")[:-3]

                # Check if the line contains the measurement data
                if data.startswith("ICurT:"):

                    # Extract the measurement values from the line
                    values = data.split()[1:]

                    # Calculate the currents using the formula
                    currents = [(math.sqrt(float(value) / (0.9 * 20)) * (self.VOLTAGE / 4096) * (1 / self.SENSITIVITY)) for value in values]
                    
                    self.current_0 = currents[0]
                    self.current_1 = currents[1]
                    self.current_2 = currents[2]
                    
                    current_average_filter_0 = self.current_average_filter_0.filtered(self.current_0)
                    current_average_filter_1 = self.current_average_filter_1.filtered(self.current_1)
                    current_average_filter_2 = self.current_average_filter_2.filtered(self.current_2)
                    
                    current_median_filter_0 = self.current_median_filter_0.filtered(self.current_0)
                    current_median_filter_1 = self.current_median_filter_1.filtered(self.current_1)
                    current_median_filter_2 = self.current_median_filter_2.filtered(self.current_2)
                    
                    self.parameterInfo_5.setText("{:.2f}".format(self.current_0))
                    self.parameterInfo_6.setText("{:.2f}".format(self.current_1))
                    self.parameterInfo_7.setText("{:.2f}".format(self.current_2))

                    self.parameterInfo_21.setText("{:.2f}".format(current_average_filter_0))
                    self.parameterInfo_22.setText("{:.2f}".format(current_average_filter_1))
                    self.parameterInfo_23.setText("{:.2f}".format(current_average_filter_2))
                    
                    self.parameterInfo_24.setText("{:.2f}".format(current_median_filter_0))
                    self.parameterInfo_25.setText("{:.2f}".format(current_median_filter_1))
                    self.parameterInfo_26.setText("{:.2f}".format(current_median_filter_2)) 

                    # Write the measurement values to the log file
                    if self.isActiveLog():
                        # Open the log file for writing
                        if self.logFileName() != "":
                            with open(self.logFileName(), "a") as f:
                                f.write("{} Currents: I1 = {:.2f} A, I2 = {:.2f} A, I3 = {:.2f} A\n".format(timestamp_,*currents))
                                f.flush()

                elif data.startswith("VRelayOut:"):

                    values_v = data.split()[1:]

                    # Calculate the voltages using the formula
                    voltages = [(math.sqrt(float(value_v) / (0.9 * 20)) * (self.VOLTAGE / 4096) * (461 / 1.385)) for value_v in values_v]

                    self.voltage_N = voltages[0]
                    self.voltage_0 = voltages[1]
                    self.voltage_1 = voltages[2]
                    self.voltage_2 = voltages[3]
                    
                    voltage_average_filter_0 = self.voltage_average_filter_0.filtered(self.voltage_0)
                    voltage_average_filter_1 = self.voltage_average_filter_1.filtered(self.voltage_1)
                    voltage_average_filter_2 = self.voltage_average_filter_2.filtered(self.voltage_2)
                    
                    voltage_median_filter_0 = self.voltage_median_filter_0.filtered(self.voltage_0)
                    voltage_median_filter_1 = self.voltage_median_filter_1.filtered(self.voltage_1)
                    voltage_median_filter_2 = self.voltage_median_filter_2.filtered(self.voltage_2)

                    # Update the voltage text boxes with the new values
                    self.parameterInfo_1.setText("{:.2f}".format(self.voltage_N))
                    self.parameterInfo_2.setText("{:.2f}".format(self.voltage_0))
                    self.parameterInfo_3.setText("{:.2f}".format(self.voltage_1))
                    self.parameterInfo_4.setText("{:.2f}".format(self.voltage_2))
                    
                    self.parameterInfo_27.setText("{:.2f}".format(voltage_average_filter_0))
                    self.parameterInfo_28.setText("{:.2f}".format(voltage_average_filter_1))
                    self.parameterInfo_29.setText("{:.2f}".format(voltage_average_filter_2))
                    
                    self.parameterInfo_30.setText("{:.2f}".format(voltage_median_filter_0))
                    self.parameterInfo_31.setText("{:.2f}".format(voltage_median_filter_1))
                    self.parameterInfo_32.setText("{:.2f}".format(voltage_median_filter_2)) 

                    # Write the measurement values to the log file
                    if self.isActiveLog():
                        # Open the log file for writing
                        if self.logFileName() != "":
                            with open(self.logFileName(), "a") as f:
                                f.write("{} Voltages: N = {:.2f} V, U1 = {:.2f} V, U2 = {:.2f} V, U3 = {:.2f} V\n".format(timestamp_, *voltages))
                                f.flush()

                elif data.startswith("Temp:"):

                    self.power_0 = self.voltage_0 * self.current_0
                    self.power_1 = self.voltage_1 * self.current_1
                    self.power_2 = self.voltage_2 * self.current_2
                    self.powers_sum = self.power_0 + self.power_1 + self.power_2

                    self.watt_hour_0 = self.energy_meter_0.compute(self.power_0)
                    self.watt_hour_1 = self.energy_meter_1.compute(self.power_1)
                    self.watt_hour_2 = self.energy_meter_2.compute(self.power_2)
                    
                    # Update the power text boxes with the new values
                    self.parameterInfo_8.setText("{:.2f}".format(self.power_0))
                    self.parameterInfo_9.setText("{:.2f}".format(self.power_1))
                    self.parameterInfo_10.setText("{:.2f}".format(self.power_2))

                    # Update the power Wh text boxes with the new values
                    self.parameterInfo_11.setText("{:.2f}".format(self.watt_hour_0))
                    self.parameterInfo_12.setText("{:.2f}".format(self.watt_hour_1))
                    self.parameterInfo_13.setText("{:.2f}".format(self.watt_hour_2))
                    
                    # Update the power sum text boxe with the new value
                    self.parameterInfo_14.setText("{:.2f}".format(self.powers_sum))

                    # Write the measurement values to the log file
                    if self.isActiveLog():
                        # Open the log file for writing
                        if self.logFileName() != "":
                            with open(self.logFileName(), "a") as f:
                                f.write("{} Powers: P1 = {:.2f} W, P2 = {:.2f} W, P3 = {:.2f} W\n".format(timestamp_,self.power_0,self.power_1,self.power_2))
                                f.write("{} Power_Wh: P1 = {:.2f} Wh, P2 = {:.2f} Wh, P3 = {:.2f} Wh\n".format(timestamp_, self.watt_hour_0, self.watt_hour_1, self.watt_hour_2))
                                f.write("{} Power_Sum: P = {:.2f} W\n".format(timestamp_, self.powers_sum))
                                f.flush()
                else:
                    pass
        except Exception as e:
            print(str(e))

    def timerRequestEvent(self):
        if self.serialThread is not None:
            self.serialThread.send("m\r")
            
    def timerReconnectionEvent(self):
        if self.serialThread is not None:
            self.serialThread.reconnect()

if __name__ == '__main__':
    # Create QApplication instance and show window
    app = QApplication(sys.argv)
    window = NeChargeMonitor()
    window.show()
    # Start event loop
    res = app.exec()
    window.serialStopThread()
    sys.exit(res)
