#!/usr/bin/env python

import sys
# This is a hack to work in sublime
sys.path.append("/usr/local/lib/python2.7/site-packages")
from PySide import QtCore, QtGui
import numpy as np

from threading import Thread, Lock

from serial import Serial
from serial.tools import list_ports

from pendulum.mainwindow import Ui_MainWindow

class PendulumControl(QtGui.QMainWindow):
    """Pendulum Control Mainwindow"""
    def __init__(self, ui):
        QtGui.QMainWindow.__init__(self)
        self.ui = ui

        self.serial = None
        self.quit = False
        self.connected = False
        self.read_thread = None
        self.update_period = 100
        self.plot_width = 500
        self.current_motor_control = 0
        self.count = 0
        self.update_timer = QtCore.QTimer()

        self.data = []
        self.data_lock = Lock()

        self.ui.setupUi(self)

        self.ui.connect_button.clicked.connect(self.on_connect)
        self.update_timer.timeout.connect(self.update_plot)
        self.ui.motor_control.valueChanged.connect(self.on_motor_control_change)
        self.ui.zero_button.clicked.connect(self.on_zero)
        self.update_timer.start(self.update_period)

    def on_zero(self):
        self.ui.motor_control.setValue(0)

    def on_motor_control_change(self, val):
        self.current_motor_control = float(val)

    def update_plot(self):
        data = []
        with self.data_lock:
            data = list(self.data)
            self.data = []
        if data:
            self.ui.angle_label.setText("Pendulum angle: {0}".format(data[-1]))
        self.ui.angle_plot.update_data(data, self.current_motor_control)
        if self.count == 0:
            if self.serial and self.serial.isOpen():
                self.serial.write(str(int(self.current_motor_control))+"\n")
            self.count = 0
        else:
            self.count += 1

    def closeEvent(self, event):
        self.disconnect()

    def on_connect(self):
        if self.connected:
            self.disconnect()
        else:
            temp = list_ports.comports()
            ports = []
            for t in temp:
                ports.append(t[0])
            port, accepted = QtGui.QInputDialog.getItem(self, "Select Serial Port", "Serial Port:", ports, editable=True)
            if accepted:
                self.connect(port)

    def connect(self, port):
        try:
            self.ui.statusbar.showMessage("Connecting to {0}".format(port))
            self.serial = Serial(port, 115200)
            self.serial.timeout = 1
            self.connected = True
            self.ui.connect_button.setText("Disconnect")
            self.read_thread = Thread(target=self.read)
            self.read_thread.start()
        except Exception as e:
            self.ui.statusbar.showMessage(str(e))

    def disconnect(self):
        self.connected = False
        if self.read_thread:
            self.read_thread.join()

    def disconnect_(self):
        if self.serial:
            self.serial.close()
        self.serial = None
        self.ui.connect_button.setText("Connect")
        self.connected = False
        self.ui.statusbar.showMessage("Disconnected")
        self.ui.angle_plot.clear()

    def read(self):
        """Reads from the serial port continuously"""
        self.ui.statusbar.showMessage("Connecting...")
        while self.connected and not self.quit:
                line = self.serial.readline()
                if "System Ready" in line:
                    break
        self.ui.statusbar.showMessage("Connected")
        while self.connected and not self.quit:
            try:
                val = self.serial.readline()
                val = float(val)
                with self.data_lock:
                    self.data.append(val)
            except Exception as e:
                pass
        self.update_timer.stop()
        self.disconnect_()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    ui_mainwindow = Ui_MainWindow()
    pendulum_control = PendulumControl(ui_mainwindow)
    pendulum_control.show()
    sys.exit(app.exec_())
