import matplotlib

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4']='PySide'

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

class MatplotlibWidget(FigureCanvas):
    def __init__(self, parent=None, width=500):
        self.figure = Figure()
        super(MatplotlibWidget, self).__init__(Figure())
        self.setParent(parent)
        self.width = width
        self.angle_axis = self.figure.add_subplot(211)
        self.control_axis = self.figure.add_subplot(212)
        self.clear()

    def clear(self):
        self.angle_data = [0] * self.width
        self.control_data = [0] * self.width
        self.angle_axis.clear()
        self.angle_line, = self.angle_axis.plot(self.angle_data)
        self.control_axis.clear()
        self.control_line, = self.control_axis.plot(self.control_data, 'r')

    def update_data(self, data, current_motor_control):
        # Do angle data
        if data and len(data) < self.width:
            self.angle_data = self.angle_data[len(data) - self.width:]
            self.angle_data.extend(data)
        elif len(data) >= self.width:
            self.angle_data = data[-self.width:]
        self.angle_line.set_ydata(self.angle_data)
        max_d = max(self.angle_data)
        min_d = min(self.angle_data)
        max_all = max([max_d, abs(min_d)])
        if max_all != 0:
            self.angle_axis.set_ylim(-max_all-10, max_all+10)
        else:
            self.angle_axis.set_ylim(-1, 1)
        # Do motor
        for i in range(5):
            self.control_data.pop(0)
            self.control_data.append(current_motor_control)
        self.control_line.set_ydata(self.control_data)
        max_d = max(self.control_data)
        min_d = min(self.control_data)
        max_all = max([max_d, abs(min_d)])
        if max_all != 0:
            self.control_axis.set_ylim(-max_all-10, max_all+10)
        else:
            self.control_axis.set_ylim(-1, 1)
        self.draw()


