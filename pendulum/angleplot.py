import matplotlib

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4']='PySide'

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

class AnglePlot(FigureCanvas):
    """This is a widget that will plot the angle of the pendulum"""
    def __init__(self, parent=None):
        self.figure = Figure()
        super(AnglePlot, self).__init__(self.figure)
        self.clear()

    def clear(self):
        self.line, 
