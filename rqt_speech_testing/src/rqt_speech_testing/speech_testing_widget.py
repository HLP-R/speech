import os
import rospkg
import rospy
import signal

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QFileDialog, QIcon, QWidget, QMessageBox, QHeaderView, QTreeWidgetItem

class SpeechTestWidget(QWidget):
    """
    Widget for testing speech recognition
    Handles all widget callbacks
    """

    def __init__(self, context):
        super(SpeechTestWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path("rqt_speech_testing"), "resource", "SpeechTest.ui")
        loadUi(ui_file, self)

        self.setObjectName('SpeechTestUi')
        self.setWindowTitle(self.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self)

        # Set icons for buttons because they don't persist from Qt creator for some reason
        self.openLocationButton.setIcon(QIcon.fromTheme("document-open"))
        self.recordButton.setIcon(QIcon.fromTheme("media-record"))