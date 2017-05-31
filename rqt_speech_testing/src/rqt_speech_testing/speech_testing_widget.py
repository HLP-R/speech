import os
import signal
import threading
import time

import rospkg
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal, qWarning
from python_qt_binding.QtGui import (QApplication, QFileDialog, QHeaderView,
                                     QIcon, QMessageBox, QTreeWidgetItem,
                                     QWidget)

from hlpr_speech_msgs.msg import SpeechCommand, StampedString
from hlpr_speech_recognition.speech_recognizer import SpeechRecognizer


class SpeechTestWidget(QWidget):
    """
    Widget for testing speech recognition
    Handles all widget callbacks
    """

    def __init__(self, context):
        super(SpeechTestWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path(
            "rqt_speech_testing"), "resource", "SpeechTest.ui")
        loadUi(ui_file, self)

        self.setObjectName('SpeechTestUi')
        self.setWindowTitle(self.windowTitle() + (' (%d)' %
                                                  context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self)

        self.recognizer = SpeechRecognizer(subnode=True)
        recog_topic = rospy.get_param("/speech/publish_topic", "hlpr_speech_commands")
        msg_type = eval(rospy.get_param("/speech/command_type", "StampedString")) # True if message is only str, false includes header
        rospy.Subscriber(recog_topic, msg_type, self.speechCallback)
        self.currentRootItem = None

        # Set icons for buttons because they don't persist from Qt creator
        self.openLocationButton.setIcon(QIcon.fromTheme("document-open"))
        self.recordButton.setIcon(QIcon.fromTheme("media-record"))

        # Attach event handlers
        self.openLocationButton.clicked[bool].connect(self.openAudio)
        self.location.returnPressed.connect(self.loadAudio)
        self.recordButton.toggled.connect(self.recordAudio)

        # Set sizing options for tree widget headers
        self.outputTree.header().setStretchLastSection(False)
        self.outputTree.header().setResizeMode(0, QHeaderView.Stretch)
        self.outputTree.header().setResizeMode(1, QHeaderView.ResizeToContents)

    def openAudio(self):
        location = QFileDialog.getOpenFileName(filter="*.wav;;*", directory=os.path.dirname(self.location.text()))[0]
        if not location:
            return
        self.location.setText(location)
        self.loadAudio()

    def loadAudio(self):
        location = self.location.text()

        self.currentRootItem = QTreeWidgetItem()
        self.currentRootItem.setText(0, location)
        self.outputTree.addTopLevelItem(self.currentRootItem)
        self.currentRootItem.setExpanded(True)

        QApplication.setOverrideCursor(Qt.WaitCursor)
        self.recognizer.begin_rec(file=location)
        QApplication.restoreOverrideCursor()

    def recordAudio(self, state):
        if state:
            self.currentRootItem = QTreeWidgetItem()
            self.currentRootItem.setText(0, "Recording")
            self.outputTree.addTopLevelItem(self.currentRootItem)
            self.currentRootItem.setExpanded(True)

            threading.Thread(target=self.recordAudioThread).start()
        else:
            self.recognizer.end_rec()

    def loadAudioThread(self, file):
        QApplication.setOverrideCursor(Qt.WaitCursor)
        self.recognizer.begin_rec(file=file)
        QApplication.restoreOverrideCursor()

    def recordAudioThread(self):
        self.recognizer.begin_rec()
    
    def speechCallback(self, msg):
        if msg._type == "hlpr_speech_msgs/StampedString":
            last_string = msg.keyphrase
            last_ts = msg.stamp
        else:
            last_string = msg.data

        item = QTreeWidgetItem()
        item.setText(0, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())))
        item.setText(1, last_string)
        self.currentRootItem.addChild(item)
        self.outputTree.scrollToItem(item)
