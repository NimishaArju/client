# -----------------------------------------------------------
# Multi-Boat Client
#
# (C) 2021 SAVTOA AUTONOMOUS, India
# -----------------------------------------------------------

"""PyQt5 Modules"""
# -----------------------------------------------------------
from Camera import ipCamera
from Camera import videoStream
import psutil
import signal
import PyQt5.QtGui
from PyQt5 import QtCore, QtGui
from PyQt5 import QtWidgets, QtWebEngineWidgets, uic
from PyQt5.QtCore import *
from PyQt5.QtGui import QColor, QFont, QBrush
from PyQt5.QtGui import QPixmap, QTransform, QImage
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import *
from gi.repository import Gdk, Wnck

# -----------------------------------------------------------

""" geojson Modules """
# -----------------------------------------------------------
from geojsonio import make_url
from geojson import Point, Feature, FeatureCollection, dump, loads

# -----------------------------------------------------------

""" Other Modules """
# -----------------------------------------------------------
import re
import time
import sys
import os
import inspect
import numpy
import math
import ctypes
from ctypes import *
import geopy.distance

# -----------------------------------------------------------

""" External Modules """
# -----------------------------------------------------------

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from wrapper.wrapper import RDClient, Autonomous, Geometric, Survey
from Joystick import JoystickFunc

from Commands.commands import *
from Resources.Radar import radar

# -----------------------------------------------------------

""" Global Declarations """
# -----------------------------------------------------------
libCalc = CDLL("./C++Files_SurveyMode/BS/libobject_file.so")  # loading C++ shared object file

# Initialize External Class Objects
gRDClient = RDClient()
gAutonomous = Autonomous()
gGeometric = Geometric()
gSurvey = Survey()

# global Variables
ctypearray = ctypes.c_float * 64
gWaypointdata = ctypearray()
gGpsdata = numpy.empty(20, dtype=object)

# calling functions from C++
libCalc.main()
libCalc.gSuperClientHold()
libCalc.gProcessClose()


# -----------------------------------------------------------


# semaphore receiving Qthread for gps data
# -----------------------------------------------------------
class GpsReceivingThread(QThread):
    gpsSignal = pyqtSignal()  # Initialize pyqt signal

    def run(self):
        while True:
            libCalc.semWait()
            libCalc.gps_data.restype = ctypes.POINTER(ctypes.c_float)
            self.gpsSignal.emit()  # emit signals to an external function when data arrives


class GeometricReceivingThread(QThread):
    geometricSignal = pyqtSignal()  # Initialize pyqt signal

    def run(self):
        while True:
            libCalc.geometric_semWait()
            libCalc.geometric_data.restype = ctypes.POINTER(ctypes.c_float)
            self.geometricSignal.emit()  # emit signals to an external function when data arrives


# -----------------------------------------------------------

# joystick Qthread
# -----------------------------------------------------------
class JoystickSendThread(QThread):
    def run(self):
        print("Joystick Thread")
        JoystickFunc.joystick(window.serverclicked)  # calling external joystick class & passing current server index


# -----------------------------------------------------------


# javascript class for changing web attributes
# -----------------------------------------------------------
class WebEnginePage(QtWebEngineWidgets.QWebEnginePage):
    def javaScriptConsoleMessage(self, level, msg, line, sourceID):
        pass

    def javaScriptConfirm(self, securityOrigin: QUrl, msg: str):
        pass


# -----------------------------------------------------------


# UI Class
class Ui(QtWidgets.QMainWindow):
    def __init__(self):  # Class Init Func
        super(Ui, self).__init__()
        uic.loadUi('./Resources/QT/client_opencpn.ui', self)

        self.processname = "AppRun.wrapped"
        self.processnameOpenCpn = "opencpn"

        self.qurl = QUrl("https://geojson.io/")
        self.browser = QWebEngineView()

        self.camerabrowser = QWebEngineView()
        qurl = QUrl("http://192.168.1.70:9998/index.html")
        self.camerabrowser.setUrl(qurl)

        self.weather = QWebEngineView()
        self.engine = QWebEngineView()
        self.gauge = QWebEngineView()
        self.map = QWebEngineView()

        self.weatherqurl = QUrl("http://localhost:3001/d/YQnBqYTnk/real-time-weather?orgId=1&refresh=5s&kiosk")
        self.enginequrl = QUrl("http://localhost:3001/d/pTEN3LT7k/electrical-system?orgId=1&refresh=5s&kiosk")
        self.gaugequrl = QUrl("http://localhost:3001/d/DSNfkR07z/gauges?orgId=1&refresh=5s&kiosk")
        self.mapqurl = QUrl("http://localhost:3001/d/Lnmtwao7k/map?orgId=1&refresh=5s&kiosk")

        self.page = WebEnginePage(self.browser)
        self.browser.setPage(self.page)
        self.browser.setUrl(self.qurl)
        self.map_layout.addWidget(self.browser)

        self.gpsThread = GpsReceivingThread()
        self.gpsThread.start()
        self.geometricThread = GeometricReceivingThread()
        self.geometricThread.start()
        self.gpsThread.gpsSignal.connect(self.GpsEvent)
        self.geometricThread.geometricSignal.connect(self.GeometricEvent)

        self.joystickThread = JoystickSendThread()

        self.processid = None
        self.filestatus = False
        self.popupstatus = False
        self.satelliteviewstatus = True
        self.autonomousmode = False
        self.loadfinishedstatus = False
        self.currentgpsstatus = True
        self.nogpssignal = True
        self.markerpopup = False
        self.nomarkerpopup = False
        self.startbuttonconfirm = False
        self.geometricconfirm = False
        self.mainscreenstatus = True

        self.camera1identifier = False
        self.camera2identifier = False
        self.camera3identifier = False
        self.camera4identifier = False
        self.camera5identifier = False
        self.camera6identifier = False

        self.process1identifier = False
        self.process2identifier = False
        self.process3identifier = False
        self.process4identifier = False
        self.process5identifier = False
        self.process6identifier = False

        self.sonarcount = 0
        self.index = 0
        self.updatedsavefilepath = 0
        self.serverindex = 0
        self.autonomousindex = 0
        self.refreshindex = 0
        self.waypointcount = 0
        self.refreshcount = 0
        self.autonomousrefreshcount = 0
        self.autonomouscount = 0
        self.previouslength = 0
        self.servernoindex = 0
        self.serverclicked = 0
        self.serverupdated = 0
        self.degreematching = 0
        self.positionnumber = 0
        self.latitudelist = []
        self.longitudelist = []
        self.indexlist = []
        self.uniquelist = []
        self.markerlist = []
        self.nonzerolist = []
        self.ANCuniquelist = []
        self.shape = ""

        self.coordinatechecklist = [(0, 0)]
        self.autonomousrunningstatus = [False, False, False, False, False, False, False, False, False, False]
        self.autonomousfilenamesave = ["", "", "", "", "", "", "", "", ]
        self.gpslist = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]
        self.modelist = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0),
                         (0, 0, 0), (0, 0, 0)]

        self.circlecoordinatelist = [(430, 310), (340, 290), (290, 230), (270, 160), (280, 90), (350, 20), (430, 0),
                                     (510, 30), (560, 90), (580, 160), (560, 240), (500, 290), (430, 310)]

        self.eightcoordinatelist = [(430, 330), (350, 290), (350, 200), (430, 160), (500, 120), (500, 40), (430, -10),
                                    (350, 40), (350, 110), (430, 160), (500, 200), (500, 280), (430, 330)]

        self.eightanglelist = [60, 120, 180, 120, 60, 0, 300, 240, 180, 240, 300, 0]

        self.linearcoordinatelist = [(430, 248), (430, 186), (430, 124), (430, 62), (430, 0)]

        self.tableWidget = QTableWidget()
        self.tableWidget.setRowCount(1)
        self.tableWidget.setColumnCount(3)
        self.tableWidget.horizontalHeader().hide()
        self.tableWidget.verticalHeader().hide()
        self.tableWidget.setSelectionMode(QAbstractItemView.NoSelection)
        self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)

        self.video_stream_frame.setVisible(True)

        self.mode_selector.setVisible(False)
        self.routing_window_button_frame.setVisible(False)

        self.diagonal_mode_frame.setVisible(False)
        self.coordinate_mode_frame.setVisible(False)

        self.geometric_frame.setVisible(False)
        self.animation_frame.setVisible(False)
        self.circle_frame.setVisible(False)
        self.eight_frame.setVisible(False)
        self.linear_frame.setVisible(False)
        self.circle_navigation_completed.setVisible(False)
        self.eight_navigation_completed.setVisible(False)
        self.linear_navigation_completed.setVisible(False)

        self.circle_label.setVisible(False)
        self.eight_label.setVisible(False)
        self.linear_line.setVisible(False)

        self.boat_label_1.setVisible(False)
        self.eight_boat.setVisible(False)
        self.linear_boat_label.setVisible(False)

        self.boat = QPixmap("./Resources/Icons/boat_top_view1.png")

        self.anim = QPropertyAnimation(self.boat_label_1, b"geometry")
        self.anim.setDuration(1000)

        self.animateeight = QPropertyAnimation(self.eight_boat, b"geometry")
        self.animateeight.setDuration(1000)

        self.animatelinear = QPropertyAnimation(self.linear_boat_label, b"geometry")
        self.animatelinear.setDuration(1000)

        self.markerclicktimer = QTimer()
        self.markerclicktimer.timeout.connect(self.MarkerClickTimerFunc)

        self.sonartimer = QTimer()
        self.sonartimer.timeout.connect(self.SonarTimerFunc)
        # self.Embed('./pingviewer-Release.AppImage')

        self.logo = QPixmap('./Resources/Images/savtoa-logo-web.png')
        self.logo_label.setPixmap(self.logo)
        self.logo_label.setStyleSheet("background-color: white;")

        self.left_image = QPixmap('./Resources/Images/left_image.jpg')
        self.left_image_label.setPixmap(self.left_image)
        self.left_image_label.setStyleSheet("background-color: white;")

        self.right_image = QPixmap('./Resources/Images/right_image.jpg')
        self.right_image_label.setPixmap(self.right_image)
        self.right_image_label.setStyleSheet("background-color: white;")

        self.dashboard_image = QPixmap('./Resources/Images/ship_dashboard.png')
        self.dashboard_label.setPixmap(self.dashboard_image)
        self.dashboard_label.setStyleSheet("background-color: white;")

        self.weather_button.clicked.connect(self.WeatherButtonClicked)
        self.engine_button.clicked.connect(self.EngineButtonClicked)
        self.gauge_button.clicked.connect(self.GaugeButtonClicked)
        self.map_button.clicked.connect(self.MapButtonClicked)

        self.video_label1 = QLabel()
        self.video_label2 = QLabel()
        self.video_label3 = QLabel()
        self.video_label4 = QLabel()
        self.video_label5 = QLabel()
        self.video_label6 = QLabel()

        self.process_label1 = QLabel()
        self.process_label2 = QLabel()
        self.process_label3 = QLabel()
        self.process_label4 = QLabel()
        self.process_label5 = QLabel()
        self.process_label6 = QLabel()

        self.video_label1.setScaledContents(True)
        self.video_label2.setScaledContents(True)
        self.video_label3.setScaledContents(True)
        self.video_label4.setScaledContents(True)
        self.video_label5.setScaledContents(True)
        self.video_label6.setScaledContents(True)

        self.process_label1.setScaledContents(True)
        self.process_label2.setScaledContents(True)
        self.process_label3.setScaledContents(True)
        self.process_label4.setScaledContents(True)
        self.process_label5.setScaledContents(True)
        self.process_label6.setScaledContents(True)

        self.video_label1.setVisible(False)
        self.video_label2.setVisible(False)
        self.video_label3.setVisible(False)
        self.video_label4.setVisible(False)
        self.video_label5.setVisible(False)
        self.video_label6.setVisible(False)

        self.process_label1.setVisible(False)
        self.process_label2.setVisible(False)
        self.process_label3.setVisible(False)
        self.process_label4.setVisible(False)
        self.process_label5.setVisible(False)
        self.process_label6.setVisible(False)

        self.camera1Obj = videoStream.ObjectDetection1()
        self.camera2Obj = videoStream.ObjectDetection2()
        self.camera3Obj = videoStream.ObjectDetection3()
        self.camera4Obj = videoStream.ObjectDetection4()
        self.camera5Obj = videoStream.ObjectDetection5()
        self.camera6Obj = videoStream.ObjectDetection6()

        self.process1Obj = ipCamera.ObjectDetection1()
        self.process2Obj = ipCamera.ObjectDetection2()
        self.process3Obj = ipCamera.ObjectDetection3()
        self.process4Obj = ipCamera.ObjectDetection4()
        self.process5Obj = ipCamera.ObjectDetection5()
        self.process6Obj = ipCamera.ObjectDetection6()

        self.radarObj = radar.Radar()
        self.radarObj.changePixmap.connect(self.setRadar)

        self.radar_label.setScaledContents(True)
        self.radar_label.move(300, 2)
        self.radar_label.setVisible(False)
        self.radar_label_2.setVisible(False)

        self.camera1Obj.changePixmap.connect(self.setImage1)
        self.camera2Obj.changePixmap.connect(self.setImage2)
        self.camera3Obj.changePixmap.connect(self.setImage3)
        self.camera4Obj.changePixmap.connect(self.setImage4)
        self.camera5Obj.changePixmap.connect(self.setImage5)
        self.camera6Obj.changePixmap.connect(self.setImage6)

        self.process1Obj.changePixmap.connect(self.setProcess1)
        self.process2Obj.changePixmap.connect(self.setProcess2)
        self.process3Obj.changePixmap.connect(self.setProcess3)
        self.process4Obj.changePixmap.connect(self.setProcess4)
        self.process5Obj.changePixmap.connect(self.setProcess5)
        self.process6Obj.changePixmap.connect(self.setProcess6)

        self.camera1_button.clicked.connect(self.Camera1ButtonClicked)
        self.camera2_button.clicked.connect(self.Camera2ButtonClicked)
        self.camera3_button.clicked.connect(self.Camera3ButtonClicked)
        self.camera4_button.clicked.connect(self.Camera4ButtonClicked)
        self.camera5_button.clicked.connect(self.Camera5ButtonClicked)
        self.camera6_button.clicked.connect(self.Camera6ButtonClicked)
        self.video_view_all_button.clicked.connect(self.ViewAllButtonClicked)

        self.process1_button.clicked.connect(self.Process1ButtonClicked)
        self.process2_button.clicked.connect(self.Process2ButtonClicked)
        self.process3_button.clicked.connect(self.Process3ButtonClicked)
        self.process4_button.clicked.connect(self.Process4ButtonClicked)
        self.process5_button.clicked.connect(self.Process5ButtonClicked)
        self.process5_button.setVisible(False)
        self.process6_button.setVisible(False)
        self.process6_button.clicked.connect(self.Process6ButtonClicked)
        self.process_view_all_button.clicked.connect(self.ProcessViewAllButtonClicked)

        self.sonar_button.clicked.connect(self.SonarButtonClicked)
        self.radar_button.clicked.connect(self.RadarButtonClicked)

        self.radar_close_button.clicked.connect(self.RadarCloseButtonClicked)
        self.radar_close_button.setVisible(False)
        self.radar_close_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/exit_button.png);} "
            "QPushButton::hover{ background-color : silver;}"
            "QPushButton::pressed{ background-color : red;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : brown;}")

        self.sensor_close_button.clicked.connect(self.SensorCloseButtonClicked)
        self.sensor_close_button.setVisible(False)
        self.sensor_close_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/exit_button.png);} "
            "QPushButton::hover{ background-color : silver;}"
            "QPushButton::pressed{ background-color : red;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : brown;}")

        self.video_stream_close_button.setToolTip('Exit Window')
        self.video_stream_close_button.setVisible(False)
        self.video_stream_close_button.clicked.connect(self.VideoStreamCloseButtonClicked)
        self.video_stream_close_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/exit_button.png);} "
            "QPushButton::hover{ background-color : silver;}"
            "QPushButton::pressed{ background-color : red;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : brown;}")

        self.process_close_button.setToolTip('Exit Window')
        self.process_close_button.setVisible(False)
        self.process_close_button.clicked.connect(self.ProcessCloseButtonClicked)
        self.process_close_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/exit_button.png);} "
            "QPushButton::hover{ background-color : silver;}"
            "QPushButton::pressed{ background-color : red;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : brown;}")

        self.dashboard_close_button.setToolTip('Exit Window')
        self.dashboard_close_button.setVisible(False)
        self.dashboard_close_button.clicked.connect(self.DashboardCloseButtonClicked)
        self.dashboard_close_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/exit_button.png);} "
            "QPushButton::hover{ background-color : silver;}"
            "QPushButton::pressed{ background-color : red;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : brown;}")

        self.mode_selector_close_button.clicked.connect(self.ModeSelectorCloseButtonClicked)

        self.diagonal_mode_button.clicked.connect(self.DiagonalModeButtonClicked)
        self.coordinate_mode_button.clicked.connect(self.CoordinateModeButtonClicked)

        self.diagonal_ok_button.clicked.connect(self.DiagonalOkButtonClicked)
        self.coordinate_ok_button.clicked.connect(self.CoordinateOkButtonClicked)

        self.diagonal_cancel_button.clicked.connect(self.DiagonalCancelButtonClicked)
        self.coordinate_cancel_button.clicked.connect(self.CoordinateCancelButtonClicked)

        self.geometric_stop_button.setToolTip('Stop')
        self.geometric_stop_button.clicked.connect(self.GeometricStopButtonClicked)
        self.geometric_stop_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/stop_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.navigation_button.setVisible(False)
        self.navigation_button.setToolTip('Waypoint')
        self.navigation_button.clicked.connect(self.NavigationButtonClicked)
        self.navigation_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/navigation_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.start_button.setVisible(False)
        self.start_button.setToolTip('Start')
        self.start_button.clicked.connect(self.StartButtonClicked)
        self.start_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/start_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.survey_start_button.setVisible(False)
        self.survey_start_button.setToolTip('Start')
        self.survey_start_button.clicked.connect(self.SurveyStartButtonClicked)
        self.survey_start_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/start_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.start_all_button.setVisible(False)
        self.start_all_button.setToolTip('Start All')
        self.start_all_button.clicked.connect(self.StartAllButtonClicked)
        self.start_all_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/start_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.stop_button.setVisible(False)
        self.stop_button.setToolTip('Stop')
        self.stop_button.clicked.connect(self.StopButtonClicked)
        self.stop_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/stop_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.survey_stop_button.setVisible(False)
        self.survey_stop_button.setToolTip('Stop')
        self.survey_stop_button.clicked.connect(self.SurveyStopButtonClicked)
        self.survey_stop_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/stop_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.stop_all_button.setVisible(False)
        self.stop_all_button.setToolTip('Stop All')
        self.stop_all_button.clicked.connect(self.StopAllButtonClicked)
        self.stop_all_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/stop_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.save_button.setToolTip('Save Waypoint')
        self.save_button.clicked.connect(self.SaveButtonClicked)
        self.save_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/save_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.load_button.setToolTip('Load Waypoint')
        self.load_button.clicked.connect(self.LoadButtonClicked)
        self.load_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/load_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.route_button.setToolTip('Waypoint Routing')
        self.route_button.clicked.connect(self.RouteButtonClicked)
        self.route_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/route_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.exit_button.setToolTip('Exit Routing')
        self.exit_button.setVisible(False)
        self.exit_button.clicked.connect(self.ExitButtonClicked)
        self.exit_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/exit_button.png);} "
            "QPushButton::hover{ background-color : silver;}"
            "QPushButton::pressed{ background-color : red;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : brown;}")

        self.autonomous_window_close_button.setToolTip('Exit Window')
        self.autonomous_window_close_button.setVisible(False)
        self.autonomous_window_close_button.clicked.connect(self.AutonomousWindowCloseButtonClicked)
        self.autonomous_window_close_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/exit_button.png);} "
            "QPushButton::hover{ background-color : silver;}"
            "QPushButton::pressed{ background-color : red;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : brown;}")

        self.zoom_in_button.setToolTip('Zoom In Map')
        self.zoom_in_button.clicked.connect(self.ZoomInButtonClicked)
        self.zoom_in_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/zoom_in_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.zoom_out_button.setToolTip('Zoom Out Map')
        self.zoom_out_button.clicked.connect(self.ZoomOutButtonClicked)
        self.zoom_out_button.setStyleSheet(
            "QPushButton{ border-image: url(./Resources/Icons/zoom_out_button.png);} "
            "QPushButton::hover{ background-color : #aaffff;}"
            "QPushButton::pressed{ background-color : #00007f;}"
            "QPushButton{ border-radius : 10;}"
            "QPushButton{ background-color : #00007f;}")

        self.circle_button.clicked.connect(self.CircleButtonClicked)
        self.eight_button.clicked.connect(self.EightButtonClicked)
        self.linear_button.clicked.connect(self.LinearButtonClicked)
        self.geometric_confirm_button.clicked.connect(self.GeometricConfirmButtonClicked)
        self.geometric_cancel_button.clicked.connect(self.GeometricCancelButtonClicked)

        self.browser.page().profile().downloadRequested.connect(self.DownloadRequested)

        self.boat_joystick_button.clicked.connect(self.BoatJoystickButtonClicked)

        self.base_station_joystick_button.clicked.connect(self.BaseStationJoystickButtonClicked)

        self.autonomous_button.clicked.connect(self.AutonomousButtonClicked)

        self.survey_button.clicked.connect(self.SurveyButtonClicked)

        self.geometric_button.clicked.connect(self.GeometricButtonClicked)

        self.browser.loadFinished.connect(self.OnLoadFinished)

        self.setStyleSheet("QTabWidget>QWidget>QWidget{background:white;}")

        self.tabWidget.setCurrentIndex(0)

        self.settings_sub_tab.setTabEnabled(1, 0)
        self.settings_sub_tab.setCurrentIndex(0)
        self.settings_sub_tab.setStyleSheet(
            "QTabBar::tab:enabled { width: 158px; border: 0px; color: transparent; background: transparent;}"
            "QTabBar::tab:disabled { width: 158px; border: 0px; color: transparent; background: transparent;}")

        self.general_button.clicked.connect(self.GeneralButtonClicked)
        self.general_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : white;}")

        self.log_button.clicked.connect(self.LogButtonClicked)
        self.log_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

        self.survey_settings_button.clicked.connect(self.SurveySettingsButtonClicked)
        self.survey_settings_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

        self.connection_ip.textChanged.connect(self.ConnectionIpTextChanged)
        self.connection_port.textChanged.connect(self.ConnectionPortTextChanged)
        self.connect_button.clicked.connect(self.ConnectButtonClicked)

        self.tabWidget.currentChanged.connect(self.TabChangeEvent)

        self.markercolor = ['#8a8a8a', '#8a8a8a', '#8a8a8a', '#8a8a8a', '#8a8a8a', '#8a8a8a', '#8a8a8a', '#8a8a8a',
                            '#8a8a8a', '#8a8a8a']

        self.setFixedSize(1100, 600)
        self.test_button = QPushButton()

        self.show()

    def DiagonalModeButtonClicked(self):
        self.diagonal_mode_button.setChecked(True)
        self.coordinate_mode_button.setChecked(False)

        self.diagonal_mode_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 12; background-color : white; border: 2px solid #00007f;}")
        self.coordinate_mode_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 12; background-color : #00007f;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

    def CoordinateModeButtonClicked(self):
        self.diagonal_mode_button.setChecked(False)
        self.coordinate_mode_button.setChecked(True)

        self.coordinate_mode_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 12; background-color : white; border: 2px solid #00007f;}")
        self.diagonal_mode_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 12; background-color : #00007f;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

    def SurveyStartButtonClicked(self):
        gRDClient.setDeviceMode(self.serverclicked, SURVEY_ON_STATE)
        self.survey_start_button.setVisible(False)
        self.survey_stop_button.setVisible(True)

    def SurveyStopButtonClicked(self):
        gRDClient.setDeviceMode(self.serverclicked, SURVEY_OFF_STATE)
        self.survey_start_button.setVisible(False)
        self.survey_stop_button.setVisible(False)

    def SurveyButtonClicked(self):
        self.mode_selector_close_button.click()
        gRDClient.setDeviceMode(self.serverclicked, MODE_SURVEY)
        if self.diagonal_mode_button.isChecked():
            self.diagonal_mode_frame.setVisible(True)
        if self.coordinate_mode_button.isChecked():
            self.coordinate_mode_frame.setVisible(True)

    def DiagonalOkButtonClicked(self):
        if not self.diagonal_length.text():
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Information)
            msgBox.setText("INVALID VALUE")
            msgBox.setWindowTitle("ERROR")
            msgBox.setStyleSheet(
                "QMessageBox{ background-color : #aaffff;} "
                "QLabel{ color : #00007f;}"
                "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                "QPushButton::pressed{ background-color : #00007f; color : white;}"
                "QPushButton{ background-color : #00007f;}"
                "QPushButton{ color : white;}")
            msgBox.exec()

        if int(self.diagonal_length.text()) >= 10:
            gSurvey.send_Survey_data(self.serverclicked, int(int(self.diagonal_length.text()) / math.sqrt(2)))
            self.diagonal_mode_frame.setVisible(False)
            self.survey_start_button.setVisible(True)
        elif 10 > int(self.diagonal_length.text()) >= 0:
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Information)
            msgBox.setText("VALUE MUST BE ATLEAST 10")
            msgBox.setWindowTitle("ALERT")
            msgBox.setStyleSheet(
                "QMessageBox{ background-color : #aaffff;} "
                "QLabel{ color : #00007f;}"
                "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                "QPushButton::pressed{ background-color : #00007f; color : white;}"
                "QPushButton{ background-color : #00007f;}"
                "QPushButton{ color : white;}")
            msgBox.exec()

    def CoordinateOkButtonClicked(self):
        if self.coordinate_lat.text() and self.coordinate_long.text():
            coords_1 = (
                float(self.coordinate_lat.text().split(',')[0]), float(self.coordinate_lat.text().split(',')[1]))
            coords_2 = (
                float(self.coordinate_long.text().split(',')[0]), float(self.coordinate_long.text().split(',')[1]))
            distance = geopy.distance.distance(coords_1, coords_2).meters
            gSurvey.send_Survey_data(self.serverclicked, int(int(distance) / math.sqrt(2)))
            self.coordinate_mode_frame.setVisible(False)
            self.survey_start_button.setVisible(True)

    def DiagonalCancelButtonClicked(self):
        self.diagonal_mode_frame.setVisible(False)

    def CoordinateCancelButtonClicked(self):
        self.coordinate_mode_frame.setVisible(False)

    def SonarTimerFunc(self):
        self.sonartimer.stop()
        for procs in psutil.process_iter():
            if self.processname in procs.name():
                self.processid = procs.pid
        print(self.processid)
        # if not started:
        #      QtWidgets.QMessageBox.critical(self, 'Command "{}" not started!'.format(command), "Eh")
        #      return
        attempts = 0
        while attempts < 10:
            screen = Wnck.Screen.get_default()
            screen.force_update()
            time.sleep(0.1)
            while Gdk.events_pending():
                Gdk.event_get()
            for w in screen.get_windows():
                if w.get_pid() == self.processid:
                    # self.window = QtGui.QWindow.fromWinId(w.get_xid())
                    self.proc.setParent(self)
                    win32w = QtGui.QWindow.fromWinId(w.get_xid())  # this finally works
                    win32w.setFlags(QtCore.Qt.FramelessWindowHint)
                    self.widg = QtWidgets.QWidget.createWindowContainer(win32w)
                    # self.widg.setVisible(False)
                    self.sensor_layout.addWidget(self.widg)
                    return
            attempts += 1
        QtWidgets.QMessageBox.critical(self, 'Window not found', 'Process started but window not found')

    def OpenCpnTimerFunc(self):
        for procs in psutil.process_iter():
            if self.processnameOpenCpn in procs.name():
                self.processid = procs.pid
        print(self.processid)
        # if not started:
        #      QtWidgets.QMessageBox.critical(self, 'Command "{}" not started!'.format(command), "Eh")
        #      return
        attempts = 0
        while attempts < 10:
            screen = Wnck.Screen.get_default()
            screen.force_update()
            time.sleep(0.1)
            while Gdk.events_pending():
                Gdk.event_get()
            for w in screen.get_windows():
                if w.get_pid() == self.processid:
                    # self.window = QtGui.QWindow.fromWinId(w.get_xid())
                    self.proc.setParent(self)
                    win32w = QtGui.QWindow.fromWinId(w.get_xid())  # this finally works
                    win32w.setFlags(QtCore.Qt.FramelessWindowHint)
                    self.widg = QtWidgets.QWidget.createWindowContainer(win32w)
                    # self.widg.setVisible(False)
                    self.opencpn_layout.addWidget(self.widg)
                    return
            attempts += 1
        QtWidgets.QMessageBox.critical(self, 'Window not found', 'Process started but window not found')

    def SensorCloseButtonClicked(self):
        self.sensor_close_button.setVisible(False)
        self.widg.setVisible(False)

    def RadarCloseButtonClicked(self):
        self.radar_label_2.setVisible(False)
        self.radar_close_button.setVisible(False)
        self.sensor_frame.setVisible(True)
        self.radar_label.setVisible(False)
        self.radar_label.setVisible(False)

    def RadarButtonClicked(self):
        self.radar_label_2.setVisible(True)
        self.radar_label.setVisible(True)
        self.sensor_frame.setVisible(False)
        self.radar_close_button.setVisible(True)
        self.radarObj.start()

    def SonarButtonClicked(self):
        self.sonarcount = self.sonarcount + 1
        self.sensor_close_button.setVisible(True)
        if self.sonarcount == 1:
            # videoStreamingfilethread = ping_viewer_app_thread()
            # videoStreamingfilethread.start()
            # time.sleep(5)
            # self.Embed('Ping Viewer')
            self.Embed('./Resources/PingViewer/pingviewer-Release.AppImage')
            self.widg.setVisible(True)
        else:
            self.widg.setVisible(True)

    def Embed(self, command, *args):
        self.proc = QtCore.QProcess()
        self.proc.setProgram(command)
        self.proc.setArguments(args)

        self.proc.startDetached()
        self.sonartimer.start(3000)

    def EmbedOpenCpn(self, command, *args):
        self.proc = QtCore.QProcess()
        self.proc.setProgram(command)
        self.proc.setArguments(args)

        self.proc.startDetached()
        self.OpenCpnTimerFunc()

    def DashboardCloseButtonClicked(self):
        self.dashboard_close_button.setVisible(False)
        self.dashboard_frame.setVisible(True)
        self.weather.setParent(None)
        self.engine.setParent(None)
        self.gauge.setParent(None)
        self.map.setParent(None)

    def WeatherButtonClicked(self):
        self.dashboard_close_button.setVisible(True)
        self.dashboard_frame.setVisible(False)
        self.weather.setUrl(self.weatherqurl)
        self.dashboard_layout.addWidget(self.weather)

    def EngineButtonClicked(self):
        self.dashboard_close_button.setVisible(True)
        self.dashboard_frame.setVisible(False)
        self.engine.setUrl(self.enginequrl)
        self.dashboard_layout.addWidget(self.engine)

    def GaugeButtonClicked(self):
        self.dashboard_close_button.setVisible(True)
        self.dashboard_frame.setVisible(False)
        self.gauge.setUrl(self.gaugequrl)
        self.dashboard_layout.addWidget(self.gauge)

    def MapButtonClicked(self):
        self.dashboard_close_button.setVisible(True)
        self.dashboard_frame.setVisible(False)
        self.map.setUrl(self.mapqurl)
        self.dashboard_layout.addWidget(self.map)

    def Camera1ButtonClicked(self):
        self.camerabrowser.setZoomFactor(1)
        self.video_stream_close_button.setVisible(True)
        self.video_layout.addWidget(self.camerabrowser, 0, 0)
        # self.video_label1.setVisible(True)
        # self.video_stream_close_button.setVisible(True)
        # self.camera1Obj.start()

    def Camera2ButtonClicked(self):
        self.video_label2.setVisible(True)
        self.video_stream_close_button.setVisible(True)
        self.camera2Obj.start()

    def Camera3ButtonClicked(self):
        self.video_label3.setVisible(True)
        self.video_stream_close_button.setVisible(True)
        self.camera3Obj.start()

    def Camera4ButtonClicked(self):
        self.video_label4.setVisible(True)
        self.video_stream_close_button.setVisible(True)
        self.camera4Obj.start()

    def Camera5ButtonClicked(self):
        self.video_label5.setVisible(True)
        self.video_stream_close_button.setVisible(True)
        self.camera5Obj.start()

    def Camera6ButtonClicked(self):
        self.video_label6.setVisible(True)
        self.video_stream_close_button.setVisible(True)
        self.camera6Obj.start()

    @pyqtSlot(QImage)
    def setRadar(self, image):
        self.radar_label.resize(500, 490)
        self.radar_label.setPixmap(QPixmap.fromImage(image))

    @pyqtSlot(QImage)
    def setImage1(self, image):
        self.video_label1.setPixmap(QPixmap.fromImage(image))
        if not self.camera1identifier:
            self.video_layout.addWidget(self.video_label1, 0, 0)
        else:
            self.video_layout.addWidget(self.video_label1, 0, 0)

    @pyqtSlot(QImage)
    def setImage2(self, image):
        self.video_label2.setPixmap(QPixmap.fromImage(image))
        if not self.camera2identifier:
            self.video_layout.addWidget(self.video_label2, 0, 0)
        else:
            self.video_layout.addWidget(self.video_label2, 0, 1)

    @pyqtSlot(QImage)
    def setImage3(self, image):
        self.video_label3.setPixmap(QPixmap.fromImage(image))
        if not self.camera3identifier:
            self.video_layout.addWidget(self.video_label3, 0, 0)
        else:
            self.video_layout.addWidget(self.video_label3, 0, 2)

    @pyqtSlot(QImage)
    def setImage4(self, image):
        self.video_label4.setPixmap(QPixmap.fromImage(image))
        if not self.camera4identifier:
            self.video_layout.addWidget(self.video_label4, 0, 0)
        else:
            self.video_layout.addWidget(self.video_label4, 1, 0)

    @pyqtSlot(QImage)
    def setImage5(self, image):
        self.video_label5.setPixmap(QPixmap.fromImage(image))
        if not self.camera5identifier:
            self.video_layout.addWidget(self.video_label5, 0, 0)
        else:
            self.video_layout.addWidget(self.video_label5, 1, 1)

    @pyqtSlot(QImage)
    def setImage6(self, image):
        self.video_label6.setPixmap(QPixmap.fromImage(image))
        if not self.camera6identifier:
            self.video_layout.addWidget(self.video_label6, 0, 0)
        else:
            self.video_layout.addWidget(self.video_label6, 1, 2)

    def ViewAllButtonClicked(self):
        self.camera1Obj.start()
        self.camera2Obj.start()
        self.camera3Obj.start()
        self.camera4Obj.start()
        self.camera5Obj.start()
        self.camera6Obj.start()

        self.camera1identifier = True
        self.camera2identifier = True
        self.camera3identifier = True
        self.camera4identifier = True
        self.camera5identifier = True
        self.camera6identifier = True

        self.video_label1.setVisible(True)
        self.video_label2.setVisible(True)
        self.video_label3.setVisible(True)
        self.video_label4.setVisible(True)
        self.video_label5.setVisible(True)
        self.video_label6.setVisible(True)

        self.video_stream_close_button.setVisible(True)

    def VideoStreamCloseButtonClicked(self):
        if self.camera1Obj.isRunning():
            self.camera1Obj.StopFrame()
            # self.camera1Obj.terminate()
        # if self.camera2Obj.isRunning():
        #     self.camera2Obj.StopFrame()
        #     # self.camera2Obj.terminate()
        # if self.camera3Obj.isRunning():
        #     self.camera3Obj.StopFrame()
        #     # self.camera3Obj.terminate()
        # if self.camera4Obj.isRunning():
        #     self.camera4Obj.StopFrame()
        #     # self.camera4Obj.terminate()
        # if self.camera5Obj.isRunning():
        #     self.camera5Obj.StopFrame()
        #     # self.camera5Obj.terminate()
        # if self.camera6Obj.isRunning():
        #     self.camera6Obj.StopFrame()
        #     # self.camera6Obj.terminate()

        self.camera1identifier = False
        self.camera2identifier = False
        self.camera3identifier = False
        self.camera4identifier = False
        self.camera5identifier = False
        self.camera6identifier = False

        self.video_stream_close_button.setVisible(False)
        self.video_stream_frame.setVisible(True)

        self.video_label1.setVisible(False)
        self.video_label2.setVisible(False)
        self.video_label3.setVisible(False)
        self.video_label4.setVisible(False)
        self.video_label5.setVisible(False)
        self.video_label6.setVisible(False)

    def Process1ButtonClicked(self):
        self.process_label1.setVisible(True)
        self.process_close_button.setVisible(True)
        self.process1Obj.start()

    def Process2ButtonClicked(self):
        self.process_label2.setVisible(True)
        self.process_close_button.setVisible(True)
        self.process2Obj.start()

    def Process3ButtonClicked(self):
        self.process_label3.setVisible(True)
        self.process_close_button.setVisible(True)
        self.process3Obj.start()

    def Process4ButtonClicked(self):
        self.process_label4.setVisible(True)
        self.process_close_button.setVisible(True)
        self.process4Obj.start()

    def Process5ButtonClicked(self):
        self.process_label5.setVisible(True)
        self.process_close_button.setVisible(True)
        self.process5Obj.start()

    def Process6ButtonClicked(self):
        self.process_label6.setVisible(True)
        self.process_close_button.setVisible(True)
        self.process6Obj.start()

    @pyqtSlot(QImage)
    def setProcess1(self, image):
        self.process_label1.setPixmap(QPixmap.fromImage(image))
        if not self.process1identifier:
            self.process_layout.addWidget(self.process_label1, 0, 0)
        else:
            self.process_layout.addWidget(self.process_label1, 0, 0)

    @pyqtSlot(QImage)
    def setProcess2(self, image):
        self.process_label2.setPixmap(QPixmap.fromImage(image))
        if not self.process2identifier:
            self.process_layout.addWidget(self.process_label2, 0, 0)
        else:
            self.process_layout.addWidget(self.process_label2, 0, 1)

    @pyqtSlot(QImage)
    def setProcess3(self, image):
        self.process_label3.setPixmap(QPixmap.fromImage(image))
        if not self.process3identifier:
            self.process_layout.addWidget(self.process_label3, 0, 0)
        else:
            self.process_layout.addWidget(self.process_label3, 1, 0)

    @pyqtSlot(QImage)
    def setProcess4(self, image):
        self.process_label4.setPixmap(QPixmap.fromImage(image))
        if not self.process4identifier:
            self.process_layout.addWidget(self.process_label4, 0, 0)
        else:
            self.process_layout.addWidget(self.process_label4, 1, 1)

    @pyqtSlot(QImage)
    def setProcess5(self, image):
        self.process_label5.setPixmap(QPixmap.fromImage(image))
        if not self.process5identifier:
            self.process_layout.addWidget(self.process_label5, 0, 0)
        # else:
        #     self.process_layout.addWidget(self.process_label5, 1, 1)

    @pyqtSlot(QImage)
    def setProcess6(self, image):
        self.process_label6.setPixmap(QPixmap.fromImage(image))
        if not self.process6identifier:
            self.process_layout.addWidget(self.process_label6, 0, 0)
        # else:
        #     self.process_layout.addWidget(self.process_label6, 1, 2)

    def ProcessViewAllButtonClicked(self):
        self.process1Obj.start()
        self.process2Obj.start()
        self.process3Obj.start()
        self.process4Obj.start()
        self.process5Obj.start()
        self.process6Obj.start()

        self.process1identifier = True
        self.process2identifier = True
        self.process3identifier = True
        self.process4identifier = True
        self.process5identifier = True
        self.process6identifier = True

        self.process_label1.setVisible(True)
        self.process_label2.setVisible(True)
        self.process_label3.setVisible(True)
        self.process_label4.setVisible(True)
        self.process_label5.setVisible(True)
        self.process_label6.setVisible(True)

        self.process_close_button.setVisible(True)

    def ProcessCloseButtonClicked(self):
        if self.process1Obj.isRunning():
            self.process1Obj.StopFrame()

        self.process1identifier = False
        self.process2identifier = False
        self.process3identifier = False
        self.process4identifier = False
        self.process5identifier = False
        self.process6identifier = False

        self.process_close_button.setVisible(False)
        self.process_frame.setVisible(True)

        self.process_label1.setVisible(False)
        self.process_label2.setVisible(False)
        self.process_label3.setVisible(False)
        self.process_label4.setVisible(False)
        self.process_label5.setVisible(False)
        self.process_label6.setVisible(False)

    def CircleButtonClicked(self):
        print("CircleButtonClicked")
        self.circle_frame.setVisible(True)
        self.eight_frame.setVisible(False)
        self.linear_frame.setVisible(False)
        gGeometric.SendGeometricData(self.serverclicked, CIRCLE)
        self.eight_radius.setText("")
        self.linear_distance.setText("")
        self.circle_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; border-bottom: 4px solid #00007f; background-color : #aaffff;}")
        self.eight_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : white;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")
        self.linear_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : white;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

    def EightButtonClicked(self):
        print("EightButtonClicked")
        self.circle_frame.setVisible(False)
        self.eight_frame.setVisible(True)
        self.linear_frame.setVisible(False)
        gGeometric.SendGeometricData(self.serverclicked, EIGHT)
        self.circle_radius.setText("")
        self.linear_distance.setText("")
        self.eight_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; border-bottom: 4px solid #00007f; background-color : #aaffff;}")
        self.circle_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : white;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")
        self.linear_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : white;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

    def LinearButtonClicked(self):
        print("LinearButtonClicked")
        self.circle_frame.setVisible(False)
        self.eight_frame.setVisible(False)
        self.linear_frame.setVisible(True)
        gGeometric.SendGeometricData(self.serverclicked, LINEAR)
        self.eight_radius.setText("")
        self.circle_radius.setText("")
        self.linear_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; border-bottom: 4px solid #00007f; background-color : #aaffff;}")
        self.circle_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : white;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")
        self.eight_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : white;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

    def GeometricConfirmButtonClicked(self):
        print("GeometricConfirmButtonClicked")
        self.geometric_frame.setVisible(False)
        self.geometricconfirm = True
        self.boat_label_1.move(430, 310)
        self.eight_boat.move(430, 320)
        self.linear_boat_label.move(430, 310)
        if self.circle_radius.text():
            self.shape = "circle"
            self.radius_label.setText("Circle Radius:")
            self.circle_radius_display.setText(self.circle_radius.text())
            self.geometric_frame.setVisible(True)
            self.animation_frame.setVisible(True)
            gGeometric.SendGeometricData(self.serverclicked, int(self.circle_radius.text()))
            self.circle_radius.setText("")
            self.circle_frame.setVisible(False)
            self.circle_label.setVisible(True)
            self.boat_label_1.setVisible(True)
            self.eight_label.setVisible(False)
            self.linear_line.setVisible(False)
            self.eight_boat.setVisible(False)
            self.linear_boat_label.setVisible(False)
        if self.eight_radius.text():
            self.shape = "eight"
            self.radius_label.setText("Eight Radius:")
            self.circle_radius_display.setText(self.eight_radius.text())
            self.geometric_frame.setVisible(True)
            self.animation_frame.setVisible(True)
            gGeometric.SendGeometricData(self.serverclicked, int(self.eight_radius.text()))
            self.eight_radius.setText("")
            self.eight_frame.setVisible(False)
            self.boat_label_1.setVisible(False)
            self.eight_label.setVisible(True)
            self.circle_label.setVisible(False)
            self.linear_line.setVisible(False)
            self.eight_boat.setVisible(True)
            self.linear_boat_label.setVisible(False)
        if self.linear_distance.text():
            self.shape = "linear"
            self.radius_label.setText("Linear Distance:")
            self.circle_radius_display.setText(self.linear_distance.text())
            self.geometric_frame.setVisible(True)
            self.animation_frame.setVisible(True)
            gGeometric.SendGeometricData(self.serverclicked, int(self.linear_distance.text()))
            self.linear_distance.setText("")
            self.circle_label.setVisible(False)
            self.eight_label.setVisible(False)
            self.linear_line.setVisible(True)
            self.linear_frame.setVisible(False)
            self.eight_boat.setVisible(False)
            self.linear_boat_label.setVisible(True)

    def GeometricCancelButtonClicked(self):
        print("GeometricCancelButtonClicked")
        self.tabWidget.setTabText(0, "NAVIGATION")
        self.geometricconfirm = False
        self.geometric_frame.setVisible(False)
        self.circle_frame.setVisible(False)
        self.eight_frame.setVisible(False)
        self.linear_frame.setVisible(False)
        self.circle_radius.setText("")
        self.eight_radius.setText("")
        self.linear_distance.setText("")
        gRDClient.setDeviceMode(self.serverclicked, MODE_GEOMETRIC_OFF)

    def GeometricStopButtonClicked(self):
        print("GeometricStopButtonClicked")
        self.degreematching = 0
        if self.shape == "circle":
            rotated = self.boat.transformed(QTransform().rotate(0))
            self.boat_label_1.setPixmap(rotated)
            self.boat_label_1.move(430, 310)
        if self.shape == "eight":
            rotated = self.boat.transformed(QTransform().rotate(0))
            self.eight_boat.setPixmap(rotated)
            self.eight_boat.move(430, 320)
        if self.shape == "linear":
            self.linear_boat_label.move(430, 310)
        gRDClient.setDeviceMode(self.serverclicked, MODE_GEOMETRIC_OFF)
        self.geometric_frame.setVisible(False)
        self.animation_frame.setVisible(False)
        self.geometricconfirm = False
        self.boat_label_1.setVisible(False)
        self.eight_boat.setVisible(False)
        self.linear_boat_label.setVisible(False)
        self.circle_navigation_completed.setVisible(False)
        self.eight_navigation_completed.setVisible(False)
        self.linear_navigation_completed.setVisible(False)

    def closeEvent(self, event):
        print("Window Closing.....")
        if self.joystickThread.isRunning():
            print("Killing Joystick Thread")
            self.joystickThread.terminate()

        name = "AppRun.wrapped"
        try:
            # iterating through each instance of the process
            for line in os.popen("ps ax | grep " + name + " | grep -v grep"):
                fields = line.split()
                # extracting Process ID from the output
                pid = fields[0]
                # terminating process
                os.kill(int(pid), signal.SIGKILL)
            print("Process Successfully terminated")
        except:
            print("Error Encountered while running script")

    def TabChangeEvent(self):
        print("Tab Changed", self.tabWidget.currentIndex())
        if self.tabWidget.currentIndex() == 4:
            self.EmbedOpenCpn("opencpn")
        # if not self.startbuttonconfirm:
        #     if self.tabWidget.currentIndex() == 2:
        #         self.dashboard.setUrl(self.dashboard_qurl)
        #         self.dashboard_layout.addWidget(self.dashboard)

    def GeometricEvent(self):
        geometricdata = libCalc.geometric_data()
        print("Angle:", geometricdata[1])
        if self.geometricconfirm and geometricdata[1]:
            print("geometricconfirm")
            if self.shape == "circle":
                self.degreematching = geometricdata[1] / 30
                print("Degree Matching", self.degreematching)
                for i in range(1, 13):
                    if i == self.degreematching:
                        rotated = self.boat.transformed(QTransform().rotate(self.degreematching * 30),
                                                        Qt.SmoothTransformation)
                        self.boat_label_1.setPixmap(rotated)
                        self.anim.setEndValue(QRect(self.circlecoordinatelist[int(self.degreematching)][0],
                                                    self.circlecoordinatelist[int(self.degreematching)][1],
                                                    200, 200))
                        self.anim.start()
                    if self.degreematching == 12:
                        self.circle_navigation_completed.setVisible(True)
            if self.shape == "eight":
                self.degreematching = geometricdata[1] / 60
                print("Degree Matching", self.degreematching)
                for i in range(1, 13):
                    if i == self.degreematching:
                        rotated = self.boat.transformed(
                            QTransform().rotate(self.eightanglelist[int(self.degreematching) - 1]),
                            Qt.SmoothTransformation)
                        self.eight_boat.setPixmap(rotated)
                        self.animateeight.setEndValue(QRect(self.eightcoordinatelist[int(self.degreematching)][0],
                                                            self.eightcoordinatelist[int(self.degreematching)][1],
                                                            200, 200))
                        self.animateeight.start()
                    if self.degreematching == 12:
                        self.eight_navigation_completed.setVisible(True)
            if self.shape == "linear":
                print("Distance", int(geometricdata[1]))
                self.animatelinear.setEndValue(QRect(self.linearcoordinatelist[int(geometricdata[1]) - 1][0],
                                                     self.linearcoordinatelist[int(geometricdata[1]) - 1][1],
                                                     200, 200))
                self.animatelinear.start()
                if geometricdata[1] == 5:
                    self.linear_navigation_completed.setVisible(True)

    def GpsEvent(self):
        self.gpsdata = libCalc.gps_data()

        if self.gpsdata[3] == 1.0:
            self.serverindex = int(self.gpsdata[0])
            # print("Server Index : ", self.serverindex)
            print("Longitude :", self.gpsdata[1])
            print("Latitude :", self.gpsdata[2])
            print("Depth  :", self.gpsdata[4])
            print("Confidence :", self.gpsdata[5])
            if self.gpsdata[4] !=0.00:
                self.sonar_depth.setText(str(self.gpsdata[4]))
                self.sonar_confidence.setText(str(self.gpsdata[5]))
                self.sonar_latitude.setText(str(round(self.gpsdata[2],4)))
                self.sonar_longitude.setText(str(round(self.gpsdata[1],4)))
            self.indexlist.append(self.serverindex)
            self.uniquelist = list(set(self.indexlist))
            self.positionnumber = self.uniquelist.index(self.serverindex)
            # if self.currentgpsstatus:
            #     print("Entered")
            #     self.gpslist[self.positionnumber] = (round(self.gpsdata[1], 4), round(self.gpsdata[2], 4))
            #     print(self.gpslist)
            #     self.nonzerolist = list(filter(lambda a: a != (0, 0), self.gpslist))
            #     self.currentgpsstatus = False
            # print(self.uniquelist)
            print("GPS List: ", self.nonzerolist)

            pos = self.uniquelist.index(int(self.serverindex))
            gGpsdata[pos * 2] = round(self.gpsdata[1], 4)
            gGpsdata[(pos * 2) + 1] = round(self.gpsdata[2], 4)
            # print("GPS Array: ", gGpsdata)

            self.settings_layout.addWidget(self.tableWidget)

            font = QFont()
            font.setPointSize(8)
            ip = QTableWidgetItem("CONNECTION IP")
            port = QTableWidgetItem("CONNECTION PORT")
            status = QTableWidgetItem("STATUS")
            ip.setFont(font)
            ip.setTextAlignment(Qt.AlignCenter)
            ip.setBackground(QBrush(QColor('#00007f')))
            ip.setForeground(QBrush(QColor('white')))
            port.setFont(font)
            port.setTextAlignment(Qt.AlignCenter)
            port.setBackground(QBrush(QColor('#00007f')))
            port.setForeground(QBrush(QColor('white')))
            status.setFont(font)
            status.setTextAlignment(Qt.AlignCenter)
            status.setBackground(QBrush(QColor('#00007f')))
            status.setForeground(QBrush(QColor('white')))

            self.tableWidget.setItem(0, 0, ip)
            self.tableWidget.setItem(0, 1, port)
            self.tableWidget.setItem(0, 2, status)

            self.tableWidget.horizontalHeader().setStretchLastSection(True)
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

            if len(self.uniquelist) > self.previouslength and not self.geometricconfirm:
                self.tableWidget.insertRow(len(self.uniquelist))
                ipvalue = QTableWidgetItem(self.connection_ip.text())
                portvalue = QTableWidgetItem(self.connection_port.text())
                statusvalue = QTableWidgetItem("CONNECTED")
                ipvalue.setFont(font)
                # qurl = QUrl("http://192.168.1.76:9998/index.html")
                # self.camerabrowser.setUrl(qurl)
                # self.camerabrowser1.setUrl(qurl)
                # self.camerabrowser2.setUrl(qurl)
                # self.camerabrowser3.setUrl(qurl)
                # self.video_layout.addWidget(self.camerabrowser, 0, 0)
                # self.video_layout.addWidget(self.camerabrowser1, 0, 1)
                # self.video_layout.addWidget(self.camerabrowser2, 1, 0)
                # self.video_layout.addWidget(self.camerabrowser3, 1, 1)
                # self.camerabrowser.setZoomFactor(0.5)
                # self.camerabrowser1.setZoomFactor(0.5)
                # self.camerabrowser2.setZoomFactor(0.5)
                # self.camerabrowser3.setZoomFactor(0.5)
                ipvalue.setTextAlignment(Qt.AlignCenter)
                ipvalue.setBackground(QBrush(QColor('#aaffff')))
                ipvalue.setForeground(QBrush(QColor('#00007f')))
                portvalue.setFont(font)
                portvalue.setTextAlignment(Qt.AlignCenter)
                portvalue.setBackground(QBrush(QColor('#aaffff')))
                portvalue.setForeground(QBrush(QColor('#00007f')))
                statusvalue.setFont(font)
                statusvalue.setTextAlignment(Qt.AlignCenter)
                statusvalue.setBackground(QBrush(QColor('#aaffff')))
                statusvalue.setForeground(QBrush(QColor('#00007f')))

                self.tableWidget.setItem(pos + 1, 0, ipvalue)
                self.tableWidget.setItem(pos + 1, 1, portvalue)
                self.tableWidget.setItem(pos + 1, 2, statusvalue)

                self.refreshcount = self.refreshcount + 1
                self.currentgpsstatus = True

            if self.startbuttonconfirm:
                try:
                    if not self.mainscreenstatus:
                        self.refreshindex = self.ANCuniquelist.index(self.positionnumber)
                        self.gpslist[self.positionnumber] = (round(self.gpsdata[1], 4), round(self.gpsdata[2], 4))
                        self.nonzerolist = list(filter(lambda a: a != (0, 0), self.gpslist))
                        self.AutonomousUpdate()
                except ValueError:
                    print("Sorry no Index Matched")

            if self.currentgpsstatus and self.gpsdata[1] and self.gpsdata[2]:
                self.previouslength = len(self.uniquelist)
                self.CurrentGpsPage()
                self.currentgpsstatus = False
                self.nogpssignal = True
            if self.nogpssignal and not self.gpsdata[1] and not self.gpsdata[2]:
                self.previouslength = len(self.uniquelist)
                self.currentgpsstatus = True
                self.satelliteviewstatus = True
                self.nogpssignal = False
                self.browser.setUrl(self.qurl)

            if not self.mode_selector.isVisible():
                for i in range(0, 10):
                    if self.modelist[i] == (1, 0, 0) or self.modelist[i] == (0, 1, 0) or self.modelist[i] == (0, 0, 1):
                        if i == self.positionnumber and self.mainscreenstatus:
                            if self.gpsdata[1] and self.gpsdata[2]:
                                self.navigation_button.setVisible(False)
                                self.start_button.setVisible(False)
                                self.start_all_button.setVisible(False)
                                self.stop_button.setVisible(False)
                                if True in self.autonomousrunningstatus:
                                    self.stop_all_button.setVisible(True)
                                else:
                                    self.stop_all_button.setVisible(False)
                                self.refreshcount = self.refreshcount + 1
                                print("ServerRefresh: ", self.uniquelist[i])
                                self.gpslist[i] = (round(self.gpsdata[1], 4), round(self.gpsdata[2], 4))
                                self.nonzerolist = list(filter(lambda a: a != (0, 0), self.gpslist))
                                self.CurrentGpsPage()

    def ConnectButtonClicked(self):
        if not self.connection_ip.text() or not self.connection_port.text():
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Information)
            msgBox.setText("ENTER IP ADDRESS AND PORT")
            msgBox.setWindowTitle("ALERT")
            msgBox.setStyleSheet(
                "QMessageBox{ background-color : #aaffff;} "
                "QLabel{ color : #00007f;}"
                "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                "QPushButton::pressed{ background-color : #00007f; color : white;}"
                "QPushButton{ background-color : #00007f;}"
                "QPushButton{ color : white;}")
            msgBox.exec()

        if self.connection_ip.text() and self.connection_port.text():
            print("Connecting......")
            ipaddress = self.connection_ip.text()
            portaddress = int(self.connection_port.text())
            print(ipaddress)
            print(portaddress)
            superClientInstance = int(2)
            print("superClientInstance: ", superClientInstance)
            self.index = libCalc.gSuperClientAddServer(superClientInstance,
                                                       ctypes.c_char_p(bytes(ipaddress.encode('utf-8'))),
                                                       portaddress, libCalc.gProcess)

            print("Index: ", self.index)
            #superClientInstance = self.index + 1
            gRDClient.identifyClient(self.index)

    def CurrentGpsPage(self):
        self.markerclicktimer.start(100)
        self.satelliteviewstatus = False
        features = []
        self.gpslist[self.positionnumber] = (round(self.gpsdata[1], 4), round(self.gpsdata[2], 4))
        print(self.gpslist)
        self.nonzerolist = list(filter(lambda a: a != (0, 0), self.gpslist))
        for x in self.uniquelist:
            x = self.uniquelist.index(x)
            point = Point((float(gGpsdata[x * 2]), float(gGpsdata[(x * 2) + 1])))
            features.append(
                Feature(geometry=point,
                        properties={'marker-size': 'large', 'marker-color': self.markercolor[x],
                                    'marker-symbol': (x + 1)}))
        feature_collection = FeatureCollection(features)
        with open('map.geojson', 'w') as f:
            dump(feature_collection, f)
        with open('map.geojson') as f:
            currentgpscontents = f.read()
        currentgpsqurl = QUrl(make_url(currentgpscontents))
        if self.refreshcount % 2 == 0:
            print("Even Refresh: ", self.refreshcount)
            currentgpsqurl.setScheme("https")
            self.browser.setUrl(currentgpsqurl)
        else:
            print("Odd Refresh: ", self.refreshcount)
            self.browser.setUrl(currentgpsqurl)

    def NavigationButtonClicked(self):
        print("NavigationButtonClicked")
        self.autonomous_window_close_button.setVisible(False)
        self.navigation_button.setVisible(False)
        self.start_button.setVisible(False)
        self.stop_button.setVisible(False)
        self.start_all_button.setVisible(False)
        self.stop_all_button.setVisible(False)
        self.markerclicktimer.stop()
        self.routing_window_button_frame.setVisible(True)
        self.exit_button.setVisible(True)

    def StartButtonClicked(self):
        print("StartButtonClicked")
        self.satelliteviewstatus = False
        if self.autonomousfilenamesave[self.serverclicked]:
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Question)
            msgBox.setText("BOAT IS GOING TO START WITH THE ROUTE FILE " + self.autonomousfilenamesave[
                self.serverclicked] + ". ARE YOU SURE?")
            msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            msgBox.setWindowTitle("ALERT")
            msgBox.setStyleSheet(
                "QMessageBox{ background-color : #aaffff;} "
                "QLabel{ color : #00007f;}"
                "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                "QPushButton::pressed{ background-color : #00007f; color : white;}"
                "QPushButton{ background-color : #00007f;}"
                "QPushButton{ color : white;}")
            ret = msgBox.exec()
            if ret == QMessageBox.Yes:
                print("Boat Start Confirmed")
                self.autonomousrunningstatus[self.serverclicked] = True
                # self.image_process_file_thread.start()
                #self.imageProcessThread.start()
                self.markerclicktimer.stop()
                gAutonomous.sendCommandType1(self.serverclicked, AUTONOMOUS_ON_STATE)
                self.startbuttonconfirm = True
                self.navigation_button.setVisible(False)
                self.start_button.setVisible(False)
                self.stop_button.setVisible(True)
                self.start_all_button.setVisible(False)
                self.stop_all_button.setVisible(False)
            if ret == QMessageBox.No:
                print("Boat Start Cancelled")

    def StartAllButtonClicked(self):
        print("StartAllButtonClicked")
        self.satelliteviewstatus = False

        if self.filestatus:
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Question)
            msgBox.setText(
                "ALL BOATS ARE GOING TO START WITH THE ROUTE FILE " + self.savefilename + ". ARE YOU SURE?")
            msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            msgBox.setWindowTitle("ALERT")
            msgBox.setStyleSheet(
                "QMessageBox{ background-color : #aaffff;} "
                "QLabel{ color : #00007f;}"
                "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                "QPushButton::pressed{ background-color : #00007f; color : white;}"
                "QPushButton{ background-color : #00007f;}"
                "QPushButton{ color : white;}")
            ret = msgBox.exec()

            if ret == QMessageBox.Yes:
                print("Boat Start Confirmed")
                for i in range(0, len(self.ANCuniquelist)):
                    gAutonomous.sendCommandType1(self.uniquelist[i], 1)
                self.startbuttonconfirm = True
                self.navigation_button.setVisible(False)
                self.start_button.setVisible(False)
                self.stop_button.setVisible(True)
                self.start_all_button.setVisible(False)
                self.stop_all_button.setVisible(False)

    def StopButtonClicked(self):
        print("StopButtonClicked")
        gAutonomous.sendCommandType1(self.serverclicked, AUTONOMOUS_OFF_STATE)
        print("serverclicked:", self.serverclicked)
        self.autonomousrunningstatus[self.serverclicked] = False
        self.navigation_button.setVisible(True)
        self.start_button.setVisible(True)
        self.markerclicktimer.start(100)
        self.stop_button.setVisible(False)
        self.start_all_button.setVisible(False)
        self.stop_all_button.setVisible(False)

        self.loadfinishedstatus = False
        self.startbuttonconfirm = False

    def StopAllButtonClicked(self):
        print("StopAllButtonClicked")
        print("ANC Uniquelist: ", self.uniquelist)
        for i in range(0, len(self.uniquelist)):
            gAutonomous.sendCommandType1(self.uniquelist[i], AUTONOMOUS_OFF_STATE)
            self.autonomousrunningstatus[self.uniquelist[i]] = False
        # self.navigation_button.setVisible(True)
        # self.start_button.setVisible(True)
        self.stop_button.setVisible(False)
        self.start_all_button.setVisible(False)
        self.stop_all_button.setVisible(False)

        self.loadfinishedstatus = False
        self.startbuttonconfirm = False

    def SaveButtonClicked(self):
        # click html save button
        self.browser.page().runJavaScript(
            "document.getElementsByClassName('children')[1].getElementsByTagName('a')[2].click().innerHTML")

    def LoadButtonClicked(self):
        print("Load Map")
        self.OpenDialogBox()

    def RouteButtonClicked(self):
        print("RouteButtonClicked")
        # click html route button
        self.browser.page().runJavaScript(
            "document.getElementsByClassName('leaflet-draw-draw-polyline')[0].click().innerHTML")

    def ExitButtonClicked(self):
        print("ExitButtonClicked")
        self.mode_selector_close_button.click()
        if not self.autonomousfilenamesave[self.serverclicked]:
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Question)
            msgBox.setText("NO ROUTE FILE SELECTED. EXIT AUTONOMOUS MODE?")
            msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            msgBox.setWindowTitle("ALERT")
            msgBox.setStyleSheet(
                "QMessageBox{ background-color : #aaffff;} "
                "QLabel{ color : #00007f;}"
                "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                "QPushButton::pressed{ background-color : #00007f; color : white;}"
                "QPushButton{ background-color : #00007f;}"
                "QPushButton{ color : white;}")
            ret = msgBox.exec()
            if ret == QMessageBox.Yes:
                self.mainscreenstatus = True
                self.tabWidget.setTabText(0, "NAVIGATION")
                self.autonomousmode = False
                self.markerpopup = False
                self.nomarkerpopup = False

                # check if the marker popup exist
                self.browser.page().runJavaScript(
                    "document.getElementsByClassName('leaflet-popup-content-wrapper')[0].innerHTML",
                    self.ExitButtonCallbackFunc)

                self.routing_window_button_frame.setVisible(False)
                self.exit_button.setVisible(False)

        else:

            self.mainscreenstatus = False
            self.waypointcount = 0
            self.GetWaypoints()
            self.autonomous_window_close_button.setVisible(True)
            self.navigation_button.setVisible(True)
            self.start_button.setVisible(True)
            self.stop_button.setVisible(False)
            self.start_all_button.setVisible(False)
            self.stop_all_button.setVisible(False)
            # self.markerclicktimer.start(100)
            self.routing_window_button_frame.setVisible(False)
            self.exit_button.setVisible(False)

    def ExitButtonCallbackFunc(self, html):
        if html and not self.nomarkerpopup:  # marker popup exists. Do the following stuffs
            print("Marker Exist")
            self.markercolor[self.servernoindex] = '#8a8a8a'

            # hide any other marker popups
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('leaflet-marker-icon leaflet-zoom-animated leaflet-clickable')["
                "0].click().innerHTML")

            # open current marker popup
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('leaflet-marker-icon leaflet-zoom-animated leaflet-clickable')"
                "[" + str(self.servernoindex) + "].click().innerHTML")

            # change the color to Grey
            self.browser.page().runJavaScript("document.getElementsByTagName('input')[4].value = '#8a8a8a'")

            # save the changes
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('save col6 major')[0].click().innerHTML")
            self.modelist[self.servernoindex] = (0, 0, 0)
            self.nomarkerpopup = True
            self.markerclicktimer.start(100)
        if not html and not self.markerpopup:  # marker popup doesn't exist. do the following stuffs
            print("No Marker Exist")
            self.markercolor[self.servernoindex] = '#8a8a8a'

            # open current marker popup
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('leaflet-marker-icon leaflet-zoom-animated leaflet-clickable')"
                "[" + str(self.servernoindex) + "].click().innerHTML")

            # change the color to Grey
            self.browser.page().runJavaScript("document.getElementsByTagName('input')[4].value = '#8a8a8a'")

            # save the changes
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('save col6 major')[0].click().innerHTML")
            self.modelist[self.servernoindex] = (0, 0, 0)
            self.markerpopup = True
            self.markerclicktimer.start(100)

    def ZoomInButtonClicked(self):
        print("ZoomInButtonClicked")

        # click html zoom in
        self.browser.page().runJavaScript(
            "document.getElementsByClassName('leaflet-control-zoom-in')[0].click().innerHTML")

    def ZoomOutButtonClicked(self):
        print("ZoomOutButtonClicked")

        # click html zoom out
        self.browser.page().runJavaScript(
            "document.getElementsByClassName('leaflet-control-zoom-out')[0].click().innerHTML")

    def BoatJoystickButtonClicked(self):
        print("BoatJoystickButtonClicked")
        msgBox = QMessageBox()
        msgBox.setIcon(QMessageBox.Question)
        msgBox.setText("ENABLE BOAT JOYSTICK MODE?")
        msgBox.setStandardButtons(QMessageBox.Save | QMessageBox.Cancel)
        msgBox.setWindowTitle("ALERT")
        msgBox.setStyleSheet(
            "QMessageBox{ background-color : #aaffff;} "
            "QLabel{ color : #00007f;}"
            "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
            "QPushButton::pressed{ background-color : #00007f; color : white;}"
            "QPushButton{ background-color : #00007f;}"
            "QPushButton{ color : white;}")
        ret = msgBox.exec()
        if ret == QMessageBox.Save:
            self.autonomousfilenamesave[self.serverclicked] = ""
            self.tabWidget.setTabText(0, "NAVIGATION")
            self.autonomouscount = 0
            print("Boat Joystick Enabled")
            self.mode_selector.setVisible(False)
            self.popupstatus = True
            self.modelist[self.servernoindex] = (1, 0, 0)
            self.markercolor[self.servernoindex] = "#00aa00"

            # change color to green
            self.browser.page().runJavaScript("document.getElementsByTagName('input')[4].value = '#00aa00'")

            # save the changes
            self.browser.page().runJavaScript("document.getElementsByClassName('save col6 major')[0].click().innerHTML")

            for i in range(0, 10):
                if self.modelist[i] == (0, 0, 1):
                    self.autonomouscount = self.autonomouscount + 1
            if self.autonomouscount == 0:
                self.autonomousmode = False

            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Information)
            msgBox.setText("NOW YOU CAN CONTROL THE BOAT USING BOAT JOYSTICK")
            msgBox.setWindowTitle("ALERT")
            msgBox.setStyleSheet(
                "QMessageBox{ background-color : #aaffff;} "
                "QLabel{ color : #00007f;}"
                "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                "QPushButton::pressed{ background-color : #00007f; color : white;}"
                "QPushButton{ background-color : #00007f;}"
                "QPushButton{ color : white;}")
            msgBox.exec()
            gRDClient.setDeviceMode(self.serverclicked, MODE_BOAT_JOYSTICK)
            if self.joystickThread.isRunning():
                print("Killing Joystick Thread")
                self.joystickThread.terminate()

    def BaseStationJoystickButtonClicked(self):
        print("BaseStationJoystickButtonClicked")
        msgBox = QMessageBox()
        msgBox.setIcon(QMessageBox.Question)
        msgBox.setText("ENABLE BASE STATION JOYSTICK MODE?")
        msgBox.setStandardButtons(QMessageBox.Save | QMessageBox.Cancel)
        msgBox.setWindowTitle("ALERT")
        msgBox.setStyleSheet(
            "QMessageBox{ background-color : #aaffff;} "
            "QLabel{ color : #00007f;}"
            "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
            "QPushButton::pressed{ background-color : #00007f; color : white;}"
            "QPushButton{ background-color : #00007f;}"
            "QPushButton{ color : white;}")
        ret = msgBox.exec()
        if ret == QMessageBox.Save:
            self.autonomousfilenamesave[self.serverclicked] = ""
            self.tabWidget.setTabText(0, "NAVIGATION")
            self.autonomouscount = 0
            self.mode_selector.setVisible(False)
            self.popupstatus = True
            self.modelist[self.servernoindex] = (0, 1, 0)
            print("Base Station Joystick Enabled")
            if self.joystickThread.isRunning():
                print("Killing Joystick Thread")
                self.joystickThread.terminate()
            self.markercolor[self.servernoindex] = "#ffff00"

            # change color to yellow
            self.browser.page().runJavaScript("document.getElementsByTagName('input')[4].value = '#ffff00'")

            # save the changes
            self.browser.page().runJavaScript("document.getElementsByClassName('save col6 major')[0].click().innerHTML")
            for i in range(0, 10):
                if self.modelist[i] == (0, 0, 1):
                    self.autonomouscount = self.autonomouscount + 1
            if self.autonomouscount == 0:
                self.autonomousmode = False
            for i in range(0, 10):
                if self.modelist[i] == (0, 1, 0):
                    if i != self.servernoindex:
                        print("Repeat")
                        self.modelist[i] = (0, 0, 0)
                        self.markercolor[i] = "#8a8a8a"

                        # open any other popup with yellow
                        self.browser.page().runJavaScript(
                            "document.getElementsByClassName('leaflet-marker-icon leaflet-zoom-animated "
                            "leaflet-clickable')[" + str(i) + "].click().innerHTML")

                        # change color to Grey
                        self.browser.page().runJavaScript("document.getElementsByTagName('input')[4].value = '#8a8a8a'")

                        # save the changes
                        self.browser.page().runJavaScript(
                            "document.getElementsByClassName('save col6 major')[0].click().innerHTML")

            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Information)
            msgBox.setText("NOW YOU CAN CONTROL THE BOAT USING BASE STATION JOYSTICK")
            msgBox.setWindowTitle("ALERT")
            msgBox.setStyleSheet(
                "QMessageBox{ background-color : #aaffff;} "
                "QLabel{ color : #00007f;}"
                "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                "QPushButton::pressed{ background-color : #00007f; color : white;}"
                "QPushButton{ background-color : #00007f;}"
                "QPushButton{ color : white;}")
            msgBox.exec()
            gRDClient.setDeviceMode(self.serverclicked, MODE_BS_JOYSTICK)
            if not self.joystickThread.isRunning():
                self.joystickThread.start()

    def AutonomousButtonClicked(self):
        print("AutonomousButtonClicked")
        self.mainscreenstatus = False
        if not self.autonomousrunningstatus[self.serverclicked]:

            self.markercolor[self.servernoindex] = "#ffaa00"
            self.tabWidget.setTabText(0, "WAYPOINT ROUTING")

            # change the color to Orange
            self.browser.page().runJavaScript("document.getElementsByTagName('input')[4].value = '#ffaa00'")

            # save the changes
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('save col6 major')[0].click().innerHTML")
            self.markerclicktimer.stop()
            self.autonomousmode = True
            self.mode_selector.setVisible(False)
            self.popupstatus = True
            self.filestatus = False
            self.modelist[self.servernoindex] = (0, 0, 1)
            print("Autonomous ModeEnabled")

            self.navigation_button.setVisible(False)
            self.start_button.setVisible(False)
            self.stop_button.setVisible(False)
            self.start_all_button.setVisible(False)
            self.stop_all_button.setVisible(False)

            self.routing_window_button_frame.setVisible(True)
            self.exit_button.setVisible(True)

            gRDClient.setDeviceMode(self.serverclicked, MODE_AUTONOMOUS)
            if self.joystickThread.isRunning():
                print("Killing Joystick Thread")
                self.joystickThread.terminate()
        else:

            self.startbuttonconfirm = True
            self.markerclicktimer.stop()
            self.autonomousmode = True
            self.mode_selector.setVisible(False)
            self.popupstatus = True
            self.filestatus = False
            self.stop_button.setVisible(True)
            self.stop_all_button.setVisible(False)
            self.autonomous_window_close_button.setVisible(True)

    def AutonomousWindowCloseButtonClicked(self):
        print("AutonomousWindowCloseButtonClicked")
        self.markerclicktimer.start(100)
        self.mainscreenstatus = True
        self.startbuttonconfirm = False
        self.autonomous_window_close_button.setVisible(False)
        self.navigation_button.setVisible(False)
        self.start_button.setVisible(False)
        self.stop_button.setVisible(False)
        self.stop_all_button.setVisible(True)
        self.CurrentGpsPage()

    def GeometricButtonClicked(self):
        self.mode_selector_close_button.click()
        self.geometric_frame.setVisible(True)
        gRDClient.setDeviceMode(self.serverclicked, MODE_GEOMETRIC)
        # gAutonomous.sendCommandType1(self.serverclicked, MODE_GEOMETRIC)

    def AutonomousUpdate(self):
        # print("Current Server Index:",self.positionnumber)
        print("Autonomous Index:", self.refreshindex)

        with open(self.autonomousfilenamesave[self.serverclicked]) as f:
            savecontents = f.read()
            savefiledata = loads(savecontents)
            count = 0
            for feature in savefiledata['features']:
                types = feature['properties']
                if types:
                    if types['marker-color'] == '#ffaa00':
                        count = count + 1
                        if (count - 1) == self.refreshindex:
                            self.autonomousrefreshcount = self.autonomousrefreshcount + 1
                            feature['geometry']['coordinates'] = ""
                            feature['geometry']['coordinates'] = [float(self.gpsdata[1]), float(self.gpsdata[2])]
                            print(count - 1, feature['geometry']['coordinates'])
                            # if count == len(self.ANCuniquelist):
                            #     count = 0

        with open(self.autonomousfilenamesave[self.serverclicked], 'w+') as f:
            dump(savefiledata, f)

        with open(self.autonomousfilenamesave[self.serverclicked]) as f:
            savecontents = f.read()
            qurl = QUrl(make_url(savecontents))
            if self.autonomousrefreshcount % 2 == 0:
                print("Even: ", self.autonomousrefreshcount)
                qurl.setScheme("https")
                self.browser.setUrl(qurl)
            else:
                print("Odd: ", self.autonomousrefreshcount)
                self.browser.setUrl(qurl)

    def ModeSelectorCloseButtonClicked(self):
        self.popupstatus = True
        print("ModeSelectorCloseButtonClicked")
        self.mode_selector.setVisible(False)

        # close the html popup
        self.browser.page().runJavaScript("document.getElementsByClassName('minor col6 cancel')[0].click().innerHTML")

    def GeneralButtonClicked(self):
        self.settings_sub_tab.setCurrentIndex(0)

        self.general_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : white;}")

        self.log_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

        self.survey_settings_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

    def LogButtonClicked(self):
        self.settings_sub_tab.setCurrentIndex(2)

        self.general_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

        self.log_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : white;}")

        self.survey_settings_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

    def SurveySettingsButtonClicked(self):
        self.settings_sub_tab.setCurrentIndex(4)

        self.survey_settings_button.setStyleSheet(
            "QPushButton{ color: #00007f; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : white;}")

        self.log_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

        self.general_button.setStyleSheet(
            "QPushButton::hover{color: #00007f; background-color : #aaffff;}"
            "QPushButton::pressed{ color: white; background-color : #00007f;}"
            "QPushButton{ color: white; border-radius : 10;}"
            "QPushButton{ border: 2px solid #00007f; background-color : #00007f;}")

    def GetWaypoints(self):
        self.ANCuniquelist.clear()
        for i in range(0, 10):
            if self.modelist[i][2] == 1:
                self.ANCuniquelist.append(i)
        print("ANCuniquelist:", self.ANCuniquelist)
        self.ANCuniquelist = list(set(self.ANCuniquelist))
        self.autonomousindex = self.ANCuniquelist.index(self.servernoindex)
        print("servernoindex:", self.servernoindex)
        print("autonomousindex:", self.autonomousindex)
        with open(self.autonomousfilenamesave[self.serverclicked]) as f:
            savecontents = f.read()
            savefiledata = loads(savecontents)
            for feature in savefiledata['features']:
                featuretype = feature['geometry']['type']
                if featuretype == "LineString":
                    self.latitudelist.clear()
                    self.longitudelist.clear()
                    print("Here1")
                    # self.waypointcount = self.waypointcount + 1
                    print("waypointcount:", self.waypointcount - 1)
                    # if self.autonomousindex:
                    numberofway_points = len(feature['geometry']['coordinates'])
                    print("Here2")
                    # print(numberofway_points)
                    for i in range(0, numberofway_points):
                        for j in range(0, 2):
                            k = (i * 2) + j
                            if j == 0:
                                self.longitudelist.append(feature['geometry']['coordinates'][i][j])
                                gWaypointdata[k] = feature['geometry']['coordinates'][i][j]
                            elif j == 1:
                                gWaypointdata[k] = feature['geometry']['coordinates'][i][j]
                                self.latitudelist.append(feature['geometry']['coordinates'][i][j])
                    # gRDClient.setDeviceMode(self.serverclicked, MODE_AUTONOMOUS)
                    # time.sleep(1)
                    gAutonomous.sendCommandType2(self.serverclicked, gWaypointdata)
                    print("WayPointList", gWaypointdata)
                    memset(gWaypointdata, 0, len(gWaypointdata))
                    print("Longitude: ", self.longitudelist)
                    print("\n")
                    print("Latitude: ", self.latitudelist)
                    print("\n")

    def MarkerClickTimerFunc(self):
        self.browser.page().runJavaScript(

            # check if the marker popup exist
            "document.getElementsByClassName('leaflet-popup-content-wrapper')[0].innerHTML",
            self.MarkerClickTimerCallback)

    def MarkerClickTimerCallback(self, html):
        if html:  # popup exists. do following stuffs
            # print("marker button pressed")
            self.markerlist = html
            self.popupstatus = False
            latitudeindex = self.markerlist.index("Latitude")
            longitudeindex = self.markerlist.index("Longitude")
            latitudevalue = self.markerlist[latitudeindex:longitudeindex]
            longitudevalue = self.markerlist[longitudeindex:]
            latitudesearch = re.split('; |, |>|<', latitudevalue)
            longitudesearch = re.split('; |, |>|<', longitudevalue)
            self.coordinatechecklist = (float(longitudesearch[4]), float(latitudesearch[4]))
            self.servernoindex = self.nonzerolist.index(self.coordinatechecklist)
            self.serverupdated = self.servernoindex + 1
            self.serverclicked = self.uniquelist[self.servernoindex]
            # print(self.coordinatechecklist)
            # print("Server Clicked No: ", self.serverclicked)
            # self.boat_number.setText("BOAT " + str(self.servernoindex + 1) + "")

            # Hide the html popup
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('leaflet-popup-pane')[0].style.visibility='hidden'")
            self.mode_selector.setVisible(True)
        elif not html:  # popup doesn't exist. do following stuffs '
            # print("marker button released")
            if self.serverupdated and not self.popupstatus:
                # print("HTML corrected")

                # open the marker popup
                self.browser.page().runJavaScript(
                    "document.getElementsByClassName('leaflet-marker-icon leaflet-zoom-animated leaflet-clickable')"
                    "[" + str(self.serverupdated - 1) + "].click().innerHTML")
                self.serverupdated = 0

    def OnLoadFinished(self, ok):
        if ok:
            print("Load Finished")
            self.loadfinishedstatus = True
            if self.satelliteviewstatus:

                # select satellite view
                self.browser.page().runJavaScript("document.getElementsByClassName('pad0x')[1].click().innerHTML")
            else:

                # select mapbox view
                self.browser.page().runJavaScript("document.getElementsByClassName('pad0x')[0].click().innerHTML")

            # set full screen view and remove unwanted items from the web page
            self.browser.page().runJavaScript("document.getElementsByClassName('collapse-button')[0].click().innerHTML")
            self.browser.page().runJavaScript("document.getElementsByClassName('layer-switch')[0].remove().innerHTML")
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('leaflet-top leaflet-right')[0].style.visibility='hidden'")
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('file-bar')[0].style.visibility='hidden'")
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('right')[0].remove().innerHTML")
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('leaflet-control-scale leaflet-control')[0].remove().innerHTML")
            self.browser.page().runJavaScript(
                "document.getElementsByClassName('leaflet-popup-pane')[0].style.visibility='hidden'")

    def DownloadRequested(self, download):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        savefilepath, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save File', '',
                                                                "GeoJson Files (*.geojson);;All "
                                                                "Files (*)")
        if savefilepath:
            print("Got Save File Path")
            self.updatedsavefilepath = savefilepath
            self.savefilename = self.updatedsavefilepath.split('/')[
                len(self.updatedsavefilepath.split('/')) - 1]
            download.setPath(self.updatedsavefilepath)
            download.accept()
            self.autonomousfilenamesave[self.serverclicked] = str(self.updatedsavefilepath)
            self.filestatus = True
        else:
            print("No Save File Path")

    def OpenDialogBox(self):
        filename = QFileDialog.getOpenFileName()
        loadfilepath = filename[0]
        if loadfilepath:
            self.updatedsavefilepath = loadfilepath
            initialloadfilename = self.updatedsavefilepath.split('/')[
                len(self.updatedsavefilepath.split('/')) - 1]
            loadfileextention = initialloadfilename.split('.')[
                len(initialloadfilename.split('.')) - 1]
            print(loadfileextention)
            print("Got Load File Path")
            if loadfileextention == "geojson":
                print(initialloadfilename)
                self.autonomousfilenamesave[self.serverclicked] = str(self.updatedsavefilepath)
                self.savefilename = initialloadfilename
                self.filestatus = True
            else:
                msgBox = QMessageBox()
                msgBox.setIcon(QMessageBox.Information)
                msgBox.setText("CANNOT OPEN " + loadfileextention + " FILE. PLEASE SELECT 'geojson' FORMAT")
                msgBox.setWindowTitle("ALERT")
                msgBox.setStyleSheet(
                    "QMessageBox{ background-color : #aaffff;} "
                    "QLabel{ color : #00007f;}"
                    "QPushButton::hover{ background-color : #aaffff; color : #00007f;}"
                    "QPushButton::pressed{ background-color : #00007f; color : white;}"
                    "QPushButton{ background-color : #00007f;}"
                    "QPushButton{ color : white;}")
                msgBox.exec()
        else:
            print("No Load File Path")

    def ConnectionIpTextChanged(self):
        if self.connection_ip.text():
            self.connection_ip.setStyleSheet(
                "QLineEdit{ color: #00007f; background-color: #aaffff; border-radius: 12;}")
        else:
            self.connection_ip.setStyleSheet(
                "QLineEdit{ color: #909090; background-color: #aaffff; border-radius: 12;}")

    def ConnectionPortTextChanged(self):
        if self.connection_port.text():
            self.connection_port.setStyleSheet(
                "QLineEdit{ color: #00007f; background-color: #aaffff; border-radius: 12;}")
        else:
            self.connection_port.setStyleSheet(
                "QLineEdit{ color: #909090; background-color: #aaffff; border-radius: 12;}")


app = QtWidgets.QApplication(sys.argv)
app.setWindowIcon(PyQt5.QtGui.QIcon('./Resources/Icons/AIMARINE.png'))
window = Ui()
app.exec_()
