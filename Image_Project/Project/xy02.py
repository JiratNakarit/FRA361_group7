# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'xy01.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
import motivation


class Ui_MainWindow(object):
    def __init__(self):
        self.method = 0

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(451, 480)
        MainWindow.setAutoFillBackground(False)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(140, 10, 291, 411))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.printDat = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.printDat.setObjectName("printDat")
        self.verticalLayout.addWidget(self.printDat)
        self.gridLayout.addLayout(self.verticalLayout, 5, 0, 1, 1)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.trackCon = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.trackCon.setObjectName("trackCon")
        self.horizontalLayout_2.addWidget(self.trackCon)
        self.trackApp = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.trackApp.setObjectName("trackApp")
        self.horizontalLayout_2.addWidget(self.trackApp)
        self.Capture = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.Capture.setObjectName("Capture")
        self.horizontalLayout_2.addWidget(self.Capture)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.gridLayout.addLayout(self.verticalLayout_3, 4, 0, 1, 1)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.openCam = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.openCam.setObjectName("openCam")
        self.horizontalLayout.addWidget(self.openCam)
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        self.gridLayout.addLayout(self.verticalLayout_4, 2, 0, 1, 1)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.cropImage = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.cropImage.setObjectName("cropImage")
        self.verticalLayout_5.addWidget(self.cropImage)
        self.gridLayout.addLayout(self.verticalLayout_5, 3, 0, 1, 1)
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(20, 190, 111, 54))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.createIns = QtWidgets.QPushButton(self.layoutWidget)
        self.createIns.setObjectName("createIns")
        self.verticalLayout_9.addWidget(self.createIns)
        self.cariCrop = QtWidgets.QPushButton(self.layoutWidget)
        self.cariCrop.setObjectName("cariCrop")
        self.verticalLayout_9.addWidget(self.cariCrop)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 451, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def create_ins(self):
        self.test = motivation.XYPlotter()
        print("Instance created")

    def cari_crop(self):
        self.test.caribrate_cam()

    def open_cam(self):
        self.test.capture()

    def crop_img(self):
        self.test.crop_cari_image()

    def con(self):
        self.method = self.test.TRACKING_CON
        self.test.detect_basic_shape(self.method)

    def app(self):
        self.method = self.test.TRACKING_APP
        self.test.detect_basic_shape(self.method)

    def prepare_dat(self):
        self.test.prepare_data(self.method)
        # self.test.__delete__()

    def de(self):
        self.method = self.test.TRACKING_POINT
        self.test.detect_basic_shape(self.method)
        # cv2.destroyAllWindows()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "XY_PlotterUI_test_0.2"))
        self.printDat.setText(_translate("MainWindow", "Print Data"))
        self.trackCon.setText(_translate("MainWindow", "TRACKING_CON"))
        self.trackApp.setText(_translate("MainWindow", "TRACKING_APP"))
        self.openCam.setText(_translate("MainWindow", "Open camera"))
        self.Capture.setText(_translate("MainWindow", "TRACKING_POINT"))
        self.cropImage.setText(_translate("MainWindow", "CropImage"))
        self.createIns.setText(_translate("MainWindow", "Create Instance"))
        self.cariCrop.setText(_translate("MainWindow", "Caribrate crop point"))
        self.createIns.clicked.connect(self.create_ins)
        self.cariCrop.clicked.connect(self.cari_crop)
        self.openCam.clicked.connect(self.open_cam)
        self.cropImage.clicked.connect(self.crop_img)
        self.trackApp.clicked.connect(self.app)
        self.trackCon.clicked.connect(self.con)
        self.Capture.clicked.connect(self.de)
        self.printDat.clicked.connect(self.prepare_dat)


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
