# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
import motivation


class Ui_Dialog(object):
    def funcion(self):
        self.test.capture()
        self.test.crop_cari_image()
        self.test.detect_basic_shape()

    def initproject(self):
        self.test = motivation.XYPlotter()

    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(640, 480)
        Dialog.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        Dialog.setMouseTracking(False)
        self.pushButton = QtWidgets.QPushButton(Dialog)
        self.pushButton.setGeometry(QtCore.QRect(260, 220, 101, 31))
        self.pushButton.setObjectName("pushButton")
        self.pushButton2 = QtWidgets.QPushButton(Dialog)
        self.pushButton2.setGeometry(QtCore.QRect(250, 100, 101, 31))
        self.pushButton2.setObjectName("pushButton2")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.pushButton.setText(_translate("Dialog", "\"Hello!\""))
        self.pushButton.clicked.connect(self.funcion)
        self.pushButton2.setText(_translate("Dialog", "\"First!\""))
        self.pushButton2.clicked.connect(self.initproject)


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())
