# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/mcampos/repo/mgear/scripts/mgear/maya/shifter/customStepUI.ui'
#
# Created: Wed Nov  9 14:50:41 2016
#      by: pyside-uic 0.2.14 running on PySide 1.2.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(348, 505)
        self.groupBox = QtGui.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(9, 6, 331, 491))
        self.groupBox.setObjectName("groupBox")
        self.layoutWidget = QtGui.QWidget(self.groupBox)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 20, 311, 459))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.preCustomStep_checkBox = QtGui.QCheckBox(self.layoutWidget)
        self.preCustomStep_checkBox.setObjectName("preCustomStep_checkBox")
        self.verticalLayout.addWidget(self.preCustomStep_checkBox)
        self.preCustomStep_horizontalLayout = QtGui.QHBoxLayout()
        self.preCustomStep_horizontalLayout.setObjectName("preCustomStep_horizontalLayout")
        self.preCustomStep_verticalLayout_1 = QtGui.QVBoxLayout()
        self.preCustomStep_verticalLayout_1.setObjectName("preCustomStep_verticalLayout_1")
        self.preCustomStep_listWidget = QtGui.QListWidget(self.layoutWidget)
        self.preCustomStep_listWidget.setDragDropOverwriteMode(True)
        self.preCustomStep_listWidget.setDragDropMode(QtGui.QAbstractItemView.InternalMove)
        self.preCustomStep_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.preCustomStep_listWidget.setAlternatingRowColors(True)
        self.preCustomStep_listWidget.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.preCustomStep_listWidget.setProperty("isWrapping", False)
        self.preCustomStep_listWidget.setViewMode(QtGui.QListView.ListMode)
        self.preCustomStep_listWidget.setObjectName("preCustomStep_listWidget")
        self.preCustomStep_verticalLayout_1.addWidget(self.preCustomStep_listWidget)
        self.preCustomStep_horizontalLayout.addLayout(self.preCustomStep_verticalLayout_1)
        self.preCustomStep_verticalLayout_2 = QtGui.QVBoxLayout()
        self.preCustomStep_verticalLayout_2.setObjectName("preCustomStep_verticalLayout_2")
        self.preCustomStepAdd_pushButton = QtGui.QPushButton(self.layoutWidget)
        self.preCustomStepAdd_pushButton.setObjectName("preCustomStepAdd_pushButton")
        self.preCustomStep_verticalLayout_2.addWidget(self.preCustomStepAdd_pushButton)
        self.preCustomStepRemove_pushButton = QtGui.QPushButton(self.layoutWidget)
        self.preCustomStepRemove_pushButton.setObjectName("preCustomStepRemove_pushButton")
        self.preCustomStep_verticalLayout_2.addWidget(self.preCustomStepRemove_pushButton)
        self.preCustomStepRun_pushButton = QtGui.QPushButton(self.layoutWidget)
        self.preCustomStepRun_pushButton.setObjectName("preCustomStepRun_pushButton")
        self.preCustomStep_verticalLayout_2.addWidget(self.preCustomStepRun_pushButton)
        self.preCustomStepEdit_pushButton = QtGui.QPushButton(self.layoutWidget)
        self.preCustomStepEdit_pushButton.setObjectName("preCustomStepEdit_pushButton")
        self.preCustomStep_verticalLayout_2.addWidget(self.preCustomStepEdit_pushButton)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.preCustomStep_verticalLayout_2.addItem(spacerItem)
        self.preCustomStep_horizontalLayout.addLayout(self.preCustomStep_verticalLayout_2)
        self.verticalLayout.addLayout(self.preCustomStep_horizontalLayout)
        self.verticalLayout_3.addLayout(self.verticalLayout)
        self.line = QtGui.QFrame(self.layoutWidget)
        self.line.setLineWidth(3)
        self.line.setMidLineWidth(0)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_3.addWidget(self.line)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.postCustomStep_checkBox = QtGui.QCheckBox(self.layoutWidget)
        self.postCustomStep_checkBox.setObjectName("postCustomStep_checkBox")
        self.verticalLayout_2.addWidget(self.postCustomStep_checkBox)
        self.preCustomStep_horizontalLayout_2 = QtGui.QHBoxLayout()
        self.preCustomStep_horizontalLayout_2.setObjectName("preCustomStep_horizontalLayout_2")
        self.preCustomStep_verticalLayout_3 = QtGui.QVBoxLayout()
        self.preCustomStep_verticalLayout_3.setObjectName("preCustomStep_verticalLayout_3")
        self.postCustomStep_listWidget = QtGui.QListWidget(self.layoutWidget)
        self.postCustomStep_listWidget.setDragDropOverwriteMode(True)
        self.postCustomStep_listWidget.setDragDropMode(QtGui.QAbstractItemView.InternalMove)
        self.postCustomStep_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.postCustomStep_listWidget.setAlternatingRowColors(True)
        self.postCustomStep_listWidget.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.postCustomStep_listWidget.setViewMode(QtGui.QListView.ListMode)
        self.postCustomStep_listWidget.setWordWrap(False)
        self.postCustomStep_listWidget.setSelectionRectVisible(False)
        self.postCustomStep_listWidget.setObjectName("postCustomStep_listWidget")
        self.preCustomStep_verticalLayout_3.addWidget(self.postCustomStep_listWidget)
        self.preCustomStep_horizontalLayout_2.addLayout(self.preCustomStep_verticalLayout_3)
        self.preCustomStep_verticalLayout_4 = QtGui.QVBoxLayout()
        self.preCustomStep_verticalLayout_4.setObjectName("preCustomStep_verticalLayout_4")
        self.postCustomStepAdd_pushButton = QtGui.QPushButton(self.layoutWidget)
        self.postCustomStepAdd_pushButton.setObjectName("postCustomStepAdd_pushButton")
        self.preCustomStep_verticalLayout_4.addWidget(self.postCustomStepAdd_pushButton)
        self.postCustomStepRemove_pushButton = QtGui.QPushButton(self.layoutWidget)
        self.postCustomStepRemove_pushButton.setObjectName("postCustomStepRemove_pushButton")
        self.preCustomStep_verticalLayout_4.addWidget(self.postCustomStepRemove_pushButton)
        self.postCustomStepRun_pushButton = QtGui.QPushButton(self.layoutWidget)
        self.postCustomStepRun_pushButton.setObjectName("postCustomStepRun_pushButton")
        self.preCustomStep_verticalLayout_4.addWidget(self.postCustomStepRun_pushButton)
        self.postCustomStepEdit_pushButton = QtGui.QPushButton(self.layoutWidget)
        self.postCustomStepEdit_pushButton.setObjectName("postCustomStepEdit_pushButton")
        self.preCustomStep_verticalLayout_4.addWidget(self.postCustomStepEdit_pushButton)
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.preCustomStep_verticalLayout_4.addItem(spacerItem1)
        self.preCustomStep_horizontalLayout_2.addLayout(self.preCustomStep_verticalLayout_4)
        self.verticalLayout_2.addLayout(self.preCustomStep_horizontalLayout_2)
        self.verticalLayout_3.addLayout(self.verticalLayout_2)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtGui.QApplication.translate("Form", "Form", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox.setTitle(QtGui.QApplication.translate("Form", "Custom Steps", None, QtGui.QApplication.UnicodeUTF8))
        self.preCustomStep_checkBox.setText(QtGui.QApplication.translate("Form", "Pre Custom Step", None, QtGui.QApplication.UnicodeUTF8))
        self.preCustomStepAdd_pushButton.setText(QtGui.QApplication.translate("Form", "Add", None, QtGui.QApplication.UnicodeUTF8))
        self.preCustomStepRemove_pushButton.setText(QtGui.QApplication.translate("Form", "Remove", None, QtGui.QApplication.UnicodeUTF8))
        self.preCustomStepRun_pushButton.setText(QtGui.QApplication.translate("Form", "Run Sel.", None, QtGui.QApplication.UnicodeUTF8))
        self.preCustomStepEdit_pushButton.setText(QtGui.QApplication.translate("Form", "Edit", None, QtGui.QApplication.UnicodeUTF8))
        self.postCustomStep_checkBox.setText(QtGui.QApplication.translate("Form", "Post  Custom Step", None, QtGui.QApplication.UnicodeUTF8))
        self.postCustomStepAdd_pushButton.setText(QtGui.QApplication.translate("Form", "Add", None, QtGui.QApplication.UnicodeUTF8))
        self.postCustomStepRemove_pushButton.setText(QtGui.QApplication.translate("Form", "Remove", None, QtGui.QApplication.UnicodeUTF8))
        self.postCustomStepRun_pushButton.setText(QtGui.QApplication.translate("Form", "Run Sel.", None, QtGui.QApplication.UnicodeUTF8))
        self.postCustomStepEdit_pushButton.setText(QtGui.QApplication.translate("Form", "Edit", None, QtGui.QApplication.UnicodeUTF8))
