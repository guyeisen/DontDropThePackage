"""THIS IS solver_viewer_gui originally taken as is from discopygal/tools
    WE DID NOT TOUCH ANYTHING HERE AND THIS IS NOT OUR CODE
    FULL CREDIT FOR THE DISCOPYGAL DEVELOPERS"""

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets

from discopygal.gui.gui import GUI
from discopygal.gui.logger import Logger

class About_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(414, 284)
        self.verticalLayout = QtWidgets.QVBoxLayout(Dialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/logo.png"))
        self.label.setScaledContents(True)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.label_2 = QtWidgets.QLabel(Dialog)
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setWordWrap(True)
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.label_3)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.label_4 = QtWidgets.QLabel(Dialog)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setItalic(True)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.verticalLayout.addWidget(self.label_4)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.label_2.setText(_translate("Dialog", "DiscoPygal - Solver Viewer"))
        self.label_3.setText(_translate("Dialog", "Based on the implementation of Nir Goren,  and refactored by Michael Bilevich.  Developed in Tel Aviv University,  Computational Geometry Lab,  under the supervision of Prof.  Dan Halperin."))
        self.label_4.setText(_translate("Dialog", "Copyright ©2019-2022 TAU CGL.  All right reserved."))

class Ui_dialog(object):
    def setupUi(self, dialog):
        dialog.setObjectName("dialog")
        dialog.resize(313, 209)
        self.verticalLayout = QtWidgets.QVBoxLayout(dialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(dialog)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.solverComboBox = QtWidgets.QComboBox(dialog)
        self.solverComboBox.setObjectName("solverComboBox")
        self.solverComboBox.addItem("")
        self.verticalLayout.addWidget(self.solverComboBox)
        self.selectButton = QtWidgets.QPushButton(dialog)
        self.selectButton.setObjectName("selectButton")
        self.verticalLayout.addWidget(self.selectButton)
        self.label_2 = QtWidgets.QLabel(dialog)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.browseButton = QtWidgets.QPushButton(dialog)
        self.browseButton.setObjectName("browseButton")
        self.verticalLayout.addWidget(self.browseButton)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)

        self.retranslateUi(dialog)
        QtCore.QMetaObject.connectSlotsByName(dialog)

    def retranslateUi(self, dialog):
        _translate = QtCore.QCoreApplication.translate
        dialog.setWindowTitle(_translate("dialog", "Dialog"))
        self.label.setText(_translate("dialog", "Choose a solver from the list below:"))
        self.solverComboBox.setItemText(0, _translate("dialog", "Select a solver..."))
        self.selectButton.setText(_translate("dialog", "Select Solver"))
        self.label_2.setText(_translate("dialog", "Or open a *.py file containing a solver class:"))
        self.browseButton.setText(_translate("dialog", "Browse..."))


class Ui_MainWindow(GUI):
    def __init__(self):
        super().__init__()

    def setupUi(self):
        MainWindow = self.mainWindow

        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1300, 800)
        MainWindow.setStyleSheet("QToolBar {\n"
"    background-color: rgb(200, 200, 200);\n"
"    border-width: 0px;\n"
"}")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setObjectName("verticalLayout")
        self.groupBox_2 = QtWidgets.QGroupBox(self.frame)
        self.groupBox_2.setMinimumSize(QtCore.QSize(0, 0))
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox_2)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.versionEdit = QtWidgets.QLineEdit(self.groupBox_2)
        self.versionEdit.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.versionEdit.sizePolicy().hasHeightForWidth())
        self.versionEdit.setSizePolicy(sizePolicy)
        self.versionEdit.setObjectName("versionEdit")
        self.gridLayout_2.addWidget(self.versionEdit, 1, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.groupBox_2)
        self.label_8.setObjectName("label_8")
        self.gridLayout_2.addWidget(self.label_8, 3, 0, 1, 1)
        self.solversEdit = QtWidgets.QLineEdit(self.groupBox_2)
        self.solversEdit.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.solversEdit.sizePolicy().hasHeightForWidth())
        self.solversEdit.setSizePolicy(sizePolicy)
        self.solversEdit.setObjectName("solversEdit")
        self.gridLayout_2.addWidget(self.solversEdit, 2, 1, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.groupBox_2)
        self.label_6.setObjectName("label_6")
        self.gridLayout_2.addWidget(self.label_6, 1, 0, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.groupBox_2)
        self.label_7.setObjectName("label_7")
        self.gridLayout_2.addWidget(self.label_7, 2, 0, 1, 1)
        self.sceneDetailsEdit = QtWidgets.QTextEdit(self.groupBox_2)
        self.sceneDetailsEdit.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sceneDetailsEdit.sizePolicy().hasHeightForWidth())
        self.sceneDetailsEdit.setSizePolicy(sizePolicy)
        self.sceneDetailsEdit.setObjectName("sceneDetailsEdit")
        self.gridLayout_2.addWidget(self.sceneDetailsEdit, 3, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox_2)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 0, 0, 1, 1)
        self.scenePathEdit = QtWidgets.QLineEdit(self.groupBox_2)
        self.scenePathEdit.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.scenePathEdit.sizePolicy().hasHeightForWidth())
        self.scenePathEdit.setSizePolicy(sizePolicy)
        self.scenePathEdit.setObjectName("scenePathEdit")
        self.gridLayout_2.addWidget(self.scenePathEdit, 0, 1, 1, 1)
        self.verticalLayout.addWidget(self.groupBox_2)
        self.solverFrame = QtWidgets.QGroupBox(self.frame)
        self.solverFrame.setObjectName("solverFrame")
        self.gridLayout = QtWidgets.QGridLayout(self.solverFrame)
        self.gridLayout.setObjectName("gridLayout")
        self.scrollArea = QtWidgets.QScrollArea(self.solverFrame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.scrollArea.sizePolicy().hasHeightForWidth())
        self.scrollArea.setSizePolicy(sizePolicy)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 382, 72))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.scrollAreaWidgetContents)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.noSolverLabel = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.noSolverLabel.setObjectName("noSolverLabel")
        self.gridLayout_3.addWidget(self.noSolverLabel, 0, 0, 1, 1)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.gridLayout.addWidget(self.scrollArea, 0, 0, 1, 1)
        self.verticalLayout.addWidget(self.solverFrame)
        self.groupBox = QtWidgets.QGroupBox(self.frame)
        self.groupBox.setMinimumSize(QtCore.QSize(0, 0))
        self.groupBox.setObjectName("groupBox")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.groupBox)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.textEdit = Logger(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.textEdit.sizePolicy().hasHeightForWidth())
        self.textEdit.setSizePolicy(sizePolicy)
        self.textEdit.setObjectName("textEdit")
        self.horizontalLayout_2.addWidget(self.textEdit)
        self.verticalLayout.addWidget(self.groupBox)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout.addWidget(self.frame)
        self.graphicsView = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphicsView.setObjectName("graphicsView")
        self.horizontalLayout.addWidget(self.graphicsView)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1300, 24))
        self.menubar.setNativeMenuBar(False)
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.toolBar = QtWidgets.QToolBar(MainWindow)
        self.toolBar.setAutoFillBackground(False)
        self.toolBar.setStyleSheet("")
        self.toolBar.setObjectName("toolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.actionOpenScene = QtWidgets.QAction(MainWindow)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/open_scene.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionOpenScene.setIcon(icon)
        self.actionOpenScene.setObjectName("actionOpenScene")
        self.actionOpenSolver = QtWidgets.QAction(MainWindow)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/open_solver.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionOpenSolver.setIcon(icon1)
        self.actionOpenSolver.setObjectName("actionOpenSolver")
        self.actionClear = QtWidgets.QAction(MainWindow)
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/clear.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionClear.setIcon(icon2)
        self.actionClear.setObjectName("actionClear")
        self.actionPlay = QtWidgets.QAction(MainWindow)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionPlay.setIcon(icon3)
        self.actionPlay.setObjectName("actionPlay")
        self.actionPause = QtWidgets.QAction(MainWindow)
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/pause.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionPause.setIcon(icon4)
        self.actionPause.setObjectName("actionPause")
        self.actionStop = QtWidgets.QAction(MainWindow)
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionStop.setIcon(icon5)
        self.actionStop.setObjectName("actionStop")
        self.actionShowPaths = QtWidgets.QAction(MainWindow)
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/paths.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionShowPaths.setIcon(icon6)
        self.actionShowPaths.setObjectName("actionShowPaths")
        self.actionShowGraph = QtWidgets.QAction(MainWindow)
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/graph.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionShowGraph.setIcon(icon7)
        self.actionShowGraph.setObjectName("actionShowGraph")
        self.actionShowArrangement = QtWidgets.QAction(MainWindow)
        icon8 = QtGui.QIcon()
        icon8.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/arrangement.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionShowArrangement.setIcon(icon8)
        self.actionShowArrangement.setObjectName("actionShowArrangement")
        self.actionAbout = QtWidgets.QAction(MainWindow)
        self.actionAbout.setObjectName("actionAbout")
        self.actionOpen_Scene = QtWidgets.QAction(MainWindow)
        self.actionOpen_Scene.setObjectName("actionOpen_Scene")
        self.actionOpen_Solver = QtWidgets.QAction(MainWindow)
        self.actionOpen_Solver.setObjectName("actionOpen_Solver")
        self.actionQuit = QtWidgets.QAction(MainWindow)
        self.actionQuit.setObjectName("actionQuit")
        self.actionSolve = QtWidgets.QAction(MainWindow)
        icon9 = QtGui.QIcon()
        icon9.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/solve.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionSolve.setIcon(icon9)
        self.actionSolve.setObjectName("actionSolve")
        self.actionVerify = QtWidgets.QAction(MainWindow)
        icon10 = QtGui.QIcon()
        icon10.addPixmap(QtGui.QPixmap("./tools/solver_viewer/resources/verify.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionVerify.setIcon(icon10)
        self.actionVerify.setObjectName("actionVerify")
        self.menuFile.addSeparator()
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionOpen_Scene)
        self.menuFile.addAction(self.actionOpen_Solver)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionQuit)
        self.menuHelp.addAction(self.actionAbout)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        self.toolBar.addAction(self.actionOpenScene)
        self.toolBar.addAction(self.actionOpenSolver)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionClear)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionSolve)
        self.toolBar.addAction(self.actionVerify)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionPlay)
        self.toolBar.addAction(self.actionPause)
        self.toolBar.addAction(self.actionStop)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionShowPaths)
        self.toolBar.addAction(self.actionShowGraph)
        self.toolBar.addAction(self.actionShowArrangement)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Scene Metadata"))
        self.label_8.setText(_translate("MainWindow", "Details"))
        self.label_6.setText(_translate("MainWindow", "DiscoPygal Version"))
        self.label_7.setText(_translate("MainWindow", "Suitable Solvers"))
        self.label_2.setText(_translate("MainWindow", "Scene path"))
        self.solverFrame.setTitle(_translate("MainWindow", "Solver Settings"))
        self.noSolverLabel.setText(_translate("MainWindow", "Please select a solver via the menu or the toolbar above."))
        self.groupBox.setTitle(_translate("MainWindow", "Output"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuHelp.setTitle(_translate("MainWindow", "Help"))
        self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar"))
        self.actionOpenScene.setText(_translate("MainWindow", "OpenScene"))
        self.actionOpenScene.setToolTip(_translate("MainWindow", "Open a scene file"))
        self.actionOpenSolver.setText(_translate("MainWindow", "OpenSolver"))
        self.actionOpenSolver.setToolTip(_translate("MainWindow", "Choose or open a solver"))
        self.actionClear.setText(_translate("MainWindow", "Clear"))
        self.actionClear.setToolTip(_translate("MainWindow", "Clear the scene"))
        self.actionPlay.setText(_translate("MainWindow", "Play"))
        self.actionPlay.setToolTip(_translate("MainWindow", "Play animation"))
        self.actionPause.setText(_translate("MainWindow", "Pause"))
        self.actionPause.setToolTip(_translate("MainWindow", "Pause animation"))
        self.actionStop.setText(_translate("MainWindow", "Stop"))
        self.actionStop.setToolTip(_translate("MainWindow", "Stop animation"))
        self.actionShowPaths.setText(_translate("MainWindow", "ShowPaths"))
        self.actionShowPaths.setToolTip(_translate("MainWindow", "Show paths"))
        self.actionShowGraph.setText(_translate("MainWindow", "ShowGraph"))
        self.actionShowGraph.setToolTip(_translate("MainWindow", "Show graph (if available)"))
        self.actionShowArrangement.setText(_translate("MainWindow", "ShowArrangement"))
        self.actionShowArrangement.setToolTip(_translate("MainWindow", "Show arrangement (if available)"))
        self.actionAbout.setText(_translate("MainWindow", "About"))
        self.actionOpen_Scene.setText(_translate("MainWindow", "Open Scene..."))
        self.actionOpen_Solver.setText(_translate("MainWindow", "Open Solver..."))
        self.actionQuit.setText(_translate("MainWindow", "Quit"))
        self.actionSolve.setText(_translate("MainWindow", "Solve"))
        self.actionSolve.setToolTip(_translate("MainWindow", "Solve"))
        self.actionVerify.setText(_translate("MainWindow", "Verify"))
        self.actionVerify.setToolTip(_translate("MainWindow", "Verify paths"))
