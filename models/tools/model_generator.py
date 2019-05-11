#!/usr/bin/env python
# -*- coding: utf-8 -*-

from gui import *
from generator import *
from PyQt5 import QtCore, QtGui, QtWidgets
import sys

if __name__ == '__main__' :
    app = QtWidgets.QApplication(sys.argv)
    # testing
    generate_controller([[0,1],[0,0]],[[0],[1]])
    # create gui object
    gui = ModelGenGUI()
    gui.show()
    sys.exit(app.exec_())
