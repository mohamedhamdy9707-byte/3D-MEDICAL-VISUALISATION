 """
3D Medical Visualization System - FOCUS NAVIGATION + ENHANCED FLY-THROUGH
Supports: DICOM, NIfTI, VTK, STL, OBJ, Multiple OBJ files
NEW: Focus Navigation shows internal structures by making outer layers transparent
ENHANCED: Smooth fly-through with entry transition and interior clipping
MODIFIED: "Moving Stuff" now launches the EEG/ECG application.
"""

import sys
import os
import numpy as np
import subprocess
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QComboBox, QLabel, QPushButton,
                             QSlider, QGroupBox, QFileDialog, QMessageBox,
                             QListWidget, QSplitter, QListWidgetItem,
                             QDialog, QDialogButtonBox, QRadioButton, QButtonGroup)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


class DatasetLoader:
    """Load real medical imaging datasets"""

    @staticmethod
    def load_dicom_series(directory):
        try:
            reader = vtk.vtkDICOMImageReader()
            reader.SetDirectory
