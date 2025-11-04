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
            reader.SetDirectoryName(directory)
            reader.Update()
            marchingCubes = vtk.vtkMarchingCubes()
            marchingCubes.SetInputConnection(reader.GetOutputPort())
            marchingCubes.SetValue(0, 400)
            marchingCubes.Update()
            return marchingCubes.GetOutput(), reader.GetOutput(), "DICOM Series"
        except Exception as e:
            print(f"Error loading DICOM: {e}")
            return None, None, None

    @staticmethod
    def load_nifti(filepath):
        try:
            reader = vtk.vtkNIFTIImageReader()
            reader.SetFileName(filepath)
            reader.Update()
            marchingCubes = vtk.vtkMarchingCubes()
            marchingCubes.SetInputConnection(reader.GetOutputPort())
            marchingCubes.SetValue(0, 100)
            marchingCubes.Update()
            return marchingCubes.GetOutput(), reader.GetOutput(), "NIfTI Volume"
        except Exception as e:
            print(f"Error loading NIfTI: {e}")
            return None, None, None

    @staticmethod
    def load_vtk_file(filepath):
        try:
            if filepath.endswith('.vtp'):
                reader = vtk.vtkXMLPolyDataReader()
            else:
                reader = vtk.vtkPolyDataReader()
            reader.SetFileName(filepath)
            reader.Update()
            return reader.GetOutput(), None, "VTK PolyData"
        except Exception as e:
            print(f"Error loading VTK: {e}")
            return None, None, None

    @staticmethod
    def load_stl(filepath):
        try:
            reader = vtk.vtkSTLReader()
            reader.SetFileName(filepath)
            reader.Update()
            return reader.GetOutput(), None, "STL Mesh"
        except Exception as e:
            print(f"Error loading STL: {e}")
            return None, None, None

    @staticmethod
    def load_obj(filepath):
        try:
            reader = vtk.vtkOBJReader()
            reader.SetFileName(filepath)
            reader.Update()
            return reader.GetOutput(), None, "OBJ Mesh"
        except Exception as e:
            print(f"Error loading OBJ: {e}")
            return None, None, None

    @staticmethod
    def load_multiple_obj(filepaths):
        try:
            append_filter = vtk.vtkAppendPolyData()
            successful_loads = 0
            for filepath in filepaths:
                reader = vtk.vtkOBJReader()
                reader.SetFileName(filepath)
                reader.Update()
                if reader.GetOutput().GetNumberOfPoints() > 0:
                    append_filter.AddInputData(reader.GetOutput())
                    successful_loads += 1
            if successful_loads == 0:
                return None, None, None
            append_filter.Update()
            return append_filter.GetOutput(), None, f"Multi-OBJ ({successful_loads} parts)"
        except Exception as e:
            print(f"Error loading multiple OBJ files: {e}")
            return None, None, None

    @staticmethod
    def auto_load(filepath_or_dir):
        if os.path.isdir(filepath_or_dir):
            return DatasetLoader.load_dicom_series(filepath_or_dir)
        ext = os.path.splitext(filepath_or_dir)[1].lower()
        if ext in ['.nii', '.gz'] or filepath_or_dir.endswith('.nii.gz'):
            return DatasetLoader.load_nifti(filepath_or_dir)
        elif ext in ['.vtk', '.vtp']:
            return DatasetLoader.load_vtk_file(filepath_or_dir)
        elif ext == '.stl':
            return DatasetLoader.load_stl(filepath_or_dir)
        elif ext == '.obj':
            return DatasetLoader.load_obj(filepath_or_dir)
        else:
            return None, None, None


class SystemAssignDialog(QDialog):
    def __init__(self, dataset_name, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Assign Anatomical System")
        layout = QVBoxLayout(self)
        label = QLabel(f"Assign '{dataset_name}' to anatomical system:")
        label.setWordWrap(True)
        layout.addWidget(label)
        self.button_group = QButtonGroup(self)
        systems = ["Nervous", "Cardiovascular", "Musculoskeletal", "Dental"]
        colors = {
            "Nervous": "Brain, spinal cord, nerves",
            "Cardiovascular": "Heart, blood vessels",
            "Musculoskeletal": "Bones, joints, muscles",
            "Dental": "Teeth, jaw, oral structures"
        }
        for i, system in enumerate(systems):
            rb = QRadioButton(f"{system} - {colors[system]}")
            if i == 0:
                rb.setChecked(True)
            self.button_group.addButton(rb, i)
            layout.addWidget(rb)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_selected_system(self):
        systems = ["Nervous", "Cardiovascular", "Musculoskeletal", "Dental"]
        idx = self.button_group.checkedId()
        return systems[idx] if idx >= 0 else systems[0]


class MedicalVisualization3D(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("3D Medical Visualization - Focus + Enhanced Fly-Through")
        self.setGeometry(100, 100, 1600, 1000)

        self.current_system = "Nervous"
        self.current_viz = "Surface Rendering"
        self.current_nav = "Focus Navigation"

        self.loaded_datasets = {}
        self.dataset_volumes = {}
        self.dataset_types = {}
        self.dataset_systems = {}
        self.dataset_actors = {}
        self.dataset_colors = {}
        self.dataset_parts = {}
        self.clipping_filters = {}
        self.clipping_actors = {}
        self.curved_mpr_actors = {}

        # ENHANCED FLY-THROUGH STATE
        self.flythrough_timer = QTimer()
        self.flythrough_timer.timeout.connect(self.update_flythrough_animation)
        self.flythrough_timer.setInterval(16)  # 60 FPS
        self.flythrough_phase = "idle"  # idle, entry, main
        self.flythrough_t = 0.0
        self.entry_t = 0.0
        self.flythrough_speed = 0.0005
        self.entry_speed = 0.003
        self.look_ahead_distance = 5.0
        self.clip_offset = 0.5

        self.camera_path_points = []
        self.path_tangents = []
        self.path_normals = []
        self.path_binormals = []

        self.flythrough_is_playing = False
        self.flythrough_original_camera_pos = None
        self.flythrough_original_camera_focal = None
        self.flythrough_original_camera_view_up = None
        self.flythrough_original_opacity = {}
        self.flythrough_original_interactor_style = None

        self.interior_clip_plane = vtk.vtkPlane()

        # Preview plane for clipping (add after line ~90)
        self.preview_plane_src = None
        self.preview_plane_actor = None

        self.interior_clip_enabled = False

        # Picking state
        self.fly_path_actor = None
        self.instruction_text_actor = None
        self.picked_point_count = 0
        self.fly_original_opacity = {}
        self._picking_active = False
        self._picker_points = []
        self._picker_marker_actors = []

        self.system_colors = {
            "Nervous": (0.9, 0.7, 0.7),
            "Cardiovascular": (0.8, 0.2, 0.2),
            "Musculoskeletal": (0.9, 0.9, 0.8),
            "Dental": (1.0, 1.0, 0.95)
        }

        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.1, 0.15)

        light1 = vtk.vtkLight()
        light1.SetPosition(10, 10, 10)
        light1.SetFocalPoint(0, 0, 0)
        self.renderer.AddLight(light1)

        light2 = vtk.vtkLight()
        light2.SetPosition(-10, -10, 10)
        light2.SetIntensity(0.5)
        self.renderer.AddLight(light2)

        self.clip_plane = vtk.vtkPlane()
        self.clip_plane.SetNormal(1, 0, 0)
        self.clip_inverted = False

        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.update_animation)
        self.animation_frame = 0

        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        left_splitter = QSplitter(Qt.Vertical)
        dataset_widget = self.create_dataset_manager()
        left_splitter.addWidget(dataset_widget)

        self.vtk_widget = QVTKRenderWindowInteractor()
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        left_splitter.addWidget(self.vtk_widget)

        left_splitter.setStretchFactor(0, 1)
        left_splitter.setStretchFactor(1, 4)
        main_layout.addWidget(left_splitter, stretch=3)

        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)

        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(0, 0, 200)
        camera.SetFocalPoint(0, 0, 0)
        camera.SetViewUp(0, 1, 0)

        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(style)
        self.interactor.Initialize()

        self.show_welcome_message()

    def show_welcome_message(self):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle("Welcome - Enhanced Features!")
        msg.setText("<h3>Focus Navigation + Smooth Fly-Through</h3>")
        msg.setInformativeText(
            "<b>✨ Focus Navigation:</b><br>"
            "• Move slider to see deep inside organs<br>"
            "• 0% = See internal structures<br>"
            "• 100% = See outer surface<br><br>"
            "<b>✈️ Enhanced Fly-Through:</b><br>"
            "• Smooth entry transition<br>"
            "• Natural camera rotation<br>"
            "• Interior clipping for see-through<br>"
            "• Pick 3 points, then Play!<br><br>"
            "<b>🧠❤️ Moving Stuff:</b><br>"
            "• Launches the EEG/ECG Visualizer"
        )
        msg.exec_()

    def create_dataset_manager(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        title = QLabel("📁 Dataset Manager")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        btn_layout = QHBoxLayout()
        load_file_btn = QPushButton("📄 Load File")
        load_file_btn.clicked.connect(self.load_file)
        load_file_btn.setStyleSheet("padding: 8px; font-weight: bold;")
        btn_layout.addWidget(load_file_btn)

        load_dir_btn = QPushButton("📂 Load DICOM")
        load_dir_btn.clicked.connect(self.load_directory)
        load_dir_btn.setStyleSheet("padding: 8px; font-weight: bold;")
        btn_layout.addWidget(load_dir_btn)
        layout.addLayout(btn_layout)

        load_multi_obj_btn = QPushButton("📦 Load Multiple OBJ")
        load_multi_obj_btn.clicked.connect(self.load_multiple_obj_files)
        load_multi_obj_btn.setStyleSheet("padding: 8px; font-weight: bold; background-color: #2a5a2a;")
        layout.addWidget(load_multi_obj_btn)

        list_label = QLabel("Loaded Datasets:")
        layout.addWidget(list_label)

        self.dataset_list = QListWidget()
        self.dataset_list.itemSelectionChanged.connect(self.on_dataset_selected)
        layout.addWidget(self.dataset_list)

        dataset_btn_layout = QHBoxLayout()
        toggle_btn = QPushButton("👁 Toggle Visibility")
        toggle_btn.clicked.connect(self.toggle_dataset_visibility)
        dataset_btn_layout.addWidget(toggle_btn)

        remove_btn = QPushButton("🗑 Remove")
        remove_btn.clicked.connect(self.remove_dataset)
        dataset_btn_layout.addWidget(remove_btn)
        layout.addLayout(dataset_btn_layout)

        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self.clear_all_datasets)
        layout.addWidget(clear_btn)

        self.info_label = QLabel("No datasets loaded\n\n💡 Load data to start!")
        self.info_label.setWordWrap(True)
        self.info_label.setStyleSheet("background: #1a1a2a; padding: 10px; border-radius: 5px;")
        layout.addWidget(self.info_label)

        return widget

    def create_control_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        title = QLabel("⚙️ Control Panel")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        layout.addWidget(title)

        system_group = QGroupBox("1. Filter by System")
        system_layout = QVBoxLayout()
        filter_label = QLabel("Show datasets from:")
        system_layout.addWidget(filter_label)
        self.system_combo = QComboBox()
        self.system_combo.addItems(["All Systems", "Nervous", "Cardiovascular", "Musculoskeletal", "Dental"])
        self.system_combo.currentTextChanged.connect(self.on_system_filter_changed)
        system_layout.addWidget(self.system_combo)
        system_group.setLayout(system_layout)
        layout.addWidget(system_group)

        viz_group = QGroupBox("2. Visualization Method")
        viz_layout = QVBoxLayout()
        self.viz_combo = QComboBox()
        self.viz_combo.addItems(["Surface Rendering", "Clipping Planes", "Curved MPR"])
        self.viz_combo.currentTextChanged.connect(self.on_viz_changed)
        viz_layout.addWidget(self.viz_combo)
        viz_group.setLayout(viz_layout)
        layout.addWidget(viz_group)

        nav_group = QGroupBox("3. Navigation Technique")
        nav_layout = QVBoxLayout()
        self.nav_combo = QComboBox()
        self.nav_combo.addItems(["Focus Navigation", "Moving Stuff", "Fly-through"])
        self.nav_combo.currentTextChanged.connect(self.on_nav_changed)
        nav_layout.addWidget(self.nav_combo)
        nav_info = QLabel("Focus = See through outer layers")
        nav_info.setStyleSheet("color: #4a4; font-size: 10px;")
        nav_layout.addWidget(nav_info)
        nav_group.setLayout(nav_layout)
        layout.addWidget(nav_group)

        trans_group = QGroupBox("🔍 Focus: Depth Control")
        trans_layout = QVBoxLayout()
        self.trans_label = QLabel("Focus Depth: 100%")
        trans_layout.addWidget(self.trans_label)
        self.trans_slider = QSlider(Qt.Horizontal)
        self.trans_slider.setMinimum(0)
        self.trans_slider.setMaximum(100)
        self.trans_slider.setValue(100)
        self.trans_slider.valueChanged.connect(self.on_transparency_changed)
        trans_layout.addWidget(self.trans_slider)
        focus_info = QLabel("0% = See deep inside\n100% = See surface")
        focus_info.setStyleSheet("color: #4a4; font-size: 10px;")
        trans_layout.addWidget(focus_info)
        trans_group.setLayout(trans_layout)
        layout.addWidget(trans_group)

        clip_group = QGroupBox("✂️ Clipping: Preview & Apply")
        clip_layout = QVBoxLayout()

        self.clip_label = QLabel("Position: 0.0")
        clip_layout.addWidget(self.clip_label)

        # Slider now moves preview plane (not immediate clipping)
        self.clip_slider = QSlider(Qt.Horizontal)
        self.clip_slider.setMinimum(0)
        self.clip_slider.setMaximum(100)
        self.clip_slider.setValue(0)
        self.clip_slider.valueChanged.connect(self.on_clipping_preview_changed)
        clip_layout.addWidget(self.clip_slider)

        dir_layout = QHBoxLayout()
        normal_label = QLabel("Cut Direction:")
        dir_layout.addWidget(normal_label)
        self.normal_combo = QComboBox()
        self.normal_combo.addItems(["X-axis", "Y-axis", "Z-axis"])
        self.normal_combo.currentTextChanged.connect(self.on_normal_changed)
        dir_layout.addWidget(self.normal_combo)
        clip_layout.addLayout(dir_layout)

        # Apply button
        self.apply_clip_btn = QPushButton("✂ Apply Clipping")
        self.apply_clip_btn.setStyleSheet("background-color: #882222; color: white; font-weight: bold; padding: 8px;")
        self.apply_clip_btn.clicked.connect(self.apply_clipping_plane)
        clip_layout.addWidget(self.apply_clip_btn)

        self.invert_btn = QPushButton("🔄 Invert Direction")
        self.invert_btn.clicked.connect(self.invert_clipping_direction)
        self.invert_btn.setStyleSheet("padding: 5px; background-color: #3a4a5a;")
        clip_layout.addWidget(self.invert_btn)

        clip_info = QLabel("Move slider to preview plane.\nClick 'Apply Clipping' to cut.")
        clip_info.setStyleSheet("color: #4a4; font-size: 10px;")
        clip_layout.addWidget(clip_info)

        clip_group.setLayout(clip_layout)
        layout.addWidget(clip_group)

        # ENHANCED FLY-THROUGH CONTROL
        flythrough_group = QGroupBox("✈️ Enhanced Fly-Through")
        flythrough_layout = QVBoxLayout()

        self.pick_points_btn = QPushButton("Pick Three Points")
        self.pick_points_btn.clicked.connect(self.start_inline_picking)
        self.pick_points_btn.setVisible(False)
        flythrough_layout.addWidget(self.pick_points_btn)

        self.flyplay_btn = QPushButton("▶ Play")
        # --- THIS IS THE CORRECTED LINE ---
        self.flyplay_btn.clicked.connect(self._on_flyplay_pressed)
        # --- END OF CORRECTION ---
        self.flyplay_btn.setVisible(False)
        self.flyplay_btn.setEnabled(False)
        flythrough_layout.addWidget(self.flyplay_btn)

        self.fly_instr_label = QLabel("Select 'Fly-through' to start")
        self.fly_instr_label.setStyleSheet("color: #888; font-size: 10px;")
        flythrough_layout.addWidget(self.fly_instr_label)

        flythrough_group.setLayout(flythrough_layout)
        layout.addWidget(flythrough_group)

        anim_group = QGroupBox("🎬 Animation")
        anim_layout = QVBoxLayout()
        self.play_btn = QPushButton("▶ Play Animation")
        self.play_btn.clicked.connect(self.toggle_animation)
        anim_layout.addWidget(self.play_btn)
        self.speed_label = QLabel("Speed: 1.0x")
        anim_layout.addWidget(self.speed_label)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(20)
        self.speed_slider.setValue(10)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        anim_layout.addWidget(self.speed_slider)
        anim_group.setLayout(anim_layout)
        layout.addWidget(anim_group)

        combo_label = QLabel("📊 Current Combination:")
        combo_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        layout.addWidget(combo_label)
        self.combo_display = QLabel()
        self.combo_display.setWordWrap(True)
        self.combo_display.setStyleSheet("background: #1a1a2a; padding: 10px; border-radius: 5px;")
        layout.addWidget(self.combo_display)
        self.update_combo_display()

        btn_layout = QHBoxLayout()
        reset_btn = QPushButton("🔄 Reset View")
        reset_btn.clicked.connect(self.reset_view)
        btn_layout.addWidget(reset_btn)
        screenshot_btn = QPushButton("📷 Screenshot")
        screenshot_btn.clicked.connect(self.take_screenshot)
        btn_layout.addWidget(screenshot_btn)
        layout.addLayout(btn_layout)

        layout.addStretch()
        return panel

    # === ENHANCED FLY-THROUGH IMPLEMENTATION ===

    def start_inline_picking(self):
        if not self.loaded_datasets:
            QMessageBox.information(self, "No Data", "Please load a dataset first!")
            return

        if self.current_nav != "Fly-through":
            QMessageBox.warning(
                self,
                "Wrong Navigation Mode",
                "Please select 'Fly-through' from Navigation Technique first!"
            )
            return

        if self._picking_active:
            return

        for a in list(self._picker_marker_actors):
            try:
                self.renderer.RemoveActor(a)
            except Exception:
                pass
        self._picker_marker_actors.clear()
        self._picker_points = []
        if self.fly_path_actor:
            try:
                self.renderer.RemoveActor(self.fly_path_actor)
            except Exception:
                pass
            self.fly_path_actor = None

        self._picking_active = True
        self.pick_points_btn.setEnabled(False)
        self.fly_instr_label.setText(
            "Press 'C' over organ to pick point (need 3). Esc to cancel."
        )
        self.show_instruction_text("Zoom in and press C to pick point")
        self.picked_point_count = 0
        self.pick_points_btn.setText("Points (0/3)")
        self._pick_key_obs_id = self.interactor.AddObserver(
            "KeyPressEvent", self._on_main_key_press
        )

    def _on_main_key_press(self, caller, event):
        if not self._picking_active:
            return
        key = self.interactor.GetKeySym().lower()

        if key == "c":
            click_pos = self.interactor.GetEventPosition()
            picker = vtk.vtkCellPicker()
            picker.SetTolerance(0.001)
            picker.Pick(click_pos[0], click_pos[1], 0, self.renderer)
            pos = picker.GetPickPosition()
            cell_id = picker.GetCellId()

            if (
                    cell_id >= 0
                    and pos is not None
                    and not (pos[0] == 0 and pos[1] == 0 and pos[2] == 0)
            ):
                self._picker_points.append(
                    [float(pos[0]), float(pos[1]), float(pos[2])]
                )
                self._add_pick_marker(pos)

                self.picked_point_count += 1
                self.pick_points_btn.setText(f"Points ({self.picked_point_count}/3)")

                if len(self._picker_points) >= 3:
                    try:
                        self.interactor.RemoveObserver(self._pick_key_obs_id)
                    except Exception:
                        pass
                    self._picking_active = False
                    self.pick_points_btn.setEnabled(True)
                    self._finish_main_picking()
            return 1

        elif key == "escape":
            try:
                self.interactor.RemoveObserver(self._pick_key_obs_id)
            except Exception:
                pass
            for a in list(self._picker_marker_actors):
                try:
                    self.renderer.RemoveActor(a)
                except Exception:
                    pass
            self._picker_marker_actors.clear()
            self._picker_points.clear()
            self._picking_active = False
            self.pick_points_btn.setEnabled(True)
            self.fly_instr_label.setText("Picking canceled.")
            self.vtk_widget.GetRenderWindow().Render()

    def _add_pick_marker(self, world_pos):
        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(world_pos)
        sphere.SetRadius(
            max(1.0, 0.01 * max(self.renderer.ComputeVisiblePropBounds() or [1]))
        )
        sphere.SetThetaResolution(12)
        sphere.SetPhiResolution(12)
        sphere.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.0, 0.2, 1.0)
        actor.GetProperty().SetSpecular(0.2)
        actor.GetProperty().SetSpecularPower(10)

        self.renderer.AddActor(actor)
        self._picker_marker_actors.append(actor)
        self.vtk_widget.GetRenderWindow().Render()

    def show_instruction_text(self, text):
        """Display blue instruction text above the object"""
        if self.instruction_text_actor:
            self.renderer.RemoveActor(self.instruction_text_actor)

        bounds = None
        for name, actor in self.dataset_actors.items():
            if actor.GetVisibility():
                bounds = self.loaded_datasets[name].GetBounds()
                break

        if bounds:
            text_actor = vtk.vtkTextActor()
            text_actor.SetInput(text)
            text_actor.GetTextProperty().SetFontSize(24)
            text_actor.GetTextProperty().SetColor(0.0, 0.5, 1.0)
            text_actor.GetTextProperty().SetJustificationToCentered()
            text_actor.SetDisplayPosition(
                int(self.vtk_widget.width() / 2),
                int(self.vtk_widget.height() * 0.85)
            )

            self.renderer.AddActor2D(text_actor)
            self.instruction_text_actor = text_actor
            self.vtk_widget.GetRenderWindow().Render()

    def _finish_main_picking(self):
        if len(self._picker_points) < 3:
            return

        p0, p1, p2 = [np.array(pt) for pt in self._picker_points]

        mid1 = (p0 + p1) / 2.0
        mid2 = (p1 + p2) / 2.0

        bounds = None
        for name, actor in self.dataset_actors.items():
            if actor.GetVisibility():
                bounds = self.loaded_datasets[name].GetBounds()
                break

        if bounds:
            center = np.array(
                [
                    (bounds[0] + bounds[1]) / 2,
                    (bounds[2] + bounds[3]) / 2,
                    (bounds[4] + bounds[5]) / 2,
                ]
            )

            dist1 = np.linalg.norm(p1 - p0)
            dir_to_center1 = center - mid1
            if np.linalg.norm(dir_to_center1) > 0:
                dir_to_center1 = dir_to_center1 / np.linalg.norm(dir_to_center1)
                mid1 = mid1 + dir_to_center1 * (dist1 * 0.15)

            dist2 = np.linalg.norm(p2 - p1)
            dir_to_center2 = center - mid2
            if np.linalg.norm(dir_to_center2) > 0:
                dir_to_center2 = dir_to_center2 / np.linalg.norm(dir_to_center2)
                mid2 = mid2 + dir_to_center2 * (dist2 * 0.15)

        points = [p0, mid1, p1, mid2, p2]

        temp_path_points = []
        n_segments = len(points) - 1
        samples_per_segment = 200

        for i in range(n_segments):
            p_prev = points[max(0, i - 1)]
            p_start = points[i]
            p_end = points[i + 1]
            p_next = points[min(len(points) - 1, i + 2)]

            for j in range(samples_per_segment):
                t = j / float(samples_per_segment)
                t2 = t * t
                t3 = t2 * t

                pt = 0.5 * (
                        (2 * p_start)
                        + (-p_prev + p_end) * t
                        + (2 * p_prev - 5 * p_start + 4 * p_end - p_next) * t2
                        + (-p_prev + 3 * p_start - 3 * p_end + p_next) * t3
                )
                temp_path_points.append(np.array(pt))

        cumulative_distances = [0.0]
        for i in range(1, len(temp_path_points)):
            dist = np.linalg.norm(temp_path_points[i] - temp_path_points[i - 1])
            cumulative_distances.append(cumulative_distances[-1] + dist)

        total_length = cumulative_distances[-1]
        target_num_points = len(temp_path_points)
        uniform_spacing = total_length / (target_num_points - 1)

        self.camera_path_points = [list(temp_path_points[0])]
        target_distance = uniform_spacing
        idx = 0

        while target_distance < total_length and idx < len(cumulative_distances) - 1:
            while idx < len(cumulative_distances) - 1 and cumulative_distances[idx + 1] < target_distance:
                idx += 1

            if idx < len(cumulative_distances) - 1:
                local_t = (target_distance - cumulative_distances[idx]) / (
                            cumulative_distances[idx + 1] - cumulative_distances[idx])
                interpolated_pt = temp_path_points[idx] + local_t * (temp_path_points[idx + 1] - temp_path_points[idx])
                self.camera_path_points.append(list(interpolated_pt))
                target_distance += uniform_spacing

        self.camera_path_points.append(list(temp_path_points[-1]))

        self.path_tangents = []
        self.path_normals = []
        n = len(self.camera_path_points)
        for i in range(n):
            if i < n - 1:
                tangent = np.array(self.camera_path_points[i + 1]) - np.array(self.camera_path_points[i])
            else:
                tangent = np.array(self.camera_path_points[i]) - np.array(self.camera_path_points[i - 1])

            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm > 0:
                tangent = tangent / tangent_norm
            else:
                tangent = np.array([0, 0, 1])

            self.path_tangents.append(list(tangent))
            self.path_normals.append([0, 1, 0])

        self.fly_original_opacity.clear()
        for name, actor in self.dataset_actors.items():
            if actor.GetVisibility():
                self.fly_original_opacity[name] = actor.GetProperty().GetOpacity()

        self.create_path_visualization(None)

        for name, actor in self.dataset_actors.items():
            if actor.GetVisibility():
                actor.GetProperty().SetOpacity(0.3)

        try:
            self.flyplay_btn.setVisible(True)
        except Exception:
            pass

        if self.instruction_text_actor:
            self.renderer.RemoveActor(self.instruction_text_actor)
            self.instruction_text_actor = None

        self.pick_points_btn.setText("Pick Three Points")
        self.flyplay_btn.setEnabled(True)

        self.fly_instr_label.setText("Path ready! Press Play")
        self.vtk_widget.GetRenderWindow().Render()

    def create_path_visualization(self, surface):
        """Create yellow tube showing the path"""
        pts = vtk.vtkPoints()
        lines = vtk.vtkCellArray()

        for i, pt in enumerate(self.camera_path_points):
            pts.InsertNextPoint(pt)
            if i > 0:
                line = vtk.vtkLine()
                line.GetPointIds().SetId(0, i - 1)
                line.GetPointIds().SetId(1, i)
                lines.InsertNextCell(line)

        path_poly = vtk.vtkPolyData()
        path_poly.SetPoints(pts)
        path_poly.SetLines(lines)

        if surface:
            b = surface.GetBounds()
            radius = max(np.sqrt((b[1] - b[0]) ** 2 + (b[3] - b[2]) ** 2 + (b[5] - b[4]) ** 2) * 0.003, 0.5)
        else:
            radius = 1.0

        tube = vtk.vtkTubeFilter()
        tube.SetInputData(path_poly)
        tube.SetRadius(radius)
        tube.SetNumberOfSides(20)
        tube.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tube.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1, 1, 0)
        actor.GetProperty().SetOpacity(0.95)

        if self.fly_path_actor:
            self.renderer.RemoveActor(self.fly_path_actor)

        self.fly_path_actor = actor
        self.renderer.AddActor(self.fly_path_actor)

    def _on_flyplay_pressed(self):
        """Start fly-through animation"""
        if not self.camera_path_points:
            QMessageBox.information(self, "No Path", "Please pick three points first.")
            return

        self.current_nav = "Fly-through"

        try:
            if getattr(self, "fly_path_actor", None):
                self.renderer.RemoveActor(self.fly_path_actor)
                self.fly_path_actor = None
        except Exception:
            pass

        try:
            for marker in self._picker_marker_actors:
                self.renderer.RemoveActor(marker)
            self._picker_marker_actors.clear()
        except Exception:
            pass

        for name, actor in self.dataset_actors.items():
            if actor.GetVisibility():
                actor.GetProperty().SetOpacity(1.0)

        if self.flythrough_timer.isActive():
            self.flythrough_timer.stop()
            self.restore_to_original_state()
        else:
            self.start_flythrough_animation()

    def start_flythrough_animation(self):
        """Initialize smooth fly-through with entry transition"""
        if not self.camera_path_points:
            return

        camera = self.renderer.GetActiveCamera()
        self.flythrough_original_camera_pos = list(camera.GetPosition())
        self.flythrough_original_camera_focal = list(camera.GetFocalPoint())
        self.flythrough_original_camera_view_up = list(camera.GetViewUp())

        if not self.fly_original_opacity:
            self.flythrough_original_opacity = {}
            for name, actor in self.dataset_actors.items():
                if actor.GetVisibility():
                    self.flythrough_original_opacity[name] = (
                        actor.GetProperty().GetOpacity()
                    )
            for name, actor in self.clipping_actors.items():
                if actor.GetVisibility():
                    self.flythrough_original_opacity[name] = (
                        actor.GetProperty().GetOpacity()
                    )
        else:
            self.flythrough_original_opacity = dict(self.fly_original_opacity)

        self.flythrough_original_interactor_style = self.interactor.GetInteractorStyle()
        disabled_style = vtk.vtkInteractorStyleTrackballCamera()
        disabled_style.OnLeftButtonDown = lambda: None
        disabled_style.OnMiddleButtonDown = lambda: None
        disabled_style.OnRightButtonDown = lambda: None
        disabled_style.OnMouseWheelForward = lambda: None
        disabled_style.OnMouseWheelBackward = lambda: None
        self.interactor.SetInteractorStyle(disabled_style)

        for name, actor in self.dataset_actors.items():
            if actor.GetVisibility():
                prop = actor.GetProperty()
                prop.BackfaceCullingOff()
                prop.FrontfaceCullingOff()
        for name, actor in self.clipping_actors.items():
            if actor.GetVisibility():
                prop = actor.GetProperty()
                prop.BackfaceCullingOff()
                prop.FrontfaceCullingOff()

        self.interior_clip_enabled = True

        self.flythrough_phase = "entry"
        self.entry_t = 0.0
        self.flythrough_t = 0.0
        self.flythrough_is_playing = True

        self.flythrough_timer.start(16)

    def update_flythrough_animation(self):
        """Smooth camera movement along curve"""
        camera = self.renderer.GetActiveCamera()

        if self.flythrough_phase == "entry":
            self.entry_t += self.entry_speed

            if self.entry_t >= 1.0:
                self.flythrough_phase = "main"
                self.flythrough_t = 0.0
                self.entry_t = 1.0

            t_smooth = self.entry_t * self.entry_t * (3 - 2 * self.entry_t)

            start_pos = np.array(self.flythrough_original_camera_pos)
            target_pos = np.array(self.camera_path_points[0])
            current_pos = start_pos + t_smooth * (target_pos - start_pos)

            start_focal = np.array(self.flythrough_original_camera_focal)
            target_focal = np.array(self.camera_path_points[min(5, len(self.camera_path_points) - 1)])
            current_focal = start_focal + t_smooth * (target_focal - start_focal)

            start_up = np.array(self.flythrough_original_camera_view_up)
            target_up = self.path_normals[0] if self.path_normals else np.array([0, 1, 0])
            current_up = self.slerp_vectors(start_up, target_up, t_smooth)

            camera.SetPosition(current_pos)
            camera.SetFocalPoint(current_focal)
            camera.SetViewUp(current_up)

            self.renderer.ResetCameraClippingRange()
            self.vtk_widget.GetRenderWindow().Render()

        elif self.flythrough_phase == "main":
            self.flythrough_t += self.flythrough_speed

            if self.flythrough_t >= 1.0:
                self.flythrough_timer.stop()
                self.restore_to_original_state()
                return

            n = len(self.camera_path_points)
            idx_float = self.flythrough_t * (n - 1)
            idx = int(np.floor(idx_float))
            idx = min(idx, n - 2)
            t_local = idx_float - idx

            p0 = np.array(self.camera_path_points[idx])
            p1 = np.array(self.camera_path_points[idx + 1])
            current_pos = p0 + t_local * (p1 - p0)

            tangent = np.array(self.path_tangents[idx])
            current_focal = current_pos + tangent * self.look_ahead_distance

            current_up = np.array(self.path_normals[idx]) if self.path_normals else np.array([0, 1, 0])

            camera.SetPosition(current_pos)
            camera.SetFocalPoint(current_focal)
            camera.SetViewUp(current_up)

            if self.interior_clip_enabled:
                clip_pos = current_pos + tangent * self.clip_offset
                self.interior_clip_plane.SetOrigin(clip_pos)
                self.interior_clip_plane.SetNormal(-tangent)

            self.renderer.ResetCameraClippingRange()
            self.vtk_widget.GetRenderWindow().Render()

    def slerp_vectors(self, v0, v1, t):
        """Spherical linear interpolation for smooth rotation"""
        v0_norm = v0 / np.linalg.norm(v0)
        v1_norm = v1 / np.linalg.norm(v1)

        dot = np.clip(np.dot(v0_norm, v1_norm), -1.0, 1.0)
        theta = np.arccos(dot)

        if abs(theta) < 1e-6:
            return v0_norm + t * (v1_norm - v0_norm)

        sin_theta = np.sin(theta)
        a = np.sin((1 - t) * theta) / sin_theta
        b = np.sin(t * theta) / sin_theta

        return a * v0_norm + b * v1_norm

    def restore_to_original_state(self):
        """Restore scene after fly-through"""
        self.flythrough_timer.stop()
        self.flythrough_is_playing = False
        self.flythrough_phase = "idle"
        self.interior_clip_enabled = False

        for name, opacity in self.flythrough_original_opacity.items():
            if (
                    name in self.dataset_actors
                    and self.dataset_actors[name].GetVisibility()
            ):
                self.dataset_actors[name].GetProperty().SetOpacity(opacity)
            if (
                    name in self.clipping_actors
                    and self.clipping_actors[name].GetVisibility()
            ):
                self.clipping_actors[name].GetProperty().SetOpacity(opacity)

        if self.flythrough_original_opacity:
            for name, opacity in self.flythrough_original_opacity.items():
                if (
                        name in self.dataset_actors
                        and self.dataset_actors[name].GetVisibility()
                ):
                    slider_value = int(opacity * 100)
                    self.trans_slider.setValue(slider_value)
                    self.trans_label.setText(f"Focus Depth: {slider_value}%")
                    break

        if self.flythrough_original_camera_pos is not None:
            camera = self.renderer.GetActiveCamera()
            camera.SetPosition(self.flythrough_original_camera_pos)
            camera.SetFocalPoint(self.flythrough_original_camera_focal)
            camera.SetViewUp(self.flythrough_original_camera_view_up)

        for name, actor in self.dataset_actors.items():
            if actor.GetVisibility():
                actor.GetProperty().BackfaceCullingOn()
                actor.GetProperty().FrontfaceCullingOn()
        for name, actor in self.clipping_actors.items():
            if actor.GetVisibility():
                actor.GetProperty().BackfaceCullingOn()
                actor.GetProperty().FrontfaceCullingOn()

        if self.flythrough_original_interactor_style is not None:
            self.interactor.SetInteractorStyle(
                self.flythrough_original_interactor_style
            )

        if self.fly_path_actor:
            self.fly_path_actor.SetVisibility(True)

        self.flyplay_btn.setEnabled(False)
        self.pick_points_btn.setText("Pick Three Points")
        self.picked_point_count = 0

        self.flythrough_original_opacity.clear()
        self.reset_view()
        self.vtk_widget.GetRenderWindow().Render()

    # === DATASET LOADING ===

    def load_file(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Medical Dataset", "",
            "All Supported (*.vtk *.vtp *.stl *.obj *.nii *.nii.gz);;VTK Files (*.vtk *.vtp);;STL Files (*.stl);;OBJ Files (*.obj);;NIfTI Files (*.nii *.nii.gz)"
        )
        if filename:
            self.load_dataset(filename)

    def load_directory(self):
        directory = QFileDialog.getExistingDirectory(self, "Select DICOM Directory")
        if directory:
            self.load_dataset(directory)

    def load_multiple_obj_files(self):
        filenames, _ = QFileDialog.getOpenFileNames(
            self, "Select Multiple OBJ Files for One Organ", "", "OBJ Files (*.obj)"
        )
        if filenames and len(filenames) > 0:
            QApplication.setOverrideCursor(Qt.WaitCursor)
            polydata, volume_data, dataset_type = DatasetLoader.load_multiple_obj(filenames)
            QApplication.restoreOverrideCursor()

            if polydata is None or polydata.GetNumberOfPoints() == 0:
                QMessageBox.warning(self, "Error", f"Failed to load OBJ files.\n\nSelected: {len(filenames)} files")
                return

            base_name = os.path.splitext(os.path.basename(filenames[0]))[0]
            name = f"{base_name}_combined"
            original_name = name
            counter = 1
            while name in self.loaded_datasets:
                name = f"{original_name}_{counter}"
                counter += 1

            dialog = SystemAssignDialog(name, self)
            if dialog.exec_() == QDialog.Accepted:
                assigned_system = dialog.get_selected_system()
            else:
                return

            self.loaded_datasets[name] = polydata
            self.dataset_volumes[name] = volume_data
            self.dataset_types[name] = dataset_type
            self.dataset_systems[name] = assigned_system
            part_names = [os.path.basename(f) for f in filenames]
            self.dataset_parts[name] = part_names
            color = self.system_colors.get(assigned_system, (0.8, 0.8, 0.8))
            self.dataset_colors[name] = color

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(polydata)
            mapper.ScalarVisibilityOff()
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(color)
            actor.GetProperty().SetSpecular(0.4)
            actor.GetProperty().SetSpecularPower(30)
            self.dataset_actors[name] = actor
            self.renderer.AddActor(actor)

            item = QListWidgetItem(f"[{assigned_system[:4]}] {name}")
            item.setForeground(QColor(int(color[0] * 255), int(color[1] * 255), int(color[2] * 255)))
            self.dataset_list.addItem(item)

            self.update_info_label()
            self.renderer.ResetCamera()
            self.apply_visualization_method()
            self.vtk_widget.GetRenderWindow().Render()

    def load_dataset(self, path):
        QApplication.setOverrideCursor(Qt.WaitCursor)
        polydata, volume_data, dataset_type = DatasetLoader.auto_load(path)
        QApplication.restoreOverrideCursor()

        if polydata is None or polydata.GetNumberOfPoints() == 0:
            QMessageBox.warning(self, "Error", f"Failed to load dataset from:\n{path}")
            return

        if os.path.isdir(path):
            name = os.path.basename(path)
        else:
            name = os.path.splitext(os.path.basename(path))[0]

        original_name = name
        counter = 1
        while name in self.loaded_datasets:
            name = f"{original_name}_{counter}"
            counter += 1

        dialog = SystemAssignDialog(name, self)
        if dialog.exec_() == QDialog.Accepted:
            assigned_system = dialog.get_selected_system()
        else:
            return

        self.loaded_datasets[name] = polydata
        self.dataset_volumes[name] = volume_data
        self.dataset_types[name] = dataset_type
        self.dataset_systems[name] = assigned_system
        color = self.system_colors.get(assigned_system, (0.8, 0.8, 0.8))
        self.dataset_colors[name] = color

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        mapper.ScalarVisibilityOff()
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetSpecular(0.4)
        actor.GetProperty().SetSpecularPower(30)
        self.dataset_actors[name] = actor
        self.renderer.AddActor(actor)

        item = QListWidgetItem(f"[{assigned_system[:4]}] {name}")
        item.setForeground(QColor(int(color[0] * 255), int(color[1] * 255), int(color[2] * 255)))
        self.dataset_list.addItem(item)

        self.update_info_label()
        self.renderer.ResetCamera()
        self.apply_visualization_method()
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_dataset_visibility(self):
        current_item = self.dataset_list.currentItem()
        if current_item:
            text = current_item.text()
            name = text.split("] ", 1)[1] if "] " in text else text
            if name in self.dataset_actors:
                actor = self.dataset_actors[name]
                actor.SetVisibility(not actor.GetVisibility())
                if name in self.clipping_actors:
                    self.clipping_actors[name].SetVisibility(not self.clipping_actors[name].GetVisibility())
                if name in self.curved_mpr_actors:
                    self.curved_mpr_actors[name].SetVisibility(not self.curved_mpr_actors[name].GetVisibility())
                self.vtk_widget.GetRenderWindow().Render()
                self.update_info_label()

    def remove_dataset(self):
        current_item = self.dataset_list.currentItem()
        if current_item:
            text = current_item.text()
            name = text.split("] ", 1)[1] if "] " in text else text
            if name in self.dataset_actors:
                self.renderer.RemoveActor(self.dataset_actors[name])
                del self.dataset_actors[name]
            if name in self.clipping_actors:
                self.renderer.RemoveActor(self.clipping_actors[name])
                del self.clipping_actors[name]
            if name in self.clipping_filters:
                del self.clipping_filters[name]
            if name in self.curved_mpr_actors:
                self.renderer.RemoveActor(self.curved_mpr_actors[name])
                del self.curved_mpr_actors[name]
            if name in self.loaded_datasets:
                del self.loaded_datasets[name]
            if name in self.dataset_volumes:
                del self.dataset_volumes[name]
            if name in self.dataset_types:
                del self.dataset_types[name]
            if name in self.dataset_systems:
                del self.dataset_systems[name]
            if name in self.dataset_colors:
                del self.dataset_colors[name]
            if name in self.dataset_parts:
                del self.dataset_parts[name]
            self.dataset_list.takeItem(self.dataset_list.currentRow())
            self.update_info_label()
            self.vtk_widget.GetRenderWindow().Render()

    def clear_all_datasets(self):
        if not self.loaded_datasets:
            return
        reply = QMessageBox.question(self, "Clear All", "Remove all loaded datasets?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            for actor in list(self.dataset_actors.values()):
                self.renderer.RemoveActor(actor)
            for actor in list(self.clipping_actors.values()):
                self.renderer.RemoveActor(actor)
            for actor in list(self.curved_mpr_actors.values()):
                self.renderer.RemoveActor(actor)

            # Remove preview plane
            if self.preview_plane_actor:
                self.renderer.RemoveActor(self.preview_plane_actor)
                self.preview_plane_actor = None
                self.preview_plane_src = None

            self.loaded_datasets.clear()
            self.dataset_volumes.clear()
            self.dataset_types.clear()
            self.dataset_systems.clear()
            self.dataset_actors.clear()
            self.dataset_colors.clear()
            self.clipping_filters.clear()
            self.clipping_actors.clear()
            self.curved_mpr_actors.clear()
            self.dataset_parts.clear()
            self.dataset_list.clear()
            if self.fly_path_actor:
                self.renderer.RemoveActor(self.fly_path_actor)
                self.fly_path_actor = None
            self.update_info_label()
            self.vtk_widget.GetRenderWindow().Render()

    def on_dataset_selected(self):
        self.update_info_label()

    def update_info_label(self):
        if not self.loaded_datasets:
            self.info_label.setText("No datasets loaded\n\n💡 Load data to start!")
            return
        current_item = self.dataset_list.currentItem()
        if current_item:
            text = current_item.text()
            name = text.split("] ", 1)[1] if "] " in text else text
            if name in self.loaded_datasets:
                polydata = self.loaded_datasets[name]
                dataset_type = self.dataset_types[name]
                system = self.dataset_systems[name]
                num_points = polydata.GetNumberOfPoints()
                num_cells = polydata.GetNumberOfCells()
                bounds = polydata.GetBounds()
                visible = self.dataset_actors[name].GetVisibility()
                vis_icon = "👁" if visible else "🚫"
                info = f"<b>{vis_icon} {name}</b><br>"
                info += f"<b>System:</b> {system}<br>"
                info += f"<b>Type:</b> {dataset_type}<br>"
                if name in self.dataset_parts:
                    info += f"<b>Parts:</b> {len(self.dataset_parts[name])}<br>"
                info += f"<b>Points:</b> {num_points:,}<br>"
                info += f"<b>Cells:</b> {num_cells:,}<br>"
                info += f"<b>Size:</b> {bounds[1] - bounds[0]:.1f} × {bounds[3] - bounds[2]:.1f} × {bounds[5] - bounds[4]:.1f}"
                if name in self.dataset_parts and len(self.dataset_parts[name]) > 0:
                    info += "<br><br><b>Components:</b><br>"
                    parts_to_show = self.dataset_parts[name][:5]
                    for part in parts_to_show:
                        info += f"• {part}<br>"
                    if len(self.dataset_parts[name]) > 5:
                        info += f"• ... and {len(self.dataset_parts[name]) - 5} more"
                self.info_label.setText(info)
        else:
            self.info_label.setText(f"<b>Loaded:</b> {len(self.loaded_datasets)} dataset(s)\n\nSelect one for details")

    def on_system_filter_changed(self, filter_system):
        if filter_system == "All Systems":
            for name, actor in self.dataset_actors.items():
                actor.SetVisibility(True)
        else:
            for name, actor in self.dataset_actors.items():
                system = self.dataset_systems.get(name, "")
                actor.SetVisibility(system == filter_system)
        self.current_system = filter_system if filter_system != "All Systems" else "Nervous"
        self.update_combo_display()
        self.apply_visualization_method()
        self.vtk_widget.GetRenderWindow().Render()

    def apply_visualization_method(self):
        for actor in self.clipping_actors.values():
            actor.SetVisibility(False)
        for actor in self.curved_mpr_actors.values():
            actor.SetVisibility(False)

        if self.current_viz == "Surface Rendering":
            for name, actor in self.dataset_actors.items():
                system = self.dataset_systems.get(name, "")
                filter_sys = self.system_combo.currentText()
                if filter_sys == "All Systems" or system == filter_sys:
                    actor.SetVisibility(True)
                    actor.GetProperty().SetOpacity(self.trans_slider.value() / 100.0)
                else:
                    actor.SetVisibility(False)

        elif self.current_viz == "Clipping Planes":
            for name, actor in self.dataset_actors.items():
                actor.SetVisibility(False)
            for name, polydata in self.loaded_datasets.items():
                system = self.dataset_systems.get(name, "")
                filter_sys = self.system_combo.currentText()
                if filter_sys != "All Systems" and system != filter_sys:
                    continue
                if name not in self.clipping_filters:
                    clipper = vtk.vtkClipPolyData()
                    clipper.SetInputData(polydata)
                    clipper.SetClipFunction(self.clip_plane)
                    clipper.InsideOutOff()
                    self.clipping_filters[name] = clipper
                    mapper = vtk.vtkPolyDataMapper()
                    mapper.SetInputConnection(clipper.GetOutputPort())
                    mapper.ScalarVisibilityOff()
                    actor = vtk.vtkActor()
                    actor.SetMapper(mapper)
                    color = self.dataset_colors.get(name, (0.8, 0.8, 0.8))
                    actor.GetProperty().SetColor(color)
                    actor.GetProperty().SetSpecular(0.4)
                    actor.GetProperty().SetSpecularPower(30)
                    self.clipping_actors[name] = actor
                    self.renderer.AddActor(actor)
                else:
                    clipper = self.clipping_filters[name]
                    if self.clip_inverted:
                        clipper.InsideOutOn()
                    else:
                        clipper.InsideOutOff()
                self.clipping_actors[name].SetVisibility(True)

        elif self.current_viz == "Curved MPR":
            for name, actor in self.dataset_actors.items():
                system = self.dataset_systems.get(name, "")
                filter_sys = self.system_combo.currentText()
                if filter_sys == "All Systems" or system == filter_sys:
                    actor.SetVisibility(True)
                    actor.GetProperty().SetOpacity(0.1)
                else:
                    actor.SetVisibility(False)

            for name, polydata in self.loaded_datasets.items():
                system = self.dataset_systems.get(name, "")
                filter_sys = self.system_combo.currentText()
                if filter_sys != "All Systems" and system != filter_sys:
                    continue

                if name not in self.curved_mpr_actors:
                    bounds = polydata.GetBounds()
                    center_x = (bounds[0] + bounds[1]) / 2
                    center_y = (bounds[2] + bounds[3]) / 2
                    center_z = (bounds[4] + bounds[5]) / 2
                    points = vtk.vtkPoints()
                    lines = vtk.vtkCellArray()

                    if system == "Cardiovascular":
                        n_points = 80
                        height = bounds[5] - bounds[4]
                        width = bounds[1] - bounds[0]
                        depth = bounds[3] - bounds[2]
                        for i in range(n_points):
                            t = i / (n_points - 1)
                            if t < 0.35:
                                progress = t / 0.35
                                z = bounds[4] + progress * height * 0.7
                                x = center_x - width * 0.1
                                y = center_y
                            elif t < 0.65:
                                progress = (t - 0.35) / 0.3
                                angle = progress * np.pi
                                radius = width * 0.35
                                x = center_x - width * 0.1 + radius * np.sin(angle)
                                y = center_y + depth * 0.15 * np.sin(angle)
                                z = bounds[4] + height * 0.7 + height * 0.1 * np.sin(angle * 0.5)
                            else:
                                progress = (t - 0.65) / 0.35
                                z = bounds[4] + height * 0.7 * (1 - progress * 0.4)
                                x = center_x + width * 0.25
                                y = center_y
                            points.InsertNextPoint(x, y, z)

                    elif system == "Dental":
                        n_points = 80
                        width = bounds[1] - bounds[0]
                        depth = bounds[3] - bounds[2]
                        for i in range(n_points):
                            t = i / (n_points - 1)
                            rotation_offset = np.pi / 2
                            angle = (t - 0.5) * np.pi * 1.1 + rotation_offset
                            radius = depth * 0.5
                            x = center_x + radius * np.cos(angle)
                            y = center_y
                            z = center_z + width * 0.4 + radius * 0.85 * np.sin(angle)
                            points.InsertNextPoint(x, y, z)

                    elif system == "Musculoskeletal":
                        n_points = 80
                        height = bounds[5] - bounds[4]
                        depth = bounds[3] - bounds[2]
                        for i in range(n_points):
                            t = i / (n_points - 1)
                            z = bounds[4] + t * height
                            x = center_x
                            y = center_y + depth * 0.2 * np.sin(t * 3 * np.pi)
                            points.InsertNextPoint(x, y, z)

                    else:
                        n_points = 80
                        height = bounds[5] - bounds[4]
                        width = bounds[1] - bounds[0]
                        for i in range(n_points):
                            t = i / (n_points - 1)
                            z = bounds[4] + t * height
                            x = center_x + width * 0.25 * np.sin(t * 2 * np.pi)
                            y = center_y + width * 0.15 * np.cos(t * 2 * np.pi)
                            points.InsertNextPoint(x, y, z)

                    for i in range(points.GetNumberOfPoints() - 1):
                        line = vtk.vtkLine()
                        line.GetPointIds().SetId(0, i)
                        line.GetPointIds().SetId(1, i + 1)
                        lines.InsertNextCell(line)

                    path_polydata = vtk.vtkPolyData()
                    path_polydata.SetPoints(points)
                    path_polydata.SetLines(lines)

                    tube_filter = vtk.vtkTubeFilter()
                    tube_filter.SetInputData(path_polydata)
                    if system == "Cardiovascular":
                        radius = (bounds[1] - bounds[0]) * 0.06
                    elif system == "Dental":
                        radius = (bounds[1] - bounds[0]) * 0.035
                    else:
                        radius = (bounds[1] - bounds[0]) * 0.045
                    tube_filter.SetRadius(radius)
                    tube_filter.SetNumberOfSides(32)
                    tube_filter.Update()

                    append = vtk.vtkAppendPolyData()
                    append.AddInputData(tube_filter.GetOutput())
                    step = max(1, points.GetNumberOfPoints() // 15)
                    for i in range(0, points.GetNumberOfPoints(), step):
                        pt = points.GetPoint(i)
                        disk = vtk.vtkDiskSource()
                        disk.SetInnerRadius(0)
                        if system == "Cardiovascular":
                            disk.SetOuterRadius((bounds[1] - bounds[0]) * 0.12)
                        elif system == "Dental":
                            disk.SetOuterRadius((bounds[1] - bounds[0]) * 0.08)
                        else:
                            disk.SetOuterRadius((bounds[1] - bounds[0]) * 0.10)
                        disk.SetRadialResolution(32)
                        disk.SetCircumferentialResolution(48)
                        transform = vtk.vtkTransform()
                        transform.Translate(pt[0], pt[1], pt[2])
                        if i < points.GetNumberOfPoints() - 1:
                            next_pt = points.GetPoint(i + 1)
                            dx = next_pt[0] - pt[0]
                            dy = next_pt[1] - pt[1]
                            dz = next_pt[2] - pt[2]
                            length = np.sqrt(dx * dx + dy * dy + dz * dz)
                            if length > 0:
                                dx, dy, dz = dx / length, dy / length, dz / length
                                angle_z = np.arctan2(dy, dx) * 180 / np.pi
                                angle_y = np.arctan2(dz, np.sqrt(dx * dx + dy * dy)) * 180 / np.pi
                                transform.RotateZ(angle_z)
                                transform.RotateY(-angle_y)
                        transform_filter = vtk.vtkTransformPolyDataFilter()
                        transform_filter.SetInputConnection(disk.GetOutputPort())
                        transform_filter.SetTransform(transform)
                        transform_filter.Update()
                        append.AddInputData(transform_filter.GetOutput())
                    append.Update()

                    mapper = vtk.vtkPolyDataMapper()
                    mapper.SetInputConnection(append.GetOutputPort())
                    mapper.ScalarVisibilityOff()
                    actor = vtk.vtkActor()
                    actor.SetMapper(mapper)
                    if system == "Cardiovascular":
                        actor.GetProperty().SetColor(1.0, 0.2, 0.2)
                        actor.GetProperty().SetOpacity(0.9)
                    elif system == "Dental":
                        actor.GetProperty().SetColor(1.0, 1.0, 0.3)
                        actor.GetProperty().SetOpacity(0.85)
                    elif system == "Musculoskeletal":
                        actor.GetProperty().SetColor(0.9, 0.9, 0.9)
                        actor.GetProperty().SetOpacity(0.85)
                    else:
                        actor.GetProperty().SetColor(0.3, 0.8, 1.0)
                        actor.GetProperty().SetOpacity(0.85)
                    actor.GetProperty().SetSpecular(0.6)
                    actor.GetProperty().SetSpecularPower(50)
                    actor.GetProperty().SetAmbient(0.3)
                    actor.GetProperty().SetDiffuse(0.8)
                    self.curved_mpr_actors[name] = actor
                    self.renderer.AddActor(actor)

                self.curved_mpr_actors[name].SetVisibility(True)

        self.vtk_widget.GetRenderWindow().Render()

    def on_viz_changed(self, viz):
        self.current_viz = viz
        self.apply_visualization_method()
        self.update_combo_display()

    def on_nav_changed(self, nav):
        # Store the *actual* current navigation mode before changing
        old_nav = self.current_nav

        # Special handling for "Moving Stuff"
        if nav == "Moving Stuff":
            file_to_launch = "both heart and brain.py"

            if not os.path.exists(file_to_launch):
                QMessageBox.warning(self, "File Not Found",
                                    f"Could not find file: {file_to_launch}\n\n"
                                    "Please save 'both heart and brain.py' in the same directory.")

                # Revert the combobox
                self.nav_combo.blockSignals(True)
                self.nav_combo.setCurrentText(old_nav)
                self.nav_combo.blockSignals(False)
                return  # Stop processing

            try:
                # Use sys.executable to ensure the correct python interpreter
                subprocess.Popen([sys.executable, file_to_launch])
                QMessageBox.information(self, "Launching App",
                                        "Launching the 🧠❤️ EEG/ECG Visualizer in a new window...")
            except Exception as e:
                QMessageBox.critical(self, "Launch Error", f"Failed to launch script: {e}")

            # REVERT THE COMBOBOX to the previous selection
            self.nav_combo.blockSignals(True)
            self.nav_combo.setCurrentText(old_nav)
            self.nav_combo.blockSignals(False)

            # Manually set state back
            self.current_nav = old_nav
            # No need to call apply_navigation_technique() or update_combo_display()
            # because the state hasn't *actually* changed.
            return  # We are done

        # --- Normal flow for other buttons ---
        # If we're here, a *different* nav was selected
        self.current_nav = nav
        self.apply_navigation_technique()
        self.update_combo_display()

    def apply_navigation_technique(self):
        self.animation_timer.stop()
        self.flythrough_timer.stop()

        if not self.flythrough_is_playing:
            self.flythrough_phase = "idle"

        if (
                self.current_nav != "Fly-through"
                and self.flythrough_original_interactor_style is not None
        ):
            self.interactor.SetInteractorStyle(
                self.flythrough_original_interactor_style
            )

        if self.current_nav == "Focus Navigation":
            self.trans_slider.setEnabled(True)
            self.pick_points_btn.setVisible(False)
            self.flyplay_btn.setVisible(False)
            self.fly_instr_label.setText("Use Focus Depth slider")
            if self.fly_path_actor:
                self.fly_path_actor.SetVisibility(False)

        # "Moving Stuff" elif block is removed

        elif self.current_nav == "Fly-through":
            self.trans_slider.setEnabled(True)
            if self.loaded_datasets:
                try:
                    for name, actor in self.dataset_actors.items():
                        if actor.GetVisibility():
                            actor.GetProperty().SetOpacity(1.0)
                    for name, actor in self.clipping_actors.items():
                        if actor.GetVisibility():
                            actor.GetProperty().SetOpacity(1.0)
                except Exception:
                    pass

                if self.fly_path_actor:
                    self.fly_path_actor.SetVisibility(False)

                try:
                    self.pick_points_btn.setVisible(True)
                    self.flyplay_btn.setVisible(False)
                    self.fly_instr_label.setText(
                        "Click 'Pick Three Points' to define path"
                    )
                except Exception:
                    pass

    def on_transparency_changed(self, value):
        """Handle transparency - Focus Navigation reveals internal structures"""
        self.trans_label.setText(f"Focus Depth: {value}%")

        if self.current_nav == "Focus Navigation":
            for name, actor in self.dataset_actors.items():
                if actor.GetVisibility():
                    property = actor.GetProperty()

                    if value < 50:
                        outer_opacity = 0.1 + (value / 50.0) * 0.3
                        property.SetOpacity(outer_opacity)
                        property.SetBackfaceCulling(0)
                        property.SetFrontfaceCulling(0)
                        property.SetAmbient(0.4)
                        property.SetDiffuse(0.7)
                        property.SetSpecular(0.5)
                    else:
                        outer_opacity = 0.4 + ((value - 50) / 50.0) * 0.6
                        property.SetOpacity(outer_opacity)
                        property.SetBackfaceCulling(0)
                        property.SetFrontfaceCulling(0)
                        property.SetAmbient(0.2)
                        property.SetDiffuse(0.8)
                        property.SetSpecular(0.4)

            for name, actor in self.clipping_actors.items():
                if actor.GetVisibility():
                    property = actor.GetProperty()

                    if value < 50:
                        outer_opacity = 0.1 + (value / 50.0) * 0.3
                        property.SetOpacity(outer_opacity)
                        property.SetBackfaceCulling(0)
                        property.SetFrontfaceCulling(0)
                        property.SetAmbient(0.4)
                        property.SetDiffuse(0.7)
                        property.SetSpecular(0.5)
                    else:
                        outer_opacity = 0.4 + ((value - 50) / 50.0) * 0.6
                        property.SetOpacity(outer_opacity)
                        property.SetBackfaceCulling(0)
                        property.SetFrontfaceCulling(0)
                        property.SetAmbient(0.2)
                        property.SetDiffuse(0.8)
                        property.SetSpecular(0.4)

            self.renderer.SetUseDepthPeeling(1)
            self.renderer.SetMaximumNumberOfPeels(4)
            self.renderer.SetOcclusionRatio(0.1)
        else:
            opacity = value / 100.0
            for name, actor in self.dataset_actors.items():
                if actor.GetVisibility():
                    property = actor.GetProperty()
                    property.SetOpacity(opacity)
                    property.SetBackfaceCulling(0)
                    property.SetFrontfaceCulling(0)
                    property.SetAmbient(0.2)
                    property.SetDiffuse(0.8)
                    property.SetSpecular(0.4)

            for name, actor in self.clipping_actors.items():
                if actor.GetVisibility():
                    property = actor.GetProperty()
                    property.SetOpacity(opacity)
                    property.SetBackfaceCulling(0)
                    property.SetFrontfaceCulling(0)
                    property.SetAmbient(0.2)
                    property.SetDiffuse(0.8)
                    property.SetSpecular(0.4)

            self.renderer.SetUseDepthPeeling(0)

        self.vtk_widget.GetRenderWindow().Render()

    # === NEW/MODIFIED CLIPPING METHODS ===

    def get_current_bounds(self):
        """Compute bounds of current or all visible datasets"""
        if not self.loaded_datasets:
            return (-50, 50, -50, 50, -50, 50)

        current_item = self.dataset_list.currentItem()
        if current_item:
            text = current_item.text()
            name = text.split("] ", 1)[1] if "] " in text else text
            if name in self.loaded_datasets:
                return self.loaded_datasets[name].GetBounds()

        # Otherwise compute bounds of all visible
        bounds = [float("inf"), float("-inf"), float("inf"),
                  float("-inf"), float("inf"), float("-inf")]
        for name, actor in self.dataset_actors.items():
            if actor.GetVisibility():
                b = self.loaded_datasets[name].GetBounds()
                bounds[0] = min(bounds[0], b[0])
                bounds[1] = max(bounds[1], b[1])
                bounds[2] = min(bounds[2], b[2])
                bounds[3] = max(bounds[3], b[3])
                bounds[4] = min(bounds[4], b[4])
                bounds[5] = max(bounds[5], b[5])

        if bounds[0] == float("inf"):
            return (-50, 50, -50, 50, -50, 50)
        return bounds

    def on_clipping_preview_changed(self, value):
        """Move preview plane without applying clipping"""
        if not self.loaded_datasets:
            return

        bounds = self.get_current_bounds()
        normal = self.clip_plane.GetNormal()

        # Calculate position range
        if abs(normal[0]) > 0.5:
            min_val, max_val = bounds[0], bounds[1]
        elif abs(normal[1]) > 0.5:
            min_val, max_val = bounds[2], bounds[3]
        else:
            min_val, max_val = bounds[4], bounds[5]

        if value == 0:
            position = min_val - abs(max_val - min_val) * 0.5
            self.clip_label.setText("Position: No Cut")
        else:
            t = value / 100.0
            position = min_val + t * (max_val - min_val)
            self.clip_label.setText(f"Position: {value}%")

        # Update clip plane origin
        origin = (normal[0] * position, normal[1] * position, normal[2] * position)
        self.clip_plane.SetOrigin(*origin)
        self.clip_plane.Modified()

        # Calculate plane size and center
        width = bounds[1] - bounds[0]
        height = bounds[3] - bounds[2]
        depth = bounds[5] - bounds[4]
        size = max(width, height, depth) * 1.2
        cx = (bounds[0] + bounds[1]) / 2
        cy = (bounds[2] + bounds[3]) / 2
        cz = (bounds[4] + bounds[5]) / 2

        # Create or update preview plane
        if self.preview_plane_src is None:
            self.preview_plane_src = vtk.vtkPlaneSource()
            self.preview_plane_src.SetResolution(2, 2)
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(self.preview_plane_src.GetOutputPort())
            self.preview_plane_actor = vtk.vtkActor()
            self.preview_plane_actor.SetMapper(mapper)
            self.preview_plane_actor.GetProperty().SetColor(1.0, 1.0, 0.2)  # Yellow
            self.preview_plane_actor.GetProperty().SetOpacity(0.4)
            self.preview_plane_actor.GetProperty().SetRepresentationToSurface()
            self.renderer.AddActor(self.preview_plane_actor)

        # Set plane coordinates
        self.preview_plane_src.SetOrigin(-size / 2, -size / 2, 0)
        self.preview_plane_src.SetPoint1(size / 2, -size / 2, 0)
        self.preview_plane_src.SetPoint2(-size / 2, size / 2, 0)
        self.preview_plane_src.Update()

        # Transform plane to world position
        transform = vtk.vtkTransform()
        transform.Translate(cx + origin[0], cy + origin[1], cz + origin[2])

        # Orient plane based on normal
        if tuple(normal) == (1, 0, 0):
            transform.RotateY(90)
        elif tuple(normal) == (0, 1, 0):
            transform.RotateX(-90)
        # Z-axis needs no rotation

        tfilter = vtk.vtkTransformPolyDataFilter()
        tfilter.SetInputConnection(self.preview_plane_src.GetOutputPort())
        tfilter.SetTransform(transform)
        tfilter.Update()

        self.preview_plane_actor.GetMapper().SetInputConnection(tfilter.GetOutputPort())
        self.vtk_widget.GetRenderWindow().Render()

    def apply_clipping_plane(self):
        """Apply clipping at current preview plane position"""
        if not self.loaded_datasets:
            QMessageBox.information(self, "No Data", "Please load a dataset first!")
            return

        # Remove preview plane from scene
        if self.preview_plane_actor is not None:
            self.renderer.RemoveActor(self.preview_plane_actor)
            self.preview_plane_actor = None
        if self.preview_plane_src is not None:
            self.preview_plane_src = None

        # Apply actual clipping
        for name, clipper in self.clipping_filters.items():
            if self.clip_inverted:
                clipper.InsideOutOn()
            else:
                clipper.InsideOutOff()
            clipper.Modified()
            clipper.Update()

        self.vtk_widget.GetRenderWindow().Render()
        QMessageBox.information(self, "Clipping Applied",
                                "✅ The clipping plane was applied at the current position.")

    def on_normal_changed(self, direction):
        normals = {"X-axis": (1, 0, 0), "Y-axis": (0, 1, 0), "Z-axis": (0, 0, 1)}
        self.clip_plane.SetNormal(normals[direction])
        self.clip_inverted = False
        self.clip_slider.setValue(0)
        self.clip_plane.Modified()

        # Update preview if it exists
        if self.preview_plane_actor is not None:
            self.on_clipping_preview_changed(self.clip_slider.value())

    # === END NEW/MODIFIED CLIPPING METHODS ===

    def invert_clipping_direction(self):
        self.clip_inverted = not self.clip_inverted
        for name, clipper in self.clipping_filters.items():
            if self.clip_inverted:
                clipper.InsideOutOn()
            else:
                clipper.InsideOutOff()
            clipper.Modified()
            clipper.Update()
        self.vtk_widget.GetRenderWindow().Render()
        direction = "⬅ Left side" if self.clip_inverted else "➡ Right side"
        QMessageBox.information(self, "Direction Inverted",
                                f"Now cutting from: {direction}\n\n"
                                f"{'Inverted: Showing LEFT side' if self.clip_inverted else 'Normal: Showing RIGHT side'}")

    def toggle_animation(self):
        if self.animation_timer.isActive():
            self.animation_timer.stop()
            self.play_btn.setText("▶ Play Animation")
        else:
            if not self.loaded_datasets:
                QMessageBox.information(self, "No Data", "Please load a dataset first!")
                return
            self.animation_frame = 0
            self.animation_timer.start(50)
            self.play_btn.setText("⏸ Pause Animation")

    def on_speed_changed(self, value):
        speed = value / 10.0
        self.speed_label.setText(f"Speed: {speed:.1f}x")
        if self.animation_timer.isActive() and speed > 0:
            self.animation_timer.setInterval(int(max(1, 50 / speed)))

    def update_animation(self):
        self.animation_frame += 1
        # This code will no longer run because "Moving Stuff" is handled differently
        if self.current_nav == "Moving Stuff":
            angle = self.animation_frame * 2
            for name, actor in self.dataset_actors.items():
                if actor.GetVisibility():
                    transform = vtk.vtkTransform()
                    bounds = self.loaded_datasets[name].GetBounds()
                    center = [(bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2, (bounds[4] + bounds[5]) / 2]
                    transform.Translate(center[0], center[1], center[2])
                    transform.RotateZ(angle)
                    transform.Translate(-center[0], -center[1], -center[2])
                    actor.SetUserTransform(transform)
            for name, actor in self.clipping_actors.items():
                if actor.GetVisibility():
                    transform = vtk.vtkTransform()
                    bounds = self.loaded_datasets[name].GetBounds()
                    center = [(bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2, (bounds[4] + bounds[5]) / 2]
                    transform.Translate(center[0], center[1], center[2])
                    transform.RotateZ(angle)
                    transform.Translate(-center[0], -center[1], -center[2])
                    actor.SetUserTransform(transform)
        self.vtk_widget.GetRenderWindow().Render()

    def update_combo_display(self):
        systems = ["Nervous", "Cardiovascular", "Musculoskeletal", "Dental"]
        viz_methods = ["Surface Rendering", "Clipping Planes", "Curved MPR"]
        nav_techniques = ["Focus Navigation", "Moving Stuff", "Fly-through"]
        if self.current_system in systems:
            system_idx = systems.index(self.current_system)
        else:
            system_idx = 0
        viz_idx = viz_methods.index(self.current_viz) if self.current_viz in viz_methods else 0
        nav_idx = nav_techniques.index(self.current_nav) if self.current_nav in nav_techniques else 0
        combo_number = system_idx * 9 + viz_idx * 3 + nav_idx + 1
        text = f"""<b>Combination {combo_number}/36</b><br><br>
<b>System:</b> {self.current_system}<br>
<b>Visualization:</b> {self.current_viz}<br>
<b>Navigation:</b> {self.current_nav}<br><br>
<b>Loaded:</b> {len(self.loaded_datasets)} dataset(s)"""
        self.combo_display.setText(text)

    def reset_view(self):
        if not self.loaded_datasets:
            QMessageBox.information(self, "No Data", "Please load a dataset first!")
            return
        for actor in self.dataset_actors.values():
            actor.SetUserTransform(None)
        for actor in self.clipping_actors.values():
            actor.SetUserTransform(None)
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

    def take_screenshot(self):
        if not self.loaded_datasets:
            QMessageBox.information(self, "No Data", "Please load a dataset first!")
            return
        filename, _ = QFileDialog.getSaveFileName(self, "Save Screenshot", "", "PNG Image (*.png);;JPEG Image (*.jpg)")
        if filename:
            w2if = vtk.vtkWindowToImageFilter()
            w2if.SetInput(self.vtk_widget.GetRenderWindow())
            w2if.Update()
            lower = filename.lower()
            if lower.endswith('.jpg') or lower.endswith('.jpeg'):
                writer = vtk.vtkJPEGWriter()
            else:
                writer = vtk.vtkPNGWriter()
            writer.SetFileName(filename)
            writer.SetInputConnection(w2if.GetOutputPort())
            writer.Write()
            QMessageBox.information(self, "Success", f"Screenshot saved:\n{filename}")


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    from PyQt5.QtGui import QPalette
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(45, 45, 55))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(30, 30, 40))
    palette.setColor(QPalette.AlternateBase, QColor(45, 45, 55))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(45, 45, 55))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    window = MedicalVisualization3D()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()