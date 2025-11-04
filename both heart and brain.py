"""
🧠❤️ Medical Visualization Suite - Brain (EEG) & Heart (ECG)
FULLY WORKING - Brain + Cardiac merged in one app
pip install vtk PyQt5 mne numpy scipy wfdb
"""

import sys, os, random, numpy as np, vtk, mne
from scipy.spatial import cKDTree
from scipy.signal import butter, filtfilt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QPushButton, QSlider, QFileDialog, QMessageBox, QCheckBox, QTabWidget)
from PyQt5.QtCore import Qt, QTimer
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


# ============================================================
# SHARED UTILITIES
# ============================================================

def load_poly(filepath):
    ext = os.path.splitext(filepath)[1].lower()
    if ext == ".obj":
        r = vtk.vtkOBJReader()
    elif ext in [".vtk", ".vtp"]:
        r = vtk.vtkPolyDataReader() if ext==".vtk" else vtk.vtkXMLPolyDataReader()
    elif ext == ".stl":
        r = vtk.vtkSTLReader()
    else:
        return None
    r.SetFileName(filepath)
    r.Update()
    return r.GetOutput()

def get_brain_bounds(poly):
    bounds = poly.GetBounds()
    center = np.array([(bounds[0]+bounds[1])/2, (bounds[2]+bounds[3])/2, (bounds[4]+bounds[5])/2])
    size = np.array([bounds[1]-bounds[0], bounds[3]-bounds[2], bounds[5]-bounds[4]])
    return center, size, bounds

def region_from_channel(ch):
    ch = ch.upper().strip('.').strip()
    if ch.startswith("FP") or ch.startswith("AF") or ch.startswith("F"):
        return "frontal"
    if ch.startswith("C") or ch.startswith("FC") or ch.startswith("FT"):
        return "central"
    if ch.startswith("P") or ch.startswith("CP") or ch.startswith("PO"):
        return "parietal"
    if ch.startswith("O"):
        return "occipital"
    if ch.startswith("T") or ch.startswith("TP"):
        return "temporal"
    return "misc"


# ============================================================
# BRAIN EEG FUNCTIONS
# ============================================================

def load_eeg_channels(edf_path):
    raw = mne.io.read_raw_edf(edf_path, preload=True, verbose=False)
    data = raw.get_data()
    ch_names = raw.ch_names
    eeg_map = {}
    for i, ch in enumerate(ch_names):
        sig = data[i]
        sig = sig - np.mean(sig)
        sig = sig / (np.max(np.abs(sig)) + 1e-9)
        eeg_map[ch] = sig
    return eeg_map, ch_names

def get_electrode_positions_3d(channel_names, brain_poly=None):
    cleaned_channels = {}
    for ch in channel_names:
        cleaned = ch.strip().rstrip('.')
        cleaned_channels[ch] = cleaned
    
    print(f"\n=== Electrode Mapping Diagnostics ===")
    print(f"Total channels in EEG file: {len(channel_names)}")
    
    electrode_positions = {}
    montage_names = ['standard_1005', 'standard_1020']
    
    for montage_name in montage_names:
        try:
            montage = mne.channels.make_standard_montage(montage_name)
            montage_positions = montage.get_positions()['ch_pos']
            
            montage_lookup = {}
            for montage_ch in montage_positions.keys():
                montage_lookup[montage_ch.upper()] = (montage_ch, montage_positions[montage_ch])
            
            print(f"\nTrying montage: {montage_name} ({len(montage_positions)} positions)")
            
            for original_ch, cleaned_ch in cleaned_channels.items():
                if original_ch in electrode_positions:
                    continue
                
                cleaned_upper = cleaned_ch.upper()
                if cleaned_upper in montage_lookup:
                    montage_ch, pos = montage_lookup[cleaned_upper]
                    electrode_positions[original_ch] = np.array(pos)
                    continue
                
                variations = [cleaned_ch.replace('Z', 'z'), cleaned_ch.replace('FP', 'Fp'),
                    cleaned_ch.replace('Fp', 'FP'), cleaned_ch.replace('POZ', 'POz'),
                    cleaned_ch.replace('FCZ', 'FCz'), cleaned_ch.replace('CPZ', 'CPz'),
                    cleaned_ch.replace('AFZ', 'AFz')]
                
                for var in variations:
                    var_upper = var.upper()
                    if var_upper in montage_lookup:
                        montage_ch, pos = montage_lookup[var_upper]
                        electrode_positions[original_ch] = np.array(pos)
                        break
            
            print(f"Mapped {len(electrode_positions)} channels using {montage_name}")
        except Exception as e:
            print(f"Error loading {montage_name}: {e}")
            continue
    
    print(f"\n=== Mapping Results ===")
    print(f"Successfully mapped: {len(electrode_positions)}/{len(channel_names)} channels")
    
    if len(electrode_positions) > 0:
        positions_array = np.array([pos for pos in electrode_positions.values()])
        electrode_center = np.mean(positions_array, axis=0)
        electrode_size = np.max(positions_array, axis=0) - np.min(positions_array, axis=0)
        
        print(f"\nElectrode cloud (MNE coordinates):")
        print(f"  Center: {electrode_center}")
        print(f"  Size: {electrode_size}")
        
        if brain_poly is not None:
            brain_center, brain_size, brain_bounds = get_brain_bounds(brain_poly)
            
            print(f"\nBrain mesh coordinates:")
            print(f"  Center: {brain_center}")
            print(f"  Size: {brain_size}")
            
            scale_factor = np.mean(brain_size / electrode_size) * 0.85
            
            print(f"\n=== Coordinate Alignment ===")
            print(f"  Scale factor: {scale_factor:.2f}")
            
            scaled_positions = {}
            for ch, pos in electrode_positions.items():
                centered = pos - electrode_center
                scaled = centered * scale_factor
                final_pos = scaled + brain_center
                scaled_positions[ch] = final_pos
            
            electrode_positions = scaled_positions
            
            final_array = np.array([pos for pos in electrode_positions.values()])
            print(f"\nFinal electrode positions:")
            print(f"  Center: {np.mean(final_array, axis=0)}")
            print(f"  Range: {np.min(final_array, axis=0)} to {np.max(final_array, axis=0)}")
    
    print("=" * 40 + "\n")
    return electrode_positions

def create_electrode_pathways(electrode_positions, channel_names):
    pathways = []
    
    connections = [
        ('Fp1', 'Fpz'), ('Fpz', 'Fp2'), ('Fp1', 'AF7'), ('Fp1', 'AF3'), ('Fpz', 'AFz'), ('Fp2', 'AF4'), ('Fp2', 'AF8'),
        ('AF7', 'AF3'), ('AF3', 'AFz'), ('AFz', 'AF4'), ('AF4', 'AF8'), ('AF7', 'F7'), ('AF3', 'F5'), ('AF3', 'F3'), ('AFz', 'Fz'),
        ('AF4', 'F4'), ('AF4', 'F6'), ('AF8', 'F8'), ('F7', 'F5'), ('F5', 'F3'), ('F3', 'F1'), ('F1', 'Fz'), ('Fz', 'F2'),
        ('F2', 'F4'), ('F4', 'F6'), ('F6', 'F8'), ('F7', 'FT7'), ('F5', 'FC5'), ('F3', 'FC3'), ('F1', 'FC1'), ('Fz', 'FCz'),
        ('F2', 'FC2'), ('F4', 'FC4'), ('F6', 'FC6'), ('F8', 'FT8'), ('FT7', 'FC5'), ('FC5', 'FC3'), ('FC3', 'FC1'), ('FC1', 'FCz'),
        ('FCz', 'FC2'), ('FC2', 'FC4'), ('FC4', 'FC6'), ('FC6', 'FT8'), ('FT7', 'T7'), ('FC5', 'C5'), ('FC3', 'C3'), ('FC1', 'C1'),
        ('FCz', 'Cz'), ('FC2', 'C2'), ('FC4', 'C4'), ('FC6', 'C6'), ('FT8', 'T8'), ('T7', 'C5'), ('C5', 'C3'), ('C3', 'C1'),
        ('C1', 'Cz'), ('Cz', 'C2'), ('C2', 'C4'), ('C4', 'C6'), ('C6', 'T8'), ('T7', 'TP7'), ('C5', 'CP5'), ('C3', 'CP3'),
        ('C1', 'CP1'), ('Cz', 'CPz'), ('C2', 'CP2'), ('C4', 'CP4'), ('C6', 'CP6'), ('T8', 'TP8'), ('TP7', 'CP5'), ('CP5', 'CP3'),
        ('CP3', 'CP1'), ('CP1', 'CPz'), ('CPz', 'CP2'), ('CP2', 'CP4'), ('CP4', 'CP6'), ('CP6', 'TP8'), ('TP7', 'P7'), ('CP5', 'P5'),
        ('CP3', 'P3'), ('CP1', 'P1'), ('CPz', 'Pz'), ('CP2', 'P2'), ('CP4', 'P4'), ('CP6', 'P6'), ('TP8', 'P8'), ('P7', 'P5'),
        ('P5', 'P3'), ('P3', 'P1'), ('P1', 'Pz'), ('Pz', 'P2'), ('P2', 'P4'), ('P4', 'P6'), ('P6', 'P8'), ('P7', 'PO7'), ('P5', 'PO5'),
        ('P3', 'PO3'), ('P1', 'PO1'), ('Pz', 'POz'), ('P2', 'PO2'), ('P4', 'PO4'), ('P6', 'PO6'), ('P8', 'PO8'), ('PO7', 'PO5'),
        ('PO5', 'PO3'), ('PO3', 'PO1'), ('PO1', 'POz'), ('POz', 'PO2'), ('PO2', 'PO4'), ('PO4', 'PO6'), ('PO6', 'PO8'), ('PO7', 'O1'),
        ('PO3', 'O1'), ('POz', 'Oz'), ('PO4', 'O2'), ('PO8', 'O2'), ('O1', 'Oz'), ('Oz', 'O2'), ('F7', 'F8'), ('FT7', 'FT8'),
        ('T7', 'T8'), ('TP7', 'TP8'), ('P7', 'P8'), ('PO7', 'PO8'), ('F3', 'F4'), ('C3', 'C4'), ('P3', 'P4'), ('O1', 'O2'),
        ('T3', 'C3'), ('C3', 'Cz'), ('Cz', 'C4'), ('C4', 'T4'), ('T3', 'T5'), ('T4', 'T6'), ('T5', 'P3'), ('P3', 'Pz'), ('Pz', 'P4'), ('P4', 'T6'),
    ]
    
    name_map = {'T7': 'T3', 'T3': 'T7', 'T8': 'T4', 'T4': 'T8', 'P7': 'T5', 'T5': 'P7', 'P8': 'T6', 'T6': 'P8'}
    
    cleaned_positions = {}
    for ch, pos in electrode_positions.items():
        cleaned_key = ch.strip().rstrip('.')
        cleaned_positions[cleaned_key] = pos
        cleaned_positions[ch] = pos
    
    matched_count = 0
    for ch1, ch2 in connections:
        pos1 = pos2 = None
        actual_ch1 = actual_ch2 = None
        
        for name1 in [ch1, ch1.upper(), ch1.lower(), name_map.get(ch1, ch1)]:
            if name1 in cleaned_positions:
                pos1 = cleaned_positions[name1]
                actual_ch1 = name1
                break
            for orig_ch in electrode_positions.keys():
                if orig_ch.strip().rstrip('.').upper() == name1.upper():
                    pos1 = electrode_positions[orig_ch]
                    actual_ch1 = orig_ch
                    break
            if pos1 is not None:
                break
        
        for name2 in [ch2, ch2.upper(), ch2.lower(), name_map.get(ch2, ch2)]:
            if name2 in cleaned_positions:
                pos2 = cleaned_positions[name2]
                actual_ch2 = name2
                break
            for orig_ch in electrode_positions.keys():
                if orig_ch.strip().rstrip('.').upper() == name2.upper():
                    pos2 = electrode_positions[orig_ch]
                    actual_ch2 = orig_ch
                    break
            if pos2 is not None:
                break
        
        if pos1 is not None and pos2 is not None:
            matched_count += 1
            segments = 25
            path_points = []
            
            mid = (pos1 + pos2) / 2
            direction = pos2 - pos1
            perpendicular = np.array([-direction[1], direction[0], 0])
            if np.linalg.norm(perpendicular) > 0:
                perpendicular = perpendicular / np.linalg.norm(perpendicular)
            
            control = mid + perpendicular * np.linalg.norm(direction) * 0.15
            control[2] += np.linalg.norm(direction) * 0.1
            
            for i in range(segments):
                t = i / (segments - 1)
                pt = (1-t)**2 * pos1 + 2*(1-t)*t * control + t**2 * pos2
                path_points.append(pt)
            
            pathways.append({
                "points": path_points,
                "start_ch": actual_ch1,
                "end_ch": actual_ch2,
                "start_region": region_from_channel(actual_ch1),
                "end_region": region_from_channel(actual_ch2),
                "length": segments
            })
    
    print(f"Created {len(pathways)} pathways from {len(connections)} possible connections")
    print(f"Pathway creation success rate: {matched_count}/{len(connections)} ({100*matched_count/len(connections):.1f}%)\n")
    
    return pathways


# ============================================================
# CARDIAC ECG FUNCTIONS
# ============================================================

def project_point_to_mesh_surface_simple(point, poly):
    if poly.GetPointData().GetNormals() is None:
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(poly)
        normals.ConsistencyOn()
        normals.AutoOrientNormalsOn()
        normals.Update()
        poly = normals.GetOutput()
    
    implicit_distance = vtk.vtkImplicitPolyDataDistance()
    implicit_distance.SetInput(poly)
    mesh_center, mesh_size, _ = get_brain_bounds(poly)
    current_pos = np.array(point, dtype=float)
    epsilon = np.linalg.norm(mesh_size) * 0.01
    
    for iteration in range(10):
        dist = implicit_distance.EvaluateFunction(current_pos)
        if abs(dist) < epsilon:
            break
        delta = epsilon / 10
        grad_x = implicit_distance.EvaluateFunction(current_pos + np.array([delta, 0, 0])) - dist
        grad_y = implicit_distance.EvaluateFunction(current_pos + np.array([0, delta, 0])) - dist
        grad_z = implicit_distance.EvaluateFunction(current_pos + np.array([0, 0, delta])) - dist
        gradient = np.array([grad_x, grad_y, grad_z]) / delta
        grad_norm = np.linalg.norm(gradient)
        if grad_norm > 1e-10:
            gradient = gradient / grad_norm
            step_size = abs(dist) * 0.5
            current_pos = current_pos - gradient * step_size
    
    return current_pos

def define_cardiac_sensors(heart_poly):
    print("\n=== Cardiac Sensor Positioning ===")
    mesh_center, mesh_size, _ = get_brain_bounds(heart_poly)
    
    sensor_definitions = {
        'RA_Lateral': mesh_center + np.array([mesh_size[0]*0.35, mesh_size[1]*0.15, 0]),
        'RA_Medial': mesh_center + np.array([mesh_size[0]*0.15, mesh_size[1]*0.15, 0]),
        'LA_Lateral': mesh_center + np.array([-mesh_size[0]*0.35, mesh_size[1]*0.15, 0]),
        'LA_Medial': mesh_center + np.array([-mesh_size[0]*0.15, mesh_size[1]*0.15, 0]),
        'RV_Free': mesh_center + np.array([mesh_size[0]*0.30, -mesh_size[1]*0.25, -mesh_size[2]*0.15]),
        'RV_Apex': mesh_center + np.array([mesh_size[0]*0.10, -mesh_size[1]*0.40, 0]),
        'LV_Free': mesh_center + np.array([-mesh_size[0]*0.30, -mesh_size[1]*0.25, -mesh_size[2]*0.15]),
        'LV_Apex': mesh_center + np.array([-mesh_size[0]*0.10, -mesh_size[1]*0.40, 0]),
        'Septum_Upper': mesh_center + np.array([0, mesh_size[1]*0.05, mesh_size[2]*0.15]),
        'Septum_Lower': mesh_center + np.array([0, -mesh_size[1]*0.20, 0]),
        'RVOT': mesh_center + np.array([mesh_size[0]*0.20, mesh_size[1]*0.25, mesh_size[2]*0.25]),
        'LVOT': mesh_center + np.array([-mesh_size[0]*0.20, mesh_size[1]*0.25, mesh_size[2]*0.25]),
    }
    
    sensor_positions = {}
    print(f"Projecting {len(sensor_definitions)} sensors to heart surface...")
    
    for sensor_name, pos in sensor_definitions.items():
        try:
            surface_pos = project_point_to_mesh_surface_simple(pos, heart_poly)
            sensor_positions[sensor_name] = surface_pos
            print(f"  ✓ {sensor_name}")
        except Exception as e:
            print(f"  ✗ {sensor_name}: {e}")
            sensor_positions[sensor_name] = pos
    
    print(f"✓ Positioned {len(sensor_positions)} cardiac sensors\n")
    return sensor_positions

def define_blood_flow_pathways(sensor_positions, heart_poly):
    pathways = []
    
    flow_routes = [
        ('RA_Lateral', 'RA_Medial'), ('RA_Medial', 'RV_Free'), ('RV_Free', 'RV_Apex'), ('RV_Apex', 'RVOT'),
        ('LA_Lateral', 'LA_Medial'), ('LA_Medial', 'LV_Free'), ('LV_Free', 'LV_Apex'), ('LV_Apex', 'LVOT'),
        ('RV_Free', 'Septum_Upper'), ('Septum_Upper', 'LV_Free'), ('RV_Apex', 'Septum_Lower'), ('Septum_Lower', 'LV_Apex'),
        ('RA_Medial', 'LA_Medial'), ('RV_Apex', 'LV_Apex'),
    ]
    
    for start_sensor, end_sensor in flow_routes:
        if start_sensor not in sensor_positions or end_sensor not in sensor_positions:
            continue
        
        pos1 = sensor_positions[start_sensor]
        pos2 = sensor_positions[end_sensor]
        
        segments = 30
        path_points = []
        
        mid = (pos1 + pos2) / 2
        direction = pos2 - pos1
        perpendicular = np.array([-direction[1], direction[0], 0])
        if np.linalg.norm(perpendicular) > 1e-10:
            perpendicular = perpendicular / np.linalg.norm(perpendicular)
        
        control = mid + perpendicular * np.linalg.norm(direction) * 0.15
        control[2] += np.linalg.norm(direction) * 0.1
        
        for i in range(segments):
            t = i / (segments - 1)
            pt = (1-t)**2 * pos1 + 2*(1-t)*t * control + t**2 * pos2
            path_points.append(pt)
        
        chamber_type = "systemic" if "LV" in start_sensor else "pulmonary"
        
        pathways.append({
            "points": path_points,
            "start_sensor": start_sensor,
            "end_sensor": end_sensor,
            "flow_type": chamber_type,
            "length": segments
        })
    
    print(f"Created {len(pathways)} blood flow pathways between sensors\n")
    return pathways

def load_ecg_from_file(filepath):
    try:
        import wfdb
        base_path = filepath.replace('.dat', '').replace('.hea', '')
        print(f"\n=== Loading ECG Data ===")
        print(f"File: {base_path}")
        record = wfdb.rdrecord(base_path)
        ecg_signal = record.p_signal[:, 0] if record.p_signal.shape[1] > 0 else record.p_signal
        sample_rate = record.fs
        print(f"✓ Loaded ECG successfully")
        print(f"  Duration: {len(ecg_signal) / sample_rate:.1f} seconds ({len(ecg_signal) / sample_rate / 60:.1f} minutes)")
        print(f"  Sample rate: {sample_rate} Hz")
        print(f"  Signal range: [{np.min(ecg_signal):.2f}, {np.max(ecg_signal):.2f}] mV\n")
        return ecg_signal, sample_rate
    except ImportError:
        print("ERROR: wfdb not installed. Run: pip install wfdb")
        return None, None
    except Exception as e:
        print(f"ERROR loading ECG: {e}")
        return None, None

def normalize_ecg_signal(ecg_signal):
    ecg_signal = ecg_signal - np.mean(ecg_signal)
    max_val = np.max(np.abs(ecg_signal))
    if max_val > 0:
        ecg_signal = ecg_signal / max_val
    try:
        b, a = butter(2, 0.3)
        ecg_signal = filtfilt(b, a, ecg_signal)
    except:
        pass
    return ecg_signal


# ============================================================
# BRAIN TAB
# ============================================================

class BrainTab(QWidget):
    def __init__(self):
        super().__init__()
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.05, 0.05, 0.08)
        self.vtk_widget = QVTKRenderWindowInteractor()
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.interactor.Initialize()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_animation)
        self.frame = 0

        self.brain_poly = None
        self.brain_actor = None
        self.signal_map = {}
        self.channel_names = []
        self.electrode_positions = {}
        self.neural_pathways = []
        self.particles = []
        self.pathway_actors = []
        self.electrode_actors = []

        self.init_ui()

    def init_ui(self):
        central = QWidget()
        self.setLayout(QHBoxLayout())
        self.layout().addWidget(self.vtk_widget, 3)

        ctrl = QWidget()
        v = QVBoxLayout(ctrl)
        self.layout().addWidget(ctrl, 1)
        v.addWidget(QLabel("<b>🧠 Brain</b>"))
        v.addWidget(QLabel("<i>EEG Electrode Flow</i>"))
        
        b1 = QPushButton("Load Brain Mesh")
        v.addWidget(b1)
        b1.clicked.connect(self.load_brain)
        
        b2 = QPushButton("Load EEG (.edf)")
        v.addWidget(b2)
        b2.clicked.connect(self.load_eeg)
        
        b3 = QPushButton("▶ Start Neural Flow")
        v.addWidget(b3)
        b3.clicked.connect(self.toggle)
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(20)
        self.speed_slider.setValue(8)
        v.addWidget(QLabel("EEG Speed:"))
        v.addWidget(self.speed_slider)
        
        self.flow_slider = QSlider(Qt.Horizontal)
        self.flow_slider.setMinimum(1)
        self.flow_slider.setMaximum(30)
        self.flow_slider.setValue(10)
        v.addWidget(QLabel("Flow Speed:"))
        v.addWidget(self.flow_slider)
        
        self.density_slider = QSlider(Qt.Horizontal)
        self.density_slider.setMinimum(10)
        self.density_slider.setMaximum(200)
        self.density_slider.setValue(80)
        v.addWidget(QLabel("Particle Density:"))
        v.addWidget(self.density_slider)
        self.density_slider.valueChanged.connect(self.update_particle_count)
        
        self.pathway_check = QCheckBox("Show Pathways")
        self.pathway_check.stateChanged.connect(self.toggle_pathways)
        v.addWidget(self.pathway_check)
        
        self.electrode_check = QCheckBox("Show Electrodes")
        self.electrode_check.stateChanged.connect(self.toggle_electrodes)
        v.addWidget(self.electrode_check)
        
        self.status = QLabel("Ready")
        v.addWidget(self.status)
        v.addStretch()

    def load_brain(self):
        f, _ = QFileDialog.getOpenFileName(self, "Select Brain", "", "OBJ/VTK/STL (*.obj *.vtk *.vtp *.stl)")
        if not f:
            return
        poly = load_poly(f)
        if not poly:
            QMessageBox.warning(self, "Error", "Failed to load brain mesh.")
            return
        if poly.GetPointData().GetNormals() is None:
            n = vtk.vtkPolyDataNormals()
            n.SetInputData(poly)
            n.ConsistencyOn()
            n.AutoOrientNormalsOn()
            n.Update()
            poly = n.GetOutput()
        
        self.brain_poly = poly
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.9, 0.9, 0.95)
        actor.GetProperty().SetOpacity(0.08)
        self.renderer.AddActor(actor)
        self.brain_actor = actor
        
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()
        self.status.setText("Brain mesh loaded. Load EEG to create pathways.")

    def load_eeg(self):
        if not self.brain_poly:
            QMessageBox.information(self, "Load brain first", "Please load the brain mesh first.")
            return
        f, _ = QFileDialog.getOpenFileName(self, "Select EEG", "", "EDF Files (*.edf)")
        if not f:
            return
        try:
            self.signal_map, self.channel_names = load_eeg_channels(f)
        except Exception as e:
            QMessageBox.warning(self, "EEG Error", str(e))
            return
        
        self.electrode_positions = get_electrode_positions_3d(self.channel_names, self.brain_poly)
        
        if len(self.electrode_positions) == 0:
            QMessageBox.warning(self, "No Positions", "Could not map channel names to 10-20 positions.")
            return
        
        self.neural_pathways = create_electrode_pathways(self.electrode_positions, self.channel_names)
        
        if len(self.neural_pathways) == 0:
            QMessageBox.warning(self, "No Pathways", "Could not create pathways between electrodes.")
            return
        
        self.create_particles()
        self.status.setText(f"EEG loaded: {len(self.signal_map)} channels, {len(self.neural_pathways)} pathways.")

    def create_particles(self):
        for p in self.particles:
            self.renderer.RemoveActor(p["actor"])
        self.particles.clear()
        
        if len(self.neural_pathways) == 0:
            return
        
        n_particles = self.density_slider.value()
        rng = random.Random(42)
        
        for i in range(n_particles):
            pathway = self.neural_pathways[i % len(self.neural_pathways)]
            start_pos = rng.uniform(0, 1)
            
            s = vtk.vtkSphereSource()
            s.SetRadius(1.5)
            s.SetThetaResolution(12)
            s.SetPhiResolution(12)
            s.Update()
            
            m = vtk.vtkPolyDataMapper()
            m.SetInputConnection(s.GetOutputPort())
            
            a = vtk.vtkActor()
            a.SetMapper(m)
            a.GetProperty().SetColor(0.3, 0.7, 1.0)
            a.GetProperty().SetOpacity(0.0)
            a.GetProperty().LightingOff()
            
            self.particles.append({
                "actor": a,
                "pathway": pathway,
                "position": start_pos,
                "start_ch": pathway["start_ch"],
                "end_ch": pathway["end_ch"],
                "speed": rng.uniform(0.8, 1.2)
            })
            
            self.renderer.AddActor(a)
        
        self.vtk_widget.GetRenderWindow().Render()

    def update_particle_count(self):
        if len(self.neural_pathways) > 0 and len(self.signal_map) > 0:
            self.create_particles()

    def toggle_pathways(self):
        if self.pathway_check.isChecked():
            self.draw_pathways()
        else:
            for actor in self.pathway_actors:
                self.renderer.RemoveActor(actor)
            self.pathway_actors.clear()
        self.vtk_widget.GetRenderWindow().Render()

    def draw_pathways(self):
        for actor in self.pathway_actors:
            self.renderer.RemoveActor(actor)
        self.pathway_actors.clear()
        
        for pathway in self.neural_pathways:
            points = vtk.vtkPoints()
            lines = vtk.vtkCellArray()
            
            path_pts = pathway["points"]
            for pt in path_pts:
                points.InsertNextPoint(*pt)
            
            line = vtk.vtkPolyLine()
            line.GetPointIds().SetNumberOfIds(len(path_pts))
            for i in range(len(path_pts)):
                line.GetPointIds().SetId(i, i)
            lines.InsertNextCell(line)
            
            polydata = vtk.vtkPolyData()
            polydata.SetPoints(points)
            polydata.SetLines(lines)
            
            tube = vtk.vtkTubeFilter()
            tube.SetInputData(polydata)
            tube.SetRadius(0.4)
            tube.SetNumberOfSides(8)
            tube.Update()
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(tube.GetOutputPort())
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.2, 0.5, 0.7)
            actor.GetProperty().SetOpacity(0.25)
            
            self.renderer.AddActor(actor)
            self.pathway_actors.append(actor)

    def toggle_electrodes(self):
        if self.electrode_check.isChecked():
            self.draw_electrodes()
        else:
            for actor in self.electrode_actors:
                self.renderer.RemoveActor(actor)
            self.electrode_actors.clear()
        self.vtk_widget.GetRenderWindow().Render()

    def draw_electrodes(self):
        for actor in self.electrode_actors:
            self.renderer.RemoveActor(actor)
        self.electrode_actors.clear()
        
        for ch_name, pos in self.electrode_positions.items():
            s = vtk.vtkSphereSource()
            s.SetCenter(*pos)
            s.SetRadius(2.0)
            s.Update()
            
            m = vtk.vtkPolyDataMapper()
            m.SetInputConnection(s.GetOutputPort())
            
            a = vtk.vtkActor()
            a.SetMapper(m)
            a.GetProperty().SetColor(1.0, 0.8, 0.2)
            a.GetProperty().SetOpacity(0.7)
            
            self.renderer.AddActor(a)
            self.electrode_actors.append(a)

    def toggle(self):
        if self.brain_poly is None or not self.signal_map or len(self.particles) == 0:
            QMessageBox.information(self, "Load data", "Load brain mesh and EEG first.")
            return
        if self.timer.isActive():
            self.timer.stop()
            self.status.setText("Paused")
            return
        self.timer.start(40)
        self.status.setText("Neural flow active...")

    def update_animation(self):
        self.frame += 1
        speed = self.speed_slider.value()
        flow_speed = self.flow_slider.value() / 1000.0
        
        channel_amps = {}
        for ch_name in self.channel_names:
            if ch_name in self.signal_map:
                s = self.signal_map[ch_name]
                idx = (self.frame * speed) % len(s)
                channel_amps[ch_name] = abs(s[int(idx)])
        
        for p in self.particles:
            pathway = p["pathway"]
            start_ch = p["start_ch"]
            end_ch = p["end_ch"]
            
            amp_start = channel_amps.get(start_ch, 0.0)
            amp_end = channel_amps.get(end_ch, 0.0)
            amp = (amp_start + amp_end) / 2.0
            
            base_speed = flow_speed * p["speed"]
            eeg_multiplier = 0.5 + amp * 2.5
            move_speed = base_speed * eeg_multiplier
            
            p["position"] += move_speed
            
            if p["position"] >= 1.0:
                p["position"] = 0.0
            
            path_pts = pathway["points"]
            t = p["position"]
            idx = int(t * (len(path_pts) - 1))
            idx = min(idx, len(path_pts) - 1)
            
            current_pt = path_pts[idx]
            
            a = p["actor"]
            a.SetPosition(*current_pt)
            
            c = [0.2 + amp * 0.8, 0.3 + amp * 0.5, 1.0 - amp * 0.3]
            a.GetProperty().SetColor(*c)
            
            opacity = min(0.95, 0.4 + amp * 1.2)
            a.GetProperty().SetOpacity(opacity)
            
            scale = 1.0 + amp * 0.6
            a.SetScale(scale, scale, scale)
        
        self.vtk_widget.GetRenderWindow().Render()


# ============================================================
# CARDIAC TAB
# ============================================================

class CardiacTab(QWidget):
    def __init__(self):
        super().__init__()
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.02, 0.02, 0.05)
        self.vtk_widget = QVTKRenderWindowInteractor()
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.interactor.Initialize()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_animation)
        self.frame = 0

        self.heart_poly = None
        self.heart_actor = None
        self.ecg_signal = None
        self.ecg_sample_rate = None
        self.sensor_positions = {}
        self.blood_pathways = []
        self.particles = []
        self.pathway_actors = []
        self.sensor_actors = []

        self.init_ui()

    def init_ui(self):
        central = QWidget()
        self.setLayout(QHBoxLayout())
        self.layout().addWidget(self.vtk_widget, 3)

        ctrl = QWidget()
        v = QVBoxLayout(ctrl)
        self.layout().addWidget(ctrl, 1)
        v.addWidget(QLabel("<b>❤️ Heart</b>"))
        v.addWidget(QLabel("<i>ECG Circulation</i>"))
        
        b1 = QPushButton("Load Heart Mesh")
        v.addWidget(b1)
        b1.clicked.connect(self.load_heart)
        
        b2 = QPushButton("Load ECG (.dat)")
        v.addWidget(b2)
        b2.clicked.connect(self.load_ecg)
        
        b3 = QPushButton("▶ Start Circulation")
        v.addWidget(b3)
        b3.clicked.connect(self.toggle_animation)
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(20)
        self.speed_slider.setValue(8)
        v.addWidget(QLabel("Heart Rate:"))
        v.addWidget(self.speed_slider)
        
        self.flow_slider = QSlider(Qt.Horizontal)
        self.flow_slider.setMinimum(1)
        self.flow_slider.setMaximum(30)
        self.flow_slider.setValue(10)
        v.addWidget(QLabel("Flow Speed:"))
        v.addWidget(self.flow_slider)
        
        self.density_slider = QSlider(Qt.Horizontal)
        self.density_slider.setMinimum(20)
        self.density_slider.setMaximum(300)
        self.density_slider.setValue(120)
        v.addWidget(QLabel("Blood Cells:"))
        v.addWidget(self.density_slider)
        self.density_slider.valueChanged.connect(self.update_particle_count)
        
        self.vessel_check = QCheckBox("Show Vessels")
        self.vessel_check.setChecked(True)
        self.vessel_check.stateChanged.connect(self.toggle_vessels)
        v.addWidget(self.vessel_check)
        
        self.sensor_check = QCheckBox("Show Sensors")
        self.sensor_check.setChecked(True)
        self.sensor_check.stateChanged.connect(self.toggle_sensors)
        v.addWidget(self.sensor_check)
        
        self.status = QLabel("Ready")
        v.addWidget(self.status)
        v.addStretch()

    def load_heart(self):
        f, _ = QFileDialog.getOpenFileName(self, "Select Heart Mesh", "", "OBJ/STL/VTK (*.obj *.stl *.vtk *.vtp)")
        if not f:
            return
        
        poly = load_poly(f)
        if not poly:
            QMessageBox.warning(self, "Error", "Failed to load heart mesh.")
            return
        
        if poly.GetPointData().GetNormals() is None:
            n = vtk.vtkPolyDataNormals()
            n.SetInputData(poly)
            n.ConsistencyOn()
            n.AutoOrientNormalsOn()
            n.Update()
            poly = n.GetOutput()
        
        self.heart_poly = poly
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.8, 0.2, 0.2)
        actor.GetProperty().SetOpacity(0.12)
        self.renderer.AddActor(actor)
        self.heart_actor = actor
        
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()
        self.status.setText("✓ Heart loaded.\nLoad ECG to position sensors.")

    def load_ecg(self):
        if not self.heart_poly:
            QMessageBox.information(self, "Load heart first", "Please load heart mesh first.")
            return
        f, _ = QFileDialog.getOpenFileName(self, "Select ECG File", "", "ECG Files (*.dat)")
        if not f:
            return
        
        ecg_signal, sample_rate = load_ecg_from_file(f)
        
        if ecg_signal is None:
            QMessageBox.warning(self, "Error", "Could not load ECG file.")
            return
        
        self.ecg_signal = normalize_ecg_signal(ecg_signal)
        self.ecg_sample_rate = sample_rate
        
        self.sensor_positions = define_cardiac_sensors(self.heart_poly)
        
        if len(self.sensor_positions) == 0:
            QMessageBox.warning(self, "Error", "Could not position sensors.")
            return
        
        self.blood_pathways = define_blood_flow_pathways(self.sensor_positions, self.heart_poly)
        
        if len(self.blood_pathways) == 0:
            QMessageBox.warning(self, "Error", "Could not create pathways.")
            return
        
        self.create_particles()
        self.draw_vessels()
        self.draw_sensors()
        
        self.status.setText(f"✓ ECG loaded: {len(self.sensor_positions)} sensors\n{len(self.blood_pathways)} vessels, {len(self.particles)} cells")

    def create_particles(self):
        for p in self.particles:
            self.renderer.RemoveActor(p["actor"])
        self.particles.clear()
        
        if len(self.blood_pathways) == 0:
            return
        
        n_particles = self.density_slider.value()
        rng = random.Random(42)
        
        for i in range(n_particles):
            pathway = self.blood_pathways[i % len(self.blood_pathways)]
            start_pos = rng.uniform(0, 1)
            
            s = vtk.vtkSphereSource()
            s.SetRadius(0.8)
            s.SetThetaResolution(8)
            s.SetPhiResolution(8)
            s.Update()
            
            m = vtk.vtkPolyDataMapper()
            m.SetInputConnection(s.GetOutputPort())
            
            a = vtk.vtkActor()
            a.SetMapper(m)
            a.GetProperty().SetColor(0.9, 0.1, 0.1)
            a.GetProperty().SetOpacity(0.0)
            a.GetProperty().LightingOff()
            
            self.particles.append({
                "actor": a,
                "pathway": pathway,
                "position": start_pos,
                "speed": rng.uniform(0.7, 1.3)
            })
            
            self.renderer.AddActor(a)
        
        self.vtk_widget.GetRenderWindow().Render()

    def update_particle_count(self):
        if len(self.blood_pathways) > 0 and self.ecg_signal is not None:
            self.create_particles()

    def draw_vessels(self):
        for actor in self.pathway_actors:
            self.renderer.RemoveActor(actor)
        self.pathway_actors.clear()
        
        for pathway in self.blood_pathways:
            points = vtk.vtkPoints()
            lines = vtk.vtkCellArray()
            
            path_pts = pathway["points"]
            for pt in path_pts:
                points.InsertNextPoint(*pt)
            
            line = vtk.vtkPolyLine()
            line.GetPointIds().SetNumberOfIds(len(path_pts))
            for i in range(len(path_pts)):
                line.GetPointIds().SetId(i, i)
            lines.InsertNextCell(line)
            
            polydata = vtk.vtkPolyData()
            polydata.SetPoints(points)
            polydata.SetLines(lines)
            
            tube = vtk.vtkTubeFilter()
            tube.SetInputData(polydata)
            tube.SetRadius(0.4)
            tube.SetNumberOfSides(8)
            tube.Update()
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(tube.GetOutputPort())
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.6, 0.1, 0.1)
            actor.GetProperty().SetOpacity(0.25)
            
            self.renderer.AddActor(actor)
            self.pathway_actors.append(actor)

    def toggle_vessels(self):
        if self.vessel_check.isChecked():
            self.draw_vessels()
        else:
            for actor in self.pathway_actors:
                self.renderer.RemoveActor(actor)
            self.pathway_actors.clear()
        self.vtk_widget.GetRenderWindow().Render()

    def draw_sensors(self):
        for actor in self.sensor_actors:
            self.renderer.RemoveActor(actor)
        self.sensor_actors.clear()
        
        for sensor_name, pos in self.sensor_positions.items():
            s = vtk.vtkSphereSource()
            s.SetCenter(*pos)
            s.SetRadius(1.5)
            s.Update()
            
            m = vtk.vtkPolyDataMapper()
            m.SetInputConnection(s.GetOutputPort())
            
            a = vtk.vtkActor()
            a.SetMapper(m)
            a.GetProperty().SetColor(1.0, 0.8, 0.2)
            a.GetProperty().SetOpacity(0.7)
            
            self.renderer.AddActor(a)
            self.sensor_actors.append(a)

    def toggle_sensors(self):
        if self.sensor_check.isChecked():
            self.draw_sensors()
        else:
            for actor in self.sensor_actors:
                self.renderer.RemoveActor(actor)
            self.sensor_actors.clear()
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_animation(self):
        if self.ecg_signal is None or len(self.particles) == 0:
            QMessageBox.information(self, "Load data", "Load ECG first.")
            return
        if self.timer.isActive():
            self.timer.stop()
            self.status.setText("⏸ Paused")
            return
        self.timer.start(40)
        self.status.setText("▶ Cardiac circulation active...")

    def update_animation(self):
        self.frame += 1
        speed = self.speed_slider.value()
        flow_speed = self.flow_slider.value() / 1000.0
        
        if self.ecg_signal is not None:
            ekg_idx = (self.frame * speed) % len(self.ecg_signal)
            ekg_amplitude = abs(self.ecg_signal[int(ekg_idx)])
        else:
            ekg_amplitude = 0.5
        
        for p in self.particles:
            pathway = p["pathway"]
            
            base_speed = flow_speed * p["speed"]
            cardiac_multiplier = 0.5 + ekg_amplitude * 2.0
            move_speed = base_speed * cardiac_multiplier
            
            p["position"] += move_speed
            
            if p["position"] >= 1.0:
                p["position"] = 0.0
            
            path_pts = pathway["points"]
            t = p["position"]
            idx = int(t * (len(path_pts) - 1))
            idx = min(idx, len(path_pts) - 1)
            current_pt = path_pts[idx]
            
            a = p["actor"]
            a.SetPosition(*current_pt)
            
            c = [0.9 + ekg_amplitude * 0.1, 0.1, 0.1]
            a.GetProperty().SetColor(*c)
            
            opacity = min(0.9, 0.4 + ekg_amplitude * 0.6)
            a.GetProperty().SetOpacity(opacity)
            
            scale = 1.0 + ekg_amplitude * 0.4
            a.SetScale(scale, scale, scale)
        
        self.vtk_widget.GetRenderWindow().Render()


# ============================================================
# MAIN APPLICATION
# ============================================================

class MedicalVisualizerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🧠❤️ Medical Visualization Suite")
        self.setGeometry(100, 100, 1800, 1000)
        
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        
        self.brain_tab = BrainTab()
        self.cardiac_tab = CardiacTab()
        
        self.tabs.addTab(self.brain_tab, "🧠 Brain (EEG)")
        self.tabs.addTab(self.cardiac_tab, "❤️ Heart (ECG)")


def main():
    app = QApplication(sys.argv)
    window = MedicalVisualizerApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
