 # 3D Medical Visualization System

This is an advanced 3D medical visualization application built with **Python**, **PyQt5**, and **VTK**. It allows users to load, inspect, and navigate complex 3D medical models using several powerful visualization and navigation techniques.

The system features innovative navigation modes like **Focus Navigation** for "see-through" transparency and an **Enhanced Fly-Through** mode with smooth camera transitions and interior clipping.

## ✨ Key Features

* **Multi-Format Loader:** Supports a wide range of medical and 3D file formats:
    * DICOM (as a directory)
    * NIfTI (`.nii`, `.nii.gz`)
    * VTK (`.vtk`, `.vtp`)
    * STL (`.stl`)
    * OBJ (`.obj`)
* **Multi-OBJ Loading:** Can load and combine multiple `.obj` files into a single, cohesive model (e.g., for multi-part organs or musculoskeletal systems).
* **Anatomical System Assignment:** Prompts the user to assign loaded models to a system (**Nervous**, **Cardiovascular**, **Musculoskeletal**, **Dental**) for automatic coloring and filtering.
* **Focus Navigation:** A "see-through" slider that dynamically adjusts opacity, making outer layers transparent to reveal deep internal structures.
* **Enhanced Fly-Through:** Allows the user to pick three points on a model to generate a smooth, curved camera path. Features a cinematic entry transition and automatic **interior clipping** during flight to see inside the model.
* **Advanced Clipping Plane:** A non-destructive clipping tool that shows a **live preview plane**. Users can position the plane with a slider before clicking "Apply" to make the cut.
* **Curved MPR (Multi-Planar Reformatting):** Generates a custom, system-specific tube and disk visualization *inside* the model (e.g., a path following the jawline for Dental models or a vessel-like path for Cardiovascular).
* **External App Launcher:** The "Moving Stuff" navigation mode now functions as a launcher for the external **EEG/ECG Visualizer** (`both heart and brain.py`).
* **Utility Tools:** Includes standard functions like dataset management (toggle, remove, clear all), view reset, and high-resolution screenshot capture.

 # Requirements
```
pip install -r requirements.txt
```


For the "Moving Stuff" feature to work, you must have the `both heart and brain.py` script in the same directory as this application.

## 🚀 How to Use

1.  Run the application from your terminal:
    ```bash
    python "TASK 3.py"
    ```
2.  A welcome message will pop up explaining the main features.

### 1. Loading and Managing Data

* Use the buttons in the **📁 Dataset Manager** to load your data.
    * **Load File:** For single files like `.obj`, `.stl`, `.nii`, or `.vtk`.
    * **Load DICOM:** Select the *directory* containing your DICOM series.
    * **Load Multiple OBJ:** Select two or more `.obj` files that make up a single organ.
* When loading, a dialog will ask you to **assign an anatomical system**. This is important for filtering and for the Curved MPR feature.
* Your loaded model will appear in the list. You can select it to see its info, toggle its visibility, or remove it.

### 2. Exploring the Visualization Modes

You can combine different methods using the **⚙️ Control Panel** dropdowns.

#### 🧠 Focus Navigation (Default)
This is the default mode, perfect for inspecting models by "peeling" away layers.

1.  Select **"Surface Rendering"** (Visualization) and **"Focus Navigation"** (Navigation).
2.  Use the **🔍 Focus: Depth Control** slider:
    * **100%:** Shows the normal, solid outer surface.
    * **0%:** Makes the outer surface highly transparent, allowing you to see deep internal structures.

#### ✈️ Enhanced Fly-Through
This mode lets you create a cinematic flight path *through* your model.

1.  Select **"Fly-through"** from the "Navigation Technique" dropdown.
2.  Click the **"Pick Three Points"** button.
3.  In the 3D window, zoom and orient your model. Press the **'C'** key on your keyboard to pick a point on the model's surface.
4.  Repeat this two more times to define a path (3 points total).
5.  A yellow tube will appear showing the smooth, calculated path.
6.  Click the **"▶ Play"** button.
7.  The camera will smoothly animate from its current position to the start of the path and then fly through the model. The view in front of the camera will be automatically clipped so you can see the interior.

#### ✂️ Clipping Planes
This mode allows you to cut the model to see a cross-section.

1.  Select **"Clipping Planes"** from the "Visualization Method" dropdown.
2.  In the **✂️ Clipping: Preview & Apply** box, choose a "Cut Direction" (X, Y, or Z).
3.  Move the slider. A **yellow preview plane** will move across your model to show you exactly where the cut will be.
4.  Once you are happy with the position, click the **"✂ Apply Clipping"** button. The model will be permanently cut at that location.
5.  You can use **"🔄 Invert Direction"** to show the other half.

#### ➰ Curved MPR
This mode generates a custom internal path to highlight specific structures.

1.  Select **"Curved MPR"** from the "Visualization Method" dropdown.
2.  The application will automatically generate a tube-like visualization *inside* your model. The path is pre-defined based on the anatomical system (e.g., "Dental", "Cardiovascular") you chose when loading the model.
3.  The main model will become semi-transparent to highlight this internal path.

#### ❤️ Launching EEG/ECG App
The "Moving Stuff" button is now a launcher.

1.  Select **"Moving Stuff"** from the "Navigation Technique" dropdown.
2.  This will immediately launch the `both heart and brain.py` application in a new window (as long as the file is in the same directory).
3.  The visualization app's controls will revert to your previous navigation setting.
![]( https://github.com/mohamedhamdy9707-byte/3D-MEDICAL-VISUALISATION/blob/main/assets/Screenshot%202025-11-04%20180150.png )
<div align="center">
</div>

 ![](https://github.com/mohamedhamdy9707-byte/3D-MEDICAL-VISUALISATION/blob/main/assets/Screenshot%202025-11-04%20180233.png )
<div align="center">
</div>
 
 ![](https://github.com/mohamedhamdy9707-byte/3D-MEDICAL-VISUALISATION/blob/main/assets/Screenshot%202025-11-04%20180255.png) 
<div align="center">
</div>
![](https://github.com/mohamedhamdy9707-byte/3D-MEDICAL-VISUALISATION/blob/main/assets/Screenshot%202025-11-04%20180904.png)  
<div align="center">
</div>
 ![](https://github.com/mohamedhamdy9707-byte/3D-MEDICAL-VISUALISATION/blob/main/assets/Screenshot%202025-11-04%20182857.png) 
<div align="center">
</div>

# Requirements
   
  # CONTRIBUTERS
[@mhmdhamddyy](https://github.com/mohamedhamdy9707-byte) 

 

[ebrahimnas577](https://github.com/ebrahimnas577) 
# Under the Supervision of
Prof. Dr. Tamer Basha


Eng. Alaa Tarek

