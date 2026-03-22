# 📡 Drone Communication Simulation with RIS (Reconfigurable Intelligent Surfaces)

## 📖 Description
This project provides a MATLAB-based simulation for modeling wireless communication between a drone and multiple transmitters enhanced by **Reconfigurable Intelligent Surfaces (RIS)**.  

The simulation leverages the **QuaDRiGa channel model** to evaluate how different movement patterns, speeds, and transmitter configurations affect signal propagation in a high-frequency environment (10.7 GHz).

It is designed for research and experimentation in:
- Wireless communication systems
- RIS-assisted propagation
- UAV (drone) mobility modeling

---

## ✨ Features
- 📡 Simulation of drone communication with **multiple RIS transmitters (1, 3, 5, 7)**
- 🛰️ Support for different **trajectory types**:
  - Linear
  - Zig-zag
  - Random
  - Reproducible random paths
- ⚡ Adjustable **drone speed and sampling density**
- 📊 Channel modeling using **QuaDRiGa**
- 📁 Dataset support for storing or loading simulation data
- 🔁 Flexible configuration for experimentation and research

---

## 📂 Project Structure
```

simulate_drone-main/
│── simulate_drone_RIS.m   # Main simulation script
│── dataset.zip            # Dataset (optional / pre-generated data)

````

---

## ⚙️ Requirements

Before running the project, ensure you have:

- **MATLAB** (R2020 or newer recommended)
- **QuaDRiGa Toolbox**  
  Download from: https://quadriga-channel-model.de

---

## 🚀 Installation

1. Clone or download this repository:
   ```bash
   git clone https://github.com/your-username/simulate_drone.git
   cd simulate_drone
````

2. Install and add **QuaDRiGa** to your MATLAB path:

   ```matlab
   addpath('path_to_quadriga');
   savepath;
   ```

3. (Optional) Extract dataset:

   ```bash
   unzip dataset.zip
   ```

---

## ▶️ Usage

1. Open MATLAB
2. Navigate to the project folder
3. Run the main script:

```matlab
simulate_drone_RIS
```

---

## 🔧 Configuration

You can modify parameters directly inside `simulate_drone_RIS.m`:

### 📡 Transmitters (RIS count)

```matlab
num_tx = 5; % Options: 1, 3, 5, 7
```

### 🛤️ Drone trajectory type

```matlab
track_type = 4; 
% 1 = linear
% 2 = zig-zag
% 3 = random
% 4 = same random (reproducible)
```

### ⚡ Speed values

```matlab
speed_values = [0.005, 1];
```

### 📏 Simulation parameters

* Track length: `500 m`
* Frequency: `10.7 GHz`
* Sample density and update rate configurable

---

## 🧠 How It Works

* The simulation initializes a **QuaDRiGa layout**
* Defines:

  * Drone (receiver)
  * RIS transmitters
* Generates movement tracks based on selected trajectory
* Computes:

  * Channel characteristics
  * Signal evolution over time and distance

---

## 🛠️ Tech Stack

* **MATLAB**
* **QuaDRiGa (Quasi Deterministic Radio Channel Generator)**
* Signal processing & wireless communication modeling

---

## 📊 Use Cases

* Research in **6G / RIS-assisted communication**
* UAV network optimization
* Channel modeling experiments
* Academic simulations and thesis work

---

## ⚠️ Notes

* Ensure QuaDRiGa is properly installed; otherwise, functions like:

  * `qd_simulation_parameters`
  * `qd_layout`
  * `qd_arrayant`
    will not work.
* Simulation performance depends on:

  * Number of transmitters
  * Sample density
  * Track length

---

## 👤 Author

Jacopo Simonini
