# Traffic Control Theory Simulation

An interactive Python simulation demonstrating adaptive traffic light control using control theory principles and exploring Braess's paradox in traffic networks.

## About

This project implements a real-time traffic simulation that compares traditional fixed-timing traffic lights with adaptive control systems. Users can adjust vehicle density and number of routes to observe how different parameters affect traffic flow and congestion.

**Key Features:**
- Adaptive traffic control using proportional feedback control
- Interactive sliders for real-time parameter adjustment  
- Live performance metrics (wait times, throughput, congestion)
- Demonstration of Braess's paradox (how more routes can worsen performance)

## Requirements

```bash
pip install numpy matplotlib
```

## Usage

```bash
python traffic_simulation.py
```

The program will display analysis results and launch an interactive GUI with:
- **Vehicle Density Slider**: Controls arrival rate of vehicles
- **Number of Routes Slider**: Adjusts network complexity
- **Start/Stop/Reset buttons**: Control simulation execution

## Controls

- Adjust sliders to change parameters in real-time
- Watch how adaptive control responds to different traffic conditions
- Observe Braess's paradox when routes exceed 4 (yellow indicator appears)
- Compare performance metrics between different configurations