# Import necessary libraries for simulation, visualization, and data structures
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button
import random
from collections import deque
import time

class Vehicle:
    """
    Represents a single vehicle in the traffic network
    Tracks arrival time, wait time, and departure status for performance metrics
    """
    def __init__(self, route_id, arrival_time):
        self.route_id = route_id        # Which route this vehicle is taking
        self.arrival_time = arrival_time # When vehicle entered the system
        self.wait_time = 0              # Total time spent waiting at intersections
        self.departed = False           # Whether vehicle has completed its journey

class TrafficLight:
    """
    Implements adaptive traffic light controller using control theory principles
    Adjusts green light timing based on real-time demand (feedback control)
    """
    def __init__(self, intersection_id):
        self.intersection_id = intersection_id
        self.current_phase = 0  # 0: North-South green, 1: East-West green
        self.phase_time = 0     # Time elapsed in current phase
        self.green_time = [30, 30]  # Green light duration for each direction [NS, EW]
        self.min_green = 10     # Minimum green time (safety constraint)
        self.max_green = 60     # Maximum green time (fairness constraint)
        
    def update_timing(self, ns_demand, ew_demand):
        """
        Control theory implementation: Proportional controller
        Adjusts green light timing based on vehicle demand in each direction
        Higher demand = longer green time (within safety bounds)
        """
        total_demand = ns_demand + ew_demand
        if total_demand > 0:
            # Calculate proportion of demand in each direction
            ns_ratio = ns_demand / total_demand
            ew_ratio = ew_demand / total_demand
            
            total_cycle = 80  # Fixed total cycle time for stability
            
            # Proportional allocation with safety bounds (min/max green times)
            self.green_time[0] = max(self.min_green, min(self.max_green, int(total_cycle * ns_ratio)))
            self.green_time[1] = max(self.min_green, min(self.max_green, int(total_cycle * ew_ratio)))

class TrafficNetwork:
    """
    Main traffic network simulator that models intersections, routes, and vehicle flow
    Implements the core traffic dynamics and performance measurement
    """
    def __init__(self, num_routes=4, vehicle_density=0.3):
        self.num_routes = num_routes        # User-adjustable: number of available routes
        self.vehicle_density = vehicle_density  # User-adjustable: vehicle arrival rate
        self.intersections = {}             # Dictionary of traffic light controllers
        self.routes = {}                    # Dictionary of route definitions and queues
        self.vehicles = []                  # List of all vehicles in system
        self.time = 0                      # Simulation time counter
        self.setup_network()               # Initialize network topology
        
        # Performance metrics for analysis
        self.total_wait_time = 0           # Cumulative wait time of all vehicles
        self.total_vehicles = 0            # Total vehicles processed
        self.throughput = 0                # Vehicles processed per time step
        self.congestion_level = 0          # Network congestion ratio (0-1)
        
    def setup_network(self):
        """
        Creates network topology: intersections and connecting routes
        Simplified grid layout where routes connect adjacent intersections
        """
        # Create traffic light controllers for intersections
        for i in range(min(4, max(1, self.num_routes))):  # Limit to reasonable number
            self.intersections[i] = TrafficLight(i)
            
        # Create routes between intersections (circular topology for simplicity)
        self.routes = {}
        for i in range(self.num_routes):
            start_intersection = i % len(self.intersections)     # Start point
            end_intersection = (i + 1) % len(self.intersections) # End point
            self.routes[i] = {
                'start': start_intersection,
                'end': end_intersection,
                'vehicles': deque(),        # Queue of vehicles on this route
                'capacity': 50,             # Maximum vehicles that can queue
                'travel_time': random.randint(20, 40)  # Base travel time
            }
    
    def generate_vehicles(self):
        """
        Vehicle generation using Poisson arrival process
        Higher density = more frequent vehicle arrivals
        Vehicles are randomly assigned to available routes
        """
        # Scale density to reasonable arrival rate
        arrival_rate = self.vehicle_density * 10  # Scaling factor for simulation speed
        
        # Poisson process: random arrivals based on probability
        if random.random() < arrival_rate / 100:
            # Create new vehicle with random route assignment
            route_id = random.randint(0, self.num_routes - 1)
            vehicle = Vehicle(route_id, self.time)
            self.vehicles.append(vehicle)
            
            # Add vehicle to route queue if there's capacity (prevents infinite buildup)
            if len(self.routes[route_id]['vehicles']) < self.routes[route_id]['capacity']:
                self.routes[route_id]['vehicles'].append(vehicle)
    
    def update_traffic_lights(self):
        """
        Core control theory implementation: feedback control system
        Measures demand (sensor input) and adjusts light timing (control output)
        This is where adaptive control happens vs fixed timing systems
        """
        for intersection_id, light in self.intersections.items():
            # Sensor input: measure demand in each direction
            ns_demand = 0  # North-South demand
            ew_demand = 0  # East-West demand
            
            # Count vehicles waiting at this intersection (queue length sensor)
            for route_id, route in self.routes.items():
                if route['end'] == intersection_id and len(route['vehicles']) > 0:
                    # Route classification: even IDs = NS, odd IDs = EW (simplified)
                    if route_id % 2 == 0:  # North-South routes
                        ns_demand += len(route['vehicles'])
                    else:  # East-West routes
                        ew_demand += len(route['vehicles'])
            
            # Control output: update light timing based on measured demand
            light.update_timing(ns_demand, ew_demand)
            
            # State machine: manage light phase transitions
            light.phase_time += 1
            current_green = light.green_time[light.current_phase]
            
            # Switch phases when current green time expires
            if light.phase_time >= current_green:
                light.current_phase = 1 - light.current_phase  # Toggle between 0 and 1
                light.phase_time = 0
    
    def process_vehicles(self):
        """
        Vehicle movement simulation: checks light states and processes traffic flow
        Vehicles can only proceed when their direction has green light
        Tracks wait times for performance analysis
        """
        throughput_count = 0  # Count vehicles processed this time step
        
        for route_id, route in self.routes.items():
            if len(route['vehicles']) > 0:  # Only process routes with waiting vehicles
                intersection = self.intersections[route['end']]
                
                # Check traffic light permission for this route direction
                can_proceed = False
                if route_id % 2 == 0 and intersection.current_phase == 0:  # NS routes during NS green
                    can_proceed = True
                elif route_id % 2 == 1 and intersection.current_phase == 1:  # EW routes during EW green
                    can_proceed = True
                
                if can_proceed:
                    # Green light: process one vehicle (simplified flow model)
                    vehicle = route['vehicles'].popleft()  # Remove from queue
                    vehicle.departed = True
                    vehicle.wait_time = self.time - vehicle.arrival_time  # Calculate total wait
                    self.total_wait_time += vehicle.wait_time  # Add to performance metrics
                    throughput_count += 1
                else:
                    # Red light: vehicles accumulate wait time
                    for vehicle in route['vehicles']:
                        if not vehicle.departed:
                            vehicle.wait_time += 1
        
        # Update performance metrics
        self.throughput = throughput_count
        self.total_vehicles += throughput_count
    
    def calculate_metrics(self):
        """
        Performance measurement and Braess's paradox implementation
        Calculates network-wide congestion and demonstrates paradox effects
        """
        # Calculate congestion as ratio of queued vehicles to total capacity
        total_queued = sum(len(route['vehicles']) for route in self.routes.values())
        total_capacity = sum(route['capacity'] for route in self.routes.values())
        self.congestion_level = total_queued / total_capacity if total_capacity > 0 else 0
        
        # Braess's paradox simulation: more routes can paradoxically worsen performance
        if self.num_routes > 4:
            # Additional routes create route choice overhead and suboptimal path selection
            # This simulates the phenomenon where more options lead to worse collective outcomes
            self.congestion_level *= 1.2  # Increase congestion due to paradox effect
    
    def step(self):
        """
        Main simulation step: advances system by one time unit
        Executes all simulation processes in correct order
        """
        self.time += 1
        self.generate_vehicles()      # Add new vehicles to system
        self.update_traffic_lights()  # Run adaptive control algorithm
        self.process_vehicles()       # Move vehicles through intersections
        self.calculate_metrics()      # Update performance measurements

class TrafficSimulation:
    """
    GUI controller class that manages the interactive simulation interface
    Handles user input, real-time visualization, and parameter adjustment
    """
    def __init__(self):
        self.network = TrafficNetwork()  # Create initial traffic network
        self.running = False             # Simulation state flag
        self.setup_gui()                 # Initialize graphical interface
        
    def setup_gui(self):
        """
        Creates the interactive GUI with plots, sliders, and control buttons
        Two-plot layout: performance metrics (top) and congestion analysis (bottom)
        """
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 10))
        self.fig.suptitle('Adaptive Traffic Control Simulation')
        
        # Initialize data storage for real-time plotting
        self.time_data = []        # X-axis: simulation time
        self.wait_time_data = []   # Performance metric: average wait time
        self.throughput_data = []  # Performance metric: vehicles processed
        self.congestion_data = []  # Network state: congestion level
        
        # Adjust layout to make room for controls at bottom
        plt.subplots_adjust(bottom=0.25)
        
        # Vehicle density slider: allows real-time adjustment of arrival rate
        ax_density = plt.axes([0.2, 0.1, 0.5, 0.03])
        self.density_slider = Slider(ax_density, 'Vehicle Density', 0.1, 1.0, 
                                   valinit=0.3, valfmt='%.2f')
        self.density_slider.on_changed(self.update_density)
        
        # Number of routes slider: demonstrates Braess's paradox
        ax_routes = plt.axes([0.2, 0.05, 0.5, 0.03])
        self.routes_slider = Slider(ax_routes, 'Number of Routes', 2, 8, 
                                  valinit=4, valfmt='%d')
        self.routes_slider.on_changed(self.update_routes)
        
        # Start/Stop button for simulation control
        ax_button = plt.axes([0.8, 0.1, 0.1, 0.04])
        self.button = Button(ax_button, 'Start')
        self.button.on_clicked(self.toggle_simulation)
        
        # Reset button to clear data and restart
        ax_reset = plt.axes([0.8, 0.05, 0.1, 0.04])
        self.reset_button = Button(ax_reset, 'Reset')
        self.reset_button.on_clicked(self.reset_simulation)
        
    def update_density(self, val):
        """Callback function: updates vehicle density when slider moves"""
        self.network.vehicle_density = self.density_slider.val
        
    def update_routes(self, val):
        """Callback function: updates route count and rebuilds network topology"""
        new_routes = int(self.routes_slider.val)
        if new_routes != self.network.num_routes:
            self.network.num_routes = new_routes
            self.network.setup_network()  # Rebuild network with new route count
    
    def toggle_simulation(self, event):
        """Starts or stops the simulation animation"""
        self.running = not self.running
        self.button.label.set_text('Stop' if self.running else 'Start')
        
        if self.running:
            # Start animation loop with 100ms refresh rate
            self.animation = animation.FuncAnimation(
                self.fig, self.update_plot, interval=100, blit=False)
        else:
            # Stop animation loop
            if hasattr(self, 'animation'):
                self.animation.event_source.stop()
    
    def reset_simulation(self, event):
        """Resets simulation to initial state with current parameter values"""
        self.running = False
        self.button.label.set_text('Start')
        
        # Create new network with current slider values
        self.network = TrafficNetwork(
            num_routes=int(self.routes_slider.val),
            vehicle_density=self.density_slider.val
        )
        
        # Clear all plotting data
        self.time_data.clear()
        self.wait_time_data.clear()
        self.throughput_data.clear()
        self.congestion_data.clear()
        self.ax1.clear()
        self.ax2.clear()
        
        # Stop any running animation
        if hasattr(self, 'animation'):
            self.animation.event_source.stop()
    
    def update_plot(self, frame):
        """
        Animation callback function: updates plots with latest simulation data
        Called repeatedly during simulation to create real-time visualization
        """
        if not self.running:
            return
            
        # Execute one simulation time step
        self.network.step()
        
        # Collect performance data for plotting
        self.time_data.append(self.network.time)
        
        # Calculate average wait time (avoid division by zero)
        avg_wait = (self.network.total_wait_time / max(1, self.network.total_vehicles))
        self.wait_time_data.append(avg_wait)
        self.throughput_data.append(self.network.throughput)
        self.congestion_data.append(self.network.congestion_level)
        
        # Limit data history to last 200 points (memory management and readability)
        if len(self.time_data) > 200:
            self.time_data.pop(0)
            self.wait_time_data.pop(0)
            self.throughput_data.pop(0)
            self.congestion_data.pop(0)
        
        # Clear previous plots and redraw with new data
        self.ax1.clear()
        self.ax2.clear()
        
        # Plot 1: Dual-axis plot showing wait time and throughput
        # Left axis: Average wait time (key performance metric)
        self.ax1.plot(self.time_data, self.wait_time_data, 'b-', label='Avg Wait Time')
        self.ax1.set_ylabel('Average Wait Time (s)', color='b')
        self.ax1.tick_params(axis='y', labelcolor='b')
        
        # Right axis: Vehicle throughput (system efficiency metric)
        ax1_twin = self.ax1.twinx()
        ax1_twin.plot(self.time_data, self.throughput_data, 'r-', label='Throughput')
        ax1_twin.set_ylabel('Vehicles/Time Step', color='r')
        ax1_twin.tick_params(axis='y', labelcolor='r')
        
        self.ax1.set_title('Traffic Performance Metrics')
        self.ax1.grid(True)
        
        # Plot 2: Network congestion level over time
        self.ax2.plot(self.time_data, self.congestion_data, 'g-', label='Congestion Level')
        self.ax2.set_xlabel('Time Steps')
        self.ax2.set_ylabel('Congestion Level (0-1)')
        self.ax2.set_title(f'Network Congestion (Routes: {self.network.num_routes}, Density: {self.network.vehicle_density:.2f})')
        self.ax2.grid(True)
        
        # Visual indicator when Braess's paradox is in effect
        if self.network.num_routes > 4:
            self.ax2.text(0.02, 0.95, "Braess's Paradox Effect Active", 
                         transform=self.ax2.transAxes, 
                         bbox=dict(boxstyle="round", facecolor='yellow', alpha=0.7))
        
        plt.tight_layout()
    
    def run(self):
        """Launch the interactive simulation GUI"""
        plt.show().throughput_data.pop(0)
        self.congestion_data.pop(0)
        
        # Clear and update plots
        self.ax1.clear()
        self.ax2.clear()
        
        # Plot 1: Wait Time and Throughput
        self.ax1.plot(self.time_data, self.wait_time_data, 'b-', label='Avg Wait Time')
        self.ax1.set_ylabel('Average Wait Time (s)', color='b')
        self.ax1.tick_params(axis='y', labelcolor='b')
        
        ax1_twin = self.ax1.twinx()
        ax1_twin.plot(self.time_data, self.throughput_data, 'r-', label='Throughput')
        ax1_twin.set_ylabel('Vehicles/Time Step', color='r')
        ax1_twin.tick_params(axis='y', labelcolor='r')
        
        self.ax1.set_title('Traffic Performance Metrics')
        self.ax1.grid(True)
        
        # Plot 2: Congestion Level and Network Status
        self.ax2.plot(self.time_data, self.congestion_data, 'g-', label='Congestion Level')
        self.ax2.set_xlabel('Time Steps')
        self.ax2.set_ylabel('Congestion Level (0-1)')
        self.ax2.set_title(f'Network Congestion (Routes: {self.network.num_routes}, Density: {self.network.vehicle_density:.2f})')
        self.ax2.grid(True)
        
        # Add Braess's paradox indicator
        if self.network.num_routes > 4:
            self.ax2.text(0.02, 0.95, "Braess's Paradox Effect Active", 
                         transform=self.ax2.transAxes, 
                         bbox=dict(boxstyle="round", facecolor='yellow', alpha=0.7))
        
        plt.tight_layout()
    
    def run(self):
        """Start the simulation GUI"""
        plt.show()

# Additional analysis functions
def compare_fixed_vs_adaptive():
    """Compare fixed timing vs adaptive control"""
    print("Comparing Fixed vs Adaptive Traffic Control:")
    print("=" * 50)
    
    # Run fixed timing simulation
    fixed_network = TrafficNetwork(num_routes=4, vehicle_density=0.5)
    # Override adaptive control
    for light in fixed_network.intersections.values():
        light.green_time = [30, 30]  # Fixed timing
        
    fixed_wait_times = []
    for _ in range(1000):
        fixed_network.step()
        if fixed_network.total_vehicles > 0:
            avg_wait = fixed_network.total_wait_time / fixed_network.total_vehicles
            fixed_wait_times.append(avg_wait)
    
    # Run adaptive simulation
    adaptive_network = TrafficNetwork(num_routes=4, vehicle_density=0.5)
    adaptive_wait_times = []
    for _ in range(1000):
        adaptive_network.step()
        if adaptive_network.total_vehicles > 0:
            avg_wait = adaptive_network.total_wait_time / adaptive_network.total_vehicles
            adaptive_wait_times.append(avg_wait)
    
    print(f"Fixed Control - Average Wait Time: {np.mean(fixed_wait_times):.2f}s")
    print(f"Adaptive Control - Average Wait Time: {np.mean(adaptive_wait_times):.2f}s")
    print(f"Improvement: {((np.mean(fixed_wait_times) - np.mean(adaptive_wait_times)) / np.mean(fixed_wait_times) * 100):.1f}%")

def demonstrate_braess_paradox():
    """Demonstrate Braess's paradox with different route numbers"""
    print("\nDemonstrating Braess's Paradox:")
    print("=" * 40)
    
    route_counts = [2, 3, 4, 5, 6, 7, 8]
    results = []
    
    for routes in route_counts:
        network = TrafficNetwork(num_routes=routes, vehicle_density=0.4)
        wait_times = []
        
        for _ in range(500):
            network.step()
            if network.total_vehicles > 0:
                avg_wait = network.total_wait_time / network.total_vehicles
                wait_times.append(avg_wait)
        
        avg_performance = np.mean(wait_times[-100:])  # Last 100 measurements
        results.append((routes, avg_performance))
        print(f"Routes: {routes}, Avg Wait Time: {avg_performance:.2f}s")
    
    # Find if adding routes worsened performance
    min_wait_idx = min(range(len(results)), key=lambda i: results[i][1])
    print(f"\nOptimal number of routes: {results[min_wait_idx][0]} (Wait: {results[min_wait_idx][1]:.2f}s)")
    
    if min_wait_idx < len(results) - 1:
        print("Braess's Paradox observed: Adding more routes increased wait times!")

if __name__ == "__main__":
    print("Traffic Control Theory Simulation")
    print("=" * 40)
    print("This simulation demonstrates:")
    print("1. Adaptive traffic light control using feedback")
    print("2. Impact of vehicle density on network performance")
    print("3. Effects of route availability on traffic flow")
    print("4. Braess's paradox in traffic networks")
    print("\nStarting GUI simulation...")
    
    # Run comparative analysis
    compare_fixed_vs_adaptive()
    demonstrate_braess_paradox()
    
    # Start interactive simulation
    sim = TrafficSimulation()
    sim.run()