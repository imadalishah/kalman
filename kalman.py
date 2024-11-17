import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import pandas as pd
import seaborn as sns

class KalmanFilterAV:
    """
    Kalman Filter implementation specifically designed for Autonomous Vehicle applications.
    This class handles state estimation for vehicle positioning, velocity, and acceleration.
    """
    
    def __init__(self, dt=0.1):
        """
        Initialize Kalman Filter with state transition parameters.
        
        Args:
            dt (float): Time step between measurements (default: 0.1 seconds)
        """
        # State transition matrix (position, velocity, acceleration)
        self.A = np.array([
            [1, dt, 0.5*dt**2],
            [0, 1, dt],
            [0, 0, 1]
        ])
        
        # Measurement matrix (we only measure position)
        self.H = np.array([[1, 0, 0]])
        
        # Process noise covariance
        self.Q = np.array([
            [0.1, 0, 0],
            [0, 0.1, 0],
            [0, 0, 0.1]
        ])
        
        # Measurement noise covariance
        self.R = np.array([[1.0]])
        
        # Initial state covariance
        self.P = np.eye(3) * 1000
        
        # Initial state
        self.x = np.zeros((3, 1))

    def predict(self):
        """Predict the next state using the state transition model."""
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, measurement):
        """
        Update the state estimate using the measurement.
        
        Args:
            measurement (float): Observed position
        """
        y = measurement - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)
        return self.x

def generate_av_scenario():
    """Generate a realistic autonomous vehicle scenario with noisy GPS measurements."""
    # True trajectory (curved path representing a turn)
    t = np.linspace(0, 10, 100)
    true_x = 10 * np.cos(0.5 * t)
    true_v = -5 * np.sin(0.5 * t)
    true_a = -2.5 * np.cos(0.5 * t)
    
    # Add GPS-like noise to measurements
    noise_std = 2.0  # GPS noise standard deviation (in meters)
    measured_x = true_x + np.random.normal(0, noise_std, len(t))
    
    return t, true_x, true_v, true_a, measured_x

def plot_results(t, true_x, measured_x, estimated_x, estimated_P):
    """Create professional visualization of the results."""
    plt.figure(figsize=(15, 10))
    
    # Plot 1: Position Tracking
    plt.subplot(2, 1, 1)
    plt.plot(t, true_x, 'g-', label='True Position', linewidth=2)
    plt.plot(t, measured_x, 'r.', label='GPS Measurements', markersize=10, alpha=0.5)
    plt.plot(t, estimated_x, 'b-', label='Kalman Filter Estimate', linewidth=2)
    
    # Add uncertainty bounds
    uncertainty = 2 * np.sqrt(estimated_P)
    plt.fill_between(t, 
                    estimated_x - uncertainty,
                    estimated_x + uncertainty,
                    color='blue', alpha=0.2,
                    label='95% Confidence Interval')
    
    plt.title('Autonomous Vehicle Position Tracking', fontsize=14)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Position (m)', fontsize=12)
    plt.grid(True)
    plt.legend(fontsize=10)
    
    # Plot 2: Error Analysis
    plt.subplot(2, 1, 2)
    error = np.abs(true_x - estimated_x)
    plt.plot(t, error, 'r-', label='Estimation Error', linewidth=2)
    plt.fill_between(t, np.zeros_like(t), error, alpha=0.3, color='red')
    
    plt.title('Position Estimation Error', fontsize=14)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Error (m)', fontsize=12)
    plt.grid(True)
    plt.legend(fontsize=10)
    
    plt.tight_layout()
    plt.savefig('av_tracking_results.png')
    plt.close()

def main():
    """Main function demonstrating Kalman Filter for AV tracking."""
    # Generate scenario data
    t, true_x, true_v, true_a, measured_x = generate_av_scenario()
    
    # Initialize Kalman Filter
    kf = KalmanFilterAV(dt=0.1)
    
    # Arrays to store results
    estimated_x = np.zeros_like(t)
    estimated_P = np.zeros_like(t)
    
    # Process each measurement
    for i in range(len(t)):
        kf.predict()
        state = kf.update(measured_x[i])
        estimated_x[i] = state[0]
        estimated_P[i] = kf.P[0, 0]
    
    # Plot and save results
    plot_results(t, true_x, measured_x, estimated_x, estimated_P)
    
    # Calculate and print performance metrics
    rmse = np.sqrt(np.mean((true_x - estimated_x)**2))
    max_error = np.max(np.abs(true_x - estimated_x))
    
    print(f"Performance Metrics:")
    print(f"RMSE: {rmse:.2f} meters")
    print(f"Maximum Error: {max_error:.2f} meters")
    
    # Create/Modify files during execution:
    print("\nGenerated files:")
    print("av_tracking_results.png")

if __name__ == "__main__":
    main()
