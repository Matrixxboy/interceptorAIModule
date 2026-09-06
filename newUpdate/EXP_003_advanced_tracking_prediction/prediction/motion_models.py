import numpy as np

class KalmanFilterPredictor:
    """
    Standard Kalman Filter for target motion prediction (Constant Velocity Model).
    """
    def __init__(self, dt=1.0):
        self.dt = dt
        # State: [x, y, vx, vy]
        self.x = np.zeros((4, 1))
        
        # State transition matrix
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement matrix (we only measure x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Initial covariance
        self.P = np.eye(4) * 1000.
        
        # Process noise covariance (increased for faster reaction to sudden movements)
        self.Q = np.eye(4) * 1.0
        
        # Measurement noise covariance (decreased to trust measurements more)
        self.R = np.eye(2) * 5.0
        
    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x[:2].flatten()
        
    def update(self, measurement):
        z = np.array(measurement).reshape(2, 1)
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

class PolynomialPredictor:
    """
    Polynomial Regression based predictor.
    """
    def __init__(self, degree=2, history_size=10):
        self.degree = degree
        self.history_size = history_size
        self.history_x = []
        self.history_y = []
        self.history_t = []
        
    def update(self, measurement, t):
        self.history_x.append(measurement[0])
        self.history_y.append(measurement[1])
        self.history_t.append(t)
        
        if len(self.history_t) > self.history_size:
            self.history_x.pop(0)
            self.history_y.pop(0)
            self.history_t.pop(0)
            
    def predict(self, future_t):
        if len(self.history_t) < self.degree + 1:
            return None # Not enough points to fit
            
        coeffs_x = np.polyfit(self.history_t, self.history_x, self.degree)
        coeffs_y = np.polyfit(self.history_t, self.history_y, self.degree)
        
        pred_x = np.polyval(coeffs_x, future_t)
        pred_y = np.polyval(coeffs_y, future_t)
        
        return (pred_x, pred_y)

if __name__ == '__main__':
    kf = KalmanFilterPredictor()
    kf.update([10, 10])
    print(f"Kalman Prediction: {kf.predict()}")
