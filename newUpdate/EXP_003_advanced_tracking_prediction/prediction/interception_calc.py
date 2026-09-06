import numpy as np

class InterceptionCalculator:
    """
    Calculates the interception point and required interceptor motion.
    """
    def __init__(self, interceptor_speed):
        self.interceptor_speed = interceptor_speed
        
    def calculate_interception(self, target_pos, target_vel, interceptor_pos):
        """
        Calculate interception point based on target position, velocity and interceptor position.
        Assuming constant velocity for both target and interceptor.
        """
        # Vector from interceptor to target
        dp = np.array(target_pos) - np.array(interceptor_pos)
        vt = np.array(target_vel)
        
        # Quadratic equation coefficients for time t: a*t^2 + b*t + c = 0
        a = np.dot(vt, vt) - self.interceptor_speed**2
        b = 2 * np.dot(dp, vt)
        c = np.dot(dp, dp)
        
        # Solve for t
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            return None, None # No real solution, interception not possible
            
        t1 = (-b + np.sqrt(discriminant)) / (2*a)
        t2 = (-b - np.sqrt(discriminant)) / (2*a)
        
        # We want the smallest positive time
        times = [t for t in (t1, t2) if t > 0]
        
        if not times:
            return None, None
            
        t_intercept = min(times)
        
        # Calculate interception point
        intercept_point = np.array(target_pos) + vt * t_intercept
        
        return intercept_point, t_intercept

if __name__ == '__main__':
    calc = InterceptionCalculator(interceptor_speed=50.0)
    pt, t = calc.calculate_interception([100, 100], [10, 0], [0, 0])
    print(f"Interception Point: {pt}, Time: {t}")
