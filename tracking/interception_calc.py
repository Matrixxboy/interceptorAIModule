import numpy as np

class InterceptionCalculator:
    """
    Calculates the interception point and required interceptor motion in 2D image space.
    """
    def __init__(self, interceptor_speed: float):
        self.interceptor_speed = interceptor_speed
        
    def calculate_interception(
        self, 
        target_pos: tuple[float, float], 
        target_vel: tuple[float, float], 
        interceptor_pos: tuple[float, float]
    ) -> tuple[tuple[float, float] | None, float | None]:
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
            
        t_intercept = float(min(times))
        
        # Calculate interception point
        intercept_point = np.array(target_pos) + vt * t_intercept
        
        return (float(intercept_point[0]), float(intercept_point[1])), t_intercept

    def calculate_interception_3d(
        self, 
        target_pos: tuple[float, float, float], 
        target_vel: tuple[float, float, float], 
        interceptor_speed_ms: float
    ) -> tuple[tuple[float, float, float] | None, float | None]:
        """
        Calculate interception point based on target 3D position, 3D velocity and interceptor speed.
        Assumes interceptor starts at origin (0,0,0).
        """
        dp = np.array(target_pos)
        vt = np.array(target_vel)
        
        # Quadratic equation coefficients for time t: a*t^2 + b*t + c = 0
        a = np.dot(vt, vt) - interceptor_speed_ms**2
        b = 2 * np.dot(dp, vt)
        c = np.dot(dp, dp)
        
        # Solve for t
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            return None, None # No real solution
            
        # Avoid division by zero
        if abs(a) < 1e-6:
            if abs(b) > 1e-6:
                t = -c / b
                if t > 0:
                    intercept_point = dp + vt * t
                    return (float(intercept_point[0]), float(intercept_point[1]), float(intercept_point[2])), t
            return None, None
            
        t1 = (-b + np.sqrt(discriminant)) / (2*a)
        t2 = (-b - np.sqrt(discriminant)) / (2*a)
        
        times = [t for t in (t1, t2) if t > 0]
        
        if not times:
            return None, None
            
        t_intercept = float(min(times))
        
        intercept_point = dp + vt * t_intercept
        return (float(intercept_point[0]), float(intercept_point[1]), float(intercept_point[2])), t_intercept
