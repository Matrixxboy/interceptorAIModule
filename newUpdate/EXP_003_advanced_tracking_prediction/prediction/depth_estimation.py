import numpy as np

class DepthEstimator:
    """
    Estimates target depth and distance based on bounding box size changes.
    Uses temporal smoothing to reduce noise.
    """
    def __init__(self, focal_length=1000, real_target_width=1.0, alpha=0.2):
        self.focal_length = focal_length
        self.real_target_width = real_target_width
        self.alpha = alpha # Smoothing factor for EMA
        self.smoothed_depth = None
        
    def estimate_depth(self, bbox_width):
        """
        Estimate distance to target using the pinhole camera model.
        distance = (real_width * focal_length) / pixel_width
        """
        if bbox_width <= 0:
            return self.smoothed_depth if self.smoothed_depth is not None else float('inf')
            
        raw_depth = (self.real_target_width * self.focal_length) / bbox_width
        
        # Exponential Moving Average Smoothing
        if self.smoothed_depth is None:
            self.smoothed_depth = raw_depth
        else:
            self.smoothed_depth = self.alpha * raw_depth + (1 - self.alpha) * self.smoothed_depth
            
        return self.smoothed_depth

if __name__ == '__main__':
    estimator = DepthEstimator()
    print(f"Estimated depth (w=50): {estimator.estimate_depth(50)}")
    print(f"Estimated depth (w=55): {estimator.estimate_depth(55)}")
