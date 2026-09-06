import cv2
from trackers.siamese_tracker import SiameseTracker
from prediction.motion_models import KalmanFilterPredictor
from prediction.depth_estimation import DepthEstimator
from prediction.interception_calc import InterceptionCalculator

class InterceptionPipeline:
    def __init__(self):
        # Initialize sub-modules
        self.tracker = SiameseTracker()
        self.predictor = KalmanFilterPredictor()
        self.depth_estimator = DepthEstimator()
        self.interception_calc = InterceptionCalculator(interceptor_speed=100.0)
        
        self.frame_count = 0
        self.detect_interval = 3 # Run heavy YOLO tracker every 3 frames to maintain accuracy
        self.last_bbox = None
        self.is_initialized = False

    def init_pipeline(self, first_frame, initial_bbox):
        """
        initial_bbox: [x, y, w, h]
        """
        self.tracker.init(first_frame, initial_bbox)
        # Initialize predictor with center of bbox
        center_x = initial_bbox[0] + initial_bbox[2] / 2.0
        center_y = initial_bbox[1] + initial_bbox[3] / 2.0
        self.predictor.update([center_x, center_y])
        self.last_bbox = initial_bbox
        self.is_initialized = True

    def process_frame(self, frame, interceptor_pos):
        if not self.is_initialized:
            raise RuntimeError("Pipeline not initialized")
            
        self.frame_count += 1
        predicted_pos = self.predictor.predict()

        # 1. Track Target (Every N frames)
        if self.frame_count % self.detect_interval == 1:
            bbox = self.tracker.update(frame)
            self.last_bbox = bbox
            center_x = bbox[0] + bbox[2] / 2.0
            center_y = bbox[1] + bbox[3] / 2.0
            self.predictor.update([center_x, center_y])
        else:
            # Fast path: Use Kalman Filter prediction
            w, h = self.last_bbox[2], self.last_bbox[3]
            center_x, center_y = predicted_pos[0], predicted_pos[1]
            bbox = [int(center_x - w/2), int(center_y - h/2), w, h]
            self.last_bbox = bbox
        
        # 3. Estimate Depth
        depth = self.depth_estimator.estimate_depth(bbox[2])
        
        # 4. Calculate Interception
        # For simplicity in this 2D stub, we ignore depth in the calc, but in a real 3D system it would be used
        # We estimate target velocity from predictor state (vx, vy)
        vx = self.predictor.x[2, 0]
        vy = self.predictor.x[3, 0]
        
        intercept_pt, time_to_intercept = self.interception_calc.calculate_interception(
            target_pos=[center_x, center_y],
            target_vel=[vx, vy],
            interceptor_pos=interceptor_pos
        )
        
        return {
            'bbox': bbox,
            'predicted_pos': predicted_pos,
            'depth': depth,
            'intercept_pt': intercept_pt,
            'time_to_intercept': time_to_intercept
        }

if __name__ == '__main__':
    pipeline = InterceptionPipeline()
    print("Pipeline loaded.")
