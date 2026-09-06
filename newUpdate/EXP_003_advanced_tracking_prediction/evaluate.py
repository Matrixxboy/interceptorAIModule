import numpy as np
from pipeline import InterceptionPipeline

def evaluate_tracker():
    """
    Evaluates the pipeline on a dummy sequence.
    """
    print("Starting Evaluation of Advanced Tracking Pipeline...")
    pipeline = InterceptionPipeline()
    
    # Dummy initialization
    initial_bbox = [100, 100, 50, 50]
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    pipeline.init_pipeline(dummy_frame, initial_bbox)
    
    # Simulate a sequence of frames
    interceptor_pos = [0, 0]
    for i in range(10):
        # Target moves slightly
        simulated_bbox = [100 + i*5, 100 + i*2, 50, 50]
        # Our stub tracker returns static bbox, but we will pass dummy frame
        results = pipeline.process_frame(dummy_frame, interceptor_pos)
        
        print(f"Frame {i+1}:")
        print(f"  Target BBox: {results['bbox']}")
        print(f"  Predicted Pos: {results['predicted_pos']}")
        print(f"  Depth: {results['depth']}")
        print(f"  Intercept Point: {results['intercept_pt']} (ETA: {results['time_to_intercept']})")

if __name__ == '__main__':
    evaluate_tracker()
