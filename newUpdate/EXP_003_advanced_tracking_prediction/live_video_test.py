import cv2
import argparse
import numpy as np
from pipeline import InterceptionPipeline

def main():
    parser = argparse.ArgumentParser(description="Live/Video Test for Advanced Tracking Pipeline")
    parser.add_argument('--source', default='0', help="Video source: '0' for webcam, or path to video file")
    args = parser.parse_args()

    # Determine source type
    if args.source.isdigit():
        source = int(args.source)
    else:
        source = args.source

    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Reduce latency by limiting buffer
    if not cap.isOpened():
        print(f"Error: Could not open video source {args.source}")
        return

    # Read first frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read first frame")
        cap.release()
        return
        
    frame = cv2.resize(frame, (640, 480))

    # We don't need manual ROI selection anymore since YOLO will auto-detect
    print("Auto-detecting target...")
    
    # Initialize Pipeline with a dummy box; YOLO will correct it in the first frame
    initial_bbox = [0, 0, 10, 10]
    pipeline = InterceptionPipeline()
    pipeline.init_pipeline(frame, initial_bbox)

    # Dummy interceptor position for demonstration
    interceptor_pos = [frame.shape[1] // 2, frame.shape[0] - 50]

    print("Starting pipeline... Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        frame = cv2.resize(frame, (640, 480))

        try:
            # Process frame
            results = pipeline.process_frame(frame, interceptor_pos)
            
            curr_bbox = results['bbox']
            pred_pos = results['predicted_pos']
            depth = results['depth']
            intercept_pt = results['intercept_pt']
            eta = results['time_to_intercept']

            # Visualization
            # 1. Target BBox
            p1 = (int(curr_bbox[0]), int(curr_bbox[1]))
            p2 = (int(curr_bbox[0] + curr_bbox[2]), int(curr_bbox[1] + curr_bbox[3]))
            cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
            cv2.putText(frame, "Target", (p1[0], p1[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 2. Predicted Position
            if pred_pos is not None:
                cv2.circle(frame, (int(pred_pos[0]), int(pred_pos[1])), 5, (0, 255, 255), -1)
                cv2.putText(frame, "Pred", (int(pred_pos[0])+10, int(pred_pos[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # 3. Interceptor Position
            cv2.circle(frame, (int(interceptor_pos[0]), int(interceptor_pos[1])), 8, (255, 0, 0), -1)
            cv2.putText(frame, "Interceptor", (int(interceptor_pos[0])+10, int(interceptor_pos[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # 4. Interception Point
            if intercept_pt is not None:
                cv2.circle(frame, (int(intercept_pt[0]), int(intercept_pt[1])), 8, (0, 0, 255), -1)
                cv2.putText(frame, f"Intercept ETA: {eta:.2f}s", (int(intercept_pt[0])+10, int(intercept_pt[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                # Draw line from interceptor to intercept point
                cv2.line(frame, (int(interceptor_pos[0]), int(interceptor_pos[1])), (int(intercept_pt[0]), int(intercept_pt[1])), (0, 0, 255), 1, cv2.LINE_AA)

            # 5. Depth Info
            cv2.putText(frame, f"Est. Depth: {depth:.2f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        except Exception as e:
            print(f"Pipeline error: {e}")

        # Show frame
        cv2.imshow("Tracking & Interception Pipeline", frame)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
