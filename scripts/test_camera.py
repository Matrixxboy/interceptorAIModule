import cv2
from ultralytics import YOLO
from pathlib import Path

def main():
    print("Loading custom trained YOLO model...")
    model_path = Path("models/drone_missile_best.pt")
    
    if not model_path.exists():
        print(f"Error: Could not find {model_path}")
        return

    # Load the trained model
    model = YOLO(str(model_path))

    # Initialize webcam (0 is usually the built-in laptop camera)
    print("Opening webcam...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Webcam opened! Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Run YOLO tracking on the frame to lock onto targets across frames
        # - botsort.yaml uses advanced motion and camera-compensation, which sticks much better to fast-moving drones than ByteTrack
        # - Lowered conf to 0.15 so it doesn't lose lock when the drone is temporarily blurry from moving fast
        results = model.track(frame, tracker="botsort.yaml", persist=True, conf=0.12, verbose=False)

        # Plot the predictions on the frame
        annotated_frame = results[0].plot()

        # Display the frame
        cv2.imshow("Custom Drone/Missile Detector", annotated_frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
