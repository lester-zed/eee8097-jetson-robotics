import cv2
from ultralytics import YOLO
# Load a lightweight, pre-trained YOLOv8 model (automatically downloads on first run)
model = YOLO("yolov8n.pt") 
# Open your Logitech C920 webcam
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret: break
    # Run YOLO detection on the frame
    results = model(frame, stream=True)
    # Visualize the bounding boxes directly on the frame
    for r in results:
        annotated_frame = r.plot()
    # Display the live AI feed
    cv2.imshow("Jetson Orin Nano - YOLOv8", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
