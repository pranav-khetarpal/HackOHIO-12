import cv2
import numpy as np
import time
import heapq
import pyttsx3
from ultralytics import YOLO
from collections import OrderedDict, deque
from scipy.spatial import distance as dist
import os
import asyncio
import serial

# Define high-priority objects
HIGH_PRIORITY = {
    "bus": 3,
    "truck": 3,
    "car": 2,
    "bicycle": 1
}

class CentroidTracker:
    def __init__(self, max_disappeared=15):
        self.next_object_id = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        self.object_labels = {}
        self.max_disappeared = max_disappeared
        self.gone_ids = set()

    def register(self, centroid, label):
        """Register a new object with its centroid and label."""
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        self.object_labels[self.next_object_id] = label
        self.next_object_id += 1

    def deregister(self, object_id):
        """Remove object by its ID."""
        del self.objects[object_id]
        del self.disappeared[object_id]
        del self.object_labels[object_id]

    def update(self, input_centroids, input_labels):
        """Update tracked objects with new centroids and labels."""
        if len(input_centroids) == 0:
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.gone_ids.add(object_id)
                    self.deregister(object_id)
            return self.objects

        if len(self.objects) == 0:
            for i in range(len(input_centroids)):
                self.register(input_centroids[i], input_labels[i])
        else:
            object_ids = list(self.objects.keys())
            object_centroids = list(self.objects.values())

            D = dist.cdist(np.array(object_centroids), input_centroids)

            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            used_cols = set()

            for (row, col) in zip(rows, cols):
                if col in used_cols:
                    continue
                object_id = object_ids[row]
                self.objects[object_id] = input_centroids[col]
                self.disappeared[object_id] = 0
                self.object_labels[object_id] = input_labels[col]
                used_cols.add(col)

            for object_id in list(self.disappeared.keys()):
                if object_id not in used_cols:
                    self.disappeared[object_id] += 1
                    if self.disappeared[object_id] > self.max_disappeared:
                        self.gone_ids.add(object_id)
                        self.deregister(object_id)

            for col in range(D.shape[1]):
                if col not in used_cols:
                    self.register(input_centroids[col], input_labels[col])

        return self.objects

# Replace with your Arduino's serial port
arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

class Vision:

    def __init__(self, model_path="yolov8n.pt", camera_index=0, frame_width=640, frame_height=480):
        # Initialize Text-to-Speech engine
        self.engine = pyttsx3.init()
        self.last_announced = None
        self.announcement_interval = 3  # seconds
        self.last_announcement_time = time.time() - self.announcement_interval

        self.announced_ids = set()
        self.tracker = CentroidTracker()

        # Load YOLOv8 model
        self.model = YOLO(model_path)

        # Initialize webcam
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        # Initialize priority heaps
        self.low_priority_heap = []
        self.high_priority_heap = []
        self.added_to_heap = set()

        heapq.heapify(self.low_priority_heap)
        heapq.heapify(self.high_priority_heap)

        # Initialize a deque to store the last 15 announcements
        self.announcement_queue = deque(maxlen=15)

    def store_announcement(self, message):
        """Store an announcement in the deque."""
        self.announcement_queue.append(message)
        print(f"Stored announcement: {message}")





    
    def send_serial_message(self, message):
        """Send a message to the ESP32 over serial."""
        print(f"Sending to Arduino: {message}")
        arduino.write(message.encode())  # Send the message to ESP32
    
    def write_warning(self):
        # Write low and high priority warning to Arduino
        arduino.write(b'1')  # Sending '1' to indicate warning

    def read_arduino(self):
        data = arduino.readline().decode('utf-8').strip()  # Read Arduino output
        if data:
            print(f"Arduino says: {data}")
            if data == "LOW_PRIORITY":
                # Do something in response to low priority warning
                print("Low priority warning received.")
                self.display_low_priority_announcements()

                self.write_warning()
            elif data == "LAST_MESSAGE":
                self.display_last_message()
                # Do something in response to last message
                print("Last message warning received.")

                self.write_warning()

    # def check_interrupt_file(self):
    #     """Read the interrupt file and take action based on its content."""
    #     if os.path.exists("interrupt.txt"):
    #         with open("interrupt.txt", "r") as file:
    #             command = file.read().strip()
    #             if command == "LOW_PRIORITY":
    #                 print("Low priority message requested")
    #                 self.display_low_priority_announcements()
    #             elif command == "LAST_MESSAGE":
    #                 print("Last message requested")
    #                 self.display_last_message()
    #         # Clear the file after reading
    #         with open("interrupt.txt", "w") as file:
    #             file.write("")

    def display_low_priority_announcements(self):
        """Display all low-priority messages."""
        if self.low_priority_heap:
            while self.low_priority_heap:
                priority, object_id, detection = heapq.heappop(self.low_priority_heap)
                label, confidence, bbox = detection
                message = f"Low priority: {label} detected."
                self.engine.say(message)
                self.engine.runAndWait()
                self.store_announcement(message)
        else:
            print("No low-priority messages available.")


    # def display_low_priority_announcements(self):
    #     """Display all low-priority messages."""
    #     for priority, object_id, detection in self.low_priority_heap:
    #         label, confidence, bbox = detection
    #         print(f"Low Priority: {label} detected.")

    def display_last_message(self):
        """Display the last stored message."""
        if self.announcement_queue:
            last_message = self.announcement_queue[-1]
            self.engine.say(last_message)
            self.engine.runAndWait()
            print(f"Last message replayed: {last_message}")
        else:
            print("No messages available.")

    # def display_last_message(self):
    #     """Display the last stored message."""
    #     if self.announcement_queue:
    #         last_message = self.announcement_queue[-1]
    #         print(f"Last Message: {last_message}")
    #     else:
    #         print("No messages available.")

    def draw_bounding_boxes(self, frame, tracked_objects, detection_object_id_map):
        for object_id, centroid in tracked_objects.items():
            detection = detection_object_id_map.get(object_id)
            if detection:
                label, confidence, bbox = detection
                x1, y1, x2, y2 = bbox

                # Draw bounding box
                color = (0, 255, 0)  # Green color for bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # Put label with confidence
                cv2.putText(
                    frame,
                    f"{label} {confidence:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )

    async def start_detection(self):
        try:
            while True:
                start_time = time.time()
                ret, frame = self.cap.read()
                if not ret:
                    break

                results = self.model(frame, imgsz=640, device="cpu")[0]

                centroids = []
                labels = []
                detection_object_id_map = {}

                for i, box in enumerate(results.boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())
                    label = self.model.names[class_id]

                    centroid = ((x1 + x2) // 2, (y1 + y2) // 2)
                    centroids.append(centroid)
                    labels.append(label)

                    detection = (label, confidence, (int(x1), int(y1), int(x2), int(y2)))
                    detection_object_id_map[i] = detection

                tracked_objects = self.tracker.update(centroids, labels)
                self.draw_bounding_boxes(frame, tracked_objects, detection_object_id_map)

                for object_id, centroid in tracked_objects.items():
                    for idx, detection in detection_object_id_map.items():
                        label, confidence, bbox = detection
                        x1, y1, x2, y2 = bbox

                        if centroid == ((x1 + x2) // 2, (y1 + y2) // 2):
                            if object_id not in self.added_to_heap:
                                if label in HIGH_PRIORITY:
                                    current_time = time.time()
                                    if current_time - self.last_announcement_time > self.announcement_interval:
                                        message = f"{label} detected"
                                        self.engine.say(message)
                                        self.engine.runAndWait()
                                        self.store_announcement(message)
                                        self.last_announcement_time = current_time
                                    self.announced_ids.add(object_id)

                                    priority = HIGH_PRIORITY[label]
                                    heapq.heappush(self.high_priority_heap, (priority, object_id, detection))
                                else:
                                    heapq.heappush(self.low_priority_heap, (1, object_id, detection))
                            self.added_to_heap.add(object_id)

                for gone_id in self.tracker.gone_ids:
                    if gone_id in self.announced_ids:
                        label = self.tracker.object_labels.get(gone_id, "Object")
                        message = f"The {label} has left"
                        self.engine.say(message)
                        self.engine.runAndWait()
                        self.store_announcement(message)
                        self.announced_ids.remove(gone_id)

                self.tracker.gone_ids.clear()

                # self.check_interrupt_file()  # Check for button presses
                self.read_arduino()

                fps = int(1 / (time.time() - start_time))
                cv2.putText(frame, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow("YOLOv8 Real-Time Detection", frame)

                if cv2.waitKey(1) == ord("q"):
                    break

        except KeyboardInterrupt:
            pass

        finally:
            self.stop()

    def stop(self):
        # Release resources
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    vision = Vision(model_path="yolov8x.pt", camera_index=1)
    asyncio.run(vision.start_detection())