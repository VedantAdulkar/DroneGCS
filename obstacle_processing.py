import cv2
import numpy as np
from enum import Enum
import requests
import time
import threading
from queue import Queue

class ProximityLevel(Enum):
    CLOSE = (0, 0, 255)    # Red
    MODERATE = (0, 255, 255)  # Yellow
    FAR = (0, 255, 0)     # Green

def lightweight_preprocess(frame):
    """Lightweight frame preprocessing for real-time performance"""
    # Simple contrast enhancement
    frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=0)
    
    # Basic sharpening
    kernel = np.array([[-1,-1,-1],
                      [-1, 9,-1],
                      [-1,-1,-1]])
    sharpened = cv2.filter2D(frame, -1, kernel)
    return sharpened

def calculate_proximity(contour_area, frame_area):
    """Calculate proximity based on contour area relative to frame size"""
    area_ratio = contour_area / frame_area
    if area_ratio > 0.15:
        return ProximityLevel.CLOSE
    elif area_ratio > 0.05:
        return ProximityLevel.MODERATE
    else:
        return ProximityLevel.FAR

def calculate_navigation_angle(centroid_x, frame_width, proximity_level):
    """Calculate navigation angle based on obstacle position and proximity"""
    relative_position = (centroid_x - frame_width/2) / (frame_width/2)
    base_angle = relative_position * 30
    
    if proximity_level == ProximityLevel.CLOSE:
        return base_angle * 2
    elif proximity_level == ProximityLevel.MODERATE:
        return base_angle * 1.5
    else:
        return base_angle

class StreamReader(threading.Thread):
    def __init__(self, url, frame_queue, queue_size=2):
        super().__init__()
        self.url = url
        self.frame_queue = frame_queue
        self.queue_size = queue_size
        self.stopped = False
        
    def run(self):
        stream = requests.get(self.url, stream=True)
        bytes_buffer = bytes()
        
        while not self.stopped:
            if stream is None:
                try:
                    stream = requests.get(self.url, stream=True)
                except Exception as e:
                    print(f"Connection error: {e}")
                    time.sleep(1)
                continue
                
            try:
                chunk = next(stream.iter_content(chunk_size=1024))
                if not chunk:
                    continue
                    
                bytes_buffer += chunk
                a = bytes_buffer.find(b'\xff\xd8')
                b = bytes_buffer.find(b'\xff\xd9')
                
                if a != -1 and b != -1:
                    jpg = bytes_buffer[a:b+2]
                    bytes_buffer = bytes_buffer[b+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    
                    # Keep only the most recent frame
                    while not self.frame_queue.empty():
                        self.frame_queue.get()
                    self.frame_queue.put(frame)
                    
            except Exception as e:
                print(f"Stream error: {e}")
                stream = None
                time.sleep(0.1)
    
    def stop(self):
        self.stopped = True

class ObstacleDetector:
    def __init__(self, stream_url):
        # Initialize frame queue and stream reader
        self.frame_queue = Queue(maxsize=2)
        self.stream_reader = StreamReader(stream_url, self.frame_queue)
        self.stream_reader.daemon = True
        self.stream_reader.start()
        
        # Store the latest processed frames
        self.latest_processed = None
        self.latest_edges = None
        self.latest_command = "No obstacles detected"
        
        # FPS calculation variables
        self.fps_start_time = time.time()
        self.fps_counter = 0
        self.fps = 0
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()
    
    def calculate_fps(self):
        self.fps_counter += 1
        if self.fps_counter % 30 == 0:
            current_time = time.time()
            self.fps = 30 / (current_time - self.fps_start_time)
            self.fps_start_time = current_time

    def process_frames(self):
        while True:
            try:
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    
                    # Basic preprocessing
                    processed_frame = lightweight_preprocess(frame)
                    
                    # Edge detection
                    gray = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2GRAY)
                    edges = cv2.Canny(gray, 50, 150)
                    
                    # Find contours
                    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    frame_area = frame.shape[0] * frame.shape[1]
                    frame_width = frame.shape[1]
                    
                    # Create color version of edges for display
                    edges_display = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                    
                    # Process contours on the edges display (as in your original code)
                    command = "No obstacles detected"
                    for contour in contours:
                        if cv2.contourArea(contour) < 1000:
                            continue
                            
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                        else:
                            continue
                            
                        proximity = calculate_proximity(cv2.contourArea(contour), frame_area)
                        
                        x, y, w, h = cv2.boundingRect(contour)
                        cv2.rectangle(processed_frame, (x, y), (x+w, y+h), proximity.value, 2)
                        
                        if cx < frame_width/2:
                            angle = calculate_navigation_angle(cx, frame_width, proximity)
                            command = f"Move right {abs(angle):.1f}°"
                            cv2.putText(processed_frame, command, (10, 30), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    self.latest_command = command
                    
                    # Calculate and display FPS
                    self.calculate_fps()
                    cv2.putText(processed_frame, f"FPS: {self.fps:.1f}", (10, 60),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    self.latest_processed = processed_frame
                    self.latest_edges = edges_display
                    
            except Exception as e:
                print(f"Error processing frame: {e}")
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
    
    def get_latest_frames(self):
        return self.latest_processed, self.latest_edges, self.latest_command
    
    def stop(self):
        self.stream_reader.stop()