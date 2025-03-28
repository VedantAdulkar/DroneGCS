# requirements.txt
"""
flask==2.0.1
paramiko==2.8.1
pymavlink==2.4.14
flask-socketio==5.1.1
"""

# app.py
from flask import Flask, render_template, jsonify, Response, request
from flask_socketio import SocketIO
import paramiko
from pymavlink import mavutil
import time
import threading
import requests
import logging
import cv2
import numpy as np
from obstacle_processing import ObstacleDetector
import queue

app = Flask(__name__)
socketio = SocketIO(app)

RPI_STREAM_URL = 'http://raspberrypi.local:8080/stream.mjpg'

# SSH Configuration
'''SSH_HOST = 'raspberrypi.local'
SSH_USERNAME = 'pi'
SSH_PASSWORD = 'pi'''

class SuppressTelemetryLogFilter(logging.Filter):
    def __init__(self):
        self.start_time = time.time()
        self.suppress = False
        print("Starting filter")

    def filter(self, record):
        elapsed_time = time.time() - self.start_time
        
        # Allow logs for 2s, then suppress for 45s
        if elapsed_time > 45 + 2:
            self.start_time = time.time()  # Reset cycle

        self.suppress = elapsed_time < 45  # Suppress before 45s

        return not (self.suppress and "/telemetrystatus" in record.getMessage())

# Apply log filter to suppress "/telemetrystatus" logs dynamically
log = logging.getLogger('werkzeug')
log.addFilter(SuppressTelemetryLogFilter())

#detector = ObstacleDetector(RPI_STREAM_URL)

def generate_frames(frame_type):
    while True:
        processed, edges, _ = detector.get_latest_frames()
        
        if frame_type == 'processed' and processed is not None:
            frame = processed
        elif frame_type == 'edges' and edges is not None:
            frame = edges
        else:
            # If frames are not available yet, yield an empty frame
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "Waiting for stream...", (50, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Convert frame to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS

class DroneController:
    def __init__(self):
        self.current_throttle = 0
        self.ssh_client = None
        self.mavlink_connection = None
        self.isRPIconnected = False
        self.is_connected = False
        self.armed = False
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.telemetry_data = {
            "altitude": "N/A",
            "battery": "N/A",
            "satellites": "N/A",
            "speed": "N/A"
        }

    def connect_ssh(self, hostname, username, password):
        try:
            self.ssh_client.connect(hostname, username=username, password=password)
            self.isRPIconnected = True
            return True, "SSH connection successful"
        except Exception as e:
            print(f"SSH Connection Error: {str(e)}")
            self.isRPIconnected = False
            return False, str(e)

    def connect_mavlink(self):
        try:
            # Connect to MAVLink
            self.master = mavutil.mavlink_connection("udp:0.0.0.0:14550")
            print("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            print("Connected!")

            self.is_connected = True
            self.mavlink_connection = self.master

            self.update_thread = threading.Thread(target=self.update_telemetry)
            self.update_thread.daemon = True
            self.update_thread.start()

            return True
        except Exception as e:
            print(f"\n MAVLink Connection Error: {str(e)}\n")
            return False
        
    def update_telemetry(self):
        while self.is_connected:
            # Non-blocking receive with short timeout
            msg = self.mavlink_connection.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'VFR_HUD':
                    self.telemetry_data["altitude"] = f"{msg.alt:.1f} m"
                    self.telemetry_data["speed"] = f"{msg.groundspeed:.1f} m/s"
                elif msg_type == 'SYS_STATUS':
                    self.telemetry_data["battery"] = f"{msg.battery_remaining}%"
                elif msg_type == 'GPS_RAW_INT':
                    self.telemetry_data["satellites"] = str(msg.satellites_visible)
            
            # Sleep briefly to avoid CPU hogging
            time.sleep(0.01)  # 10 milliseconds

    def arm_motors(self):
        if not self.is_connected:
            return False
        try:
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
            self.armed = True
            return True
        except Exception as e:
            print(f"Arming Error: {str(e)}")
            return False

    def disarm_motors(self):
        if not self.is_connected:
            return False
        try:
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0)
            self.armed = False
            return True
        except Exception as e:
            print(f"Disarming Error: {str(e)}")
            return False

    def test_motor(self, motor_number, throttle):
        try:
            # Send motor test command
            spincommand = "motortest "+str(motor_number)+" 0 "+str(throttle)+" 5"
            self.ssh_client.exec_command(f"screen -S mavproxy -X stuff '{spincommand}\\n'")
           
            #self.mavlink_connection.mav.command_long_send(
            #    self.mavlink_connection.target_system,
            #    self.mavlink_connection.target_component,
            #    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            #    0, motor_number, 1, throttle, 4, 0, 0, 0)
            return True
        except Exception as e:
            print(f"Motor Test Error: {str(e)}")
            return False

    def set_manual_control(self, x, y, z, r):
        if not self.armed:
            return False
        drone.current_throttle = z
        try:
            self.mavlink_connection.mav.manual_control_send(
                self.mavlink_connection.target_system,
                x,  # Roll (-1000 to 1000)
                y,  # Pitch (-1000 to 1000)
                z,  # Thrust (0 to 1000)
                r,  # Yaw (-1000 to 1000)
                0)
            return True
        except Exception as e:
            print(f"Manual Control Error: {str(e)}")
            return False

drone = DroneController()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/connectrpi', methods=['POST'])
def connect_rpi():
    hostname = request.form.get('hostip')
    username = request.form.get('username')
    password = request.form.get('password')
    print("\nConnecting\n")
    result, message = drone.connect_ssh(hostname, username, password)
    return jsonify({
        'status': result,
        'message': message
    })

@app.route('/connectdrone', methods=['POST'])
def connectdr():
    done = drone.connect_mavlink()
    if done:
        return jsonify({"status": "success"})
    else:
        return jsonify({"status": "failed"})
    
@app.route('/telemetrystatus', methods=['POST'])
def get_telemetry():
    return jsonify(drone.telemetry_data)

#@app.route('/video_feed')
#def video_feed():
#        def generate():
#            stream = requests.get(RPI_STREAM_URL, stream=True)
#            for chunk in stream.iter_content(chunk_size=1024):
#                if chunk:
#                    yield chunk
#
#        return Response(generate(), 
#                    mimetype='multipart/x-mixed-replace; boundary=FRAME')

@app.route('/video_feed/edges')
def video_feed():
    frame_type = 'edges'
    return Response(generate_frames(frame_type),
                    mimetype='multipart/x-mixed-replace; boundary=frame')



@app.route('/arm', methods=['POST'])
def arm():
    if drone.arm_motors():
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error'})

@app.route('/disarm', methods=['POST'])
def disarm():
    if drone.disarm_motors():
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error'})

@app.route('/land', methods=['POST'])
def land():
    while drone.current_throttle > 0:
        target_throttle = drone.current_throttle - 1
        drone.set_manual_control(0, 0, target_throttle, 0)
        time.sleep(0.1)

    if drone.current_throttle == 0:
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error'})

@app.route('/test_motor', methods=['POST'])
def test_motor():
    data = request.json
    motor_num = data.get('motor')
    throttle = data.get('throttle')
    print(motor_num , throttle)
    if drone.test_motor(motor_num, throttle):
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error'})

@socketio.on('manual_control')
def handle_manual_control(data):
    x = data.get('x', 0)
    y = data.get('y', 0)
    z = data.get('z', 0)
    r = data.get('r', 0)
    drone.set_manual_control(x, y, z, r)

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)