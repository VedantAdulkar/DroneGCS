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

app = Flask(__name__)
socketio = SocketIO(app)

RPI_STREAM_URL = 'http://raspberrypi.local:8080/stream.mjpg'

# SSH Configuration
SSH_HOST = 'raspberrypi.local'
SSH_USERNAME = 'pi'
SSH_PASSWORD = ''
MAVLINK_CONNECTION = '/dev/serial0:57600'  # Update based on your Pixhawk connection

class DroneController:
    def __init__(self):
        self.ssh_client = None
        self.mavlink_connection = None
        self.isRPIconnected = False
        self.is_connected = False
        self.armed = False
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

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
            # Forward MAVLink connection through SSH tunnel
            transport = self.ssh_client.get_transport()
            channel = transport.open_channel('direct-tcpip', 
                                          (MAVLINK_CONNECTION, 14550), 
                                          ('127.0.0.1', 14550))
            
            # Connect to MAVLink
            self.master = mavutil.mavlink_connection("udp:0.0.0.0:14550")
            print("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            print("Connected!")
            self.is_connected = True
            return True
        except Exception as e:
            print(f"MAVLink Connection Error: {str(e)}")
            return False

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
        if not self.armed:
            return False
        try:
            # Send motor test command
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0, motor_number, 1, throttle, 2, 0, 0, 0)
            return True
        except Exception as e:
            print(f"Motor Test Error: {str(e)}")
            return False

    def set_manual_control(self, x, y, z, r):
        if not self.armed:
            return False
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

@app.route('/connect', methods=['POST'])
def connect_rpi():
    hostname = request.form.get('hostip')
    username = request.form.get('username')
    password = request.form.get('password')
    
    result, message = drone.connect_ssh(hostname, username, password)
    return jsonify({
        'status': result,
        'message': message
    })

@app.route('/video_feed')
def video_feed():
    def generate():
        stream = requests.get(RPI_STREAM_URL, stream=True)
        for chunk in stream.iter_content(chunk_size=1024):
            if chunk:
                yield chunk

    return Response(generate(), 
                    mimetype='multipart/x-mixed-replace; boundary=FRAME')

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

@app.route('/test_motor', methods=['POST'])
def test_motor():
    data = requests.json
    motor_num = data.get('motor')
    throttle = data.get('throttle')
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