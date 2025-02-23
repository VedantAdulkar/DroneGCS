# DroneGCS

These are the four main controls that determine how a drone moves in 3D space.

1. Pitch (Forward/Backward)
    Controls the tilt of the drone forward or backward.

2. Roll (Left/Right)
    Controls the tilt of the drone left or right.

3. Throttle (Up/Down)
    Controls the altitude of the drone.

4. Yaw (Rotate Left/Right)
    Controls the rotation of the drone left or right around its vertical axis.

| Control | RC Channel | PWM Range	| Effect |
| ------------- | ------------- | ------------- | ------------- |
| Roll	| 1	| 1000-2000	| Left (-) / Right (+) |
| Pitch	| 2	| 1000-2000	| Forward (-) / Backward (+) |
| Throttle	| 3	| 1000-2000	| Down (-) / Up (+) |
| Yaw	| 4	| 1000-2000	| Rotate Left (-) / Rotate Right (+) |



        mavproxy.py --master=/dev/serial0 --baudrate 57600 --out udp:192.168.11.6:14550

.. will start sending messages to laptops IP {192.168.11.6}

        self.master = mavutil.mavlink_connection("udp:0.0.0.0:14550")
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Connected!")

will take drone heartbeat from any IPs.(udp:0.0.0.0:14550 cause of 0.0.0.0)

So start first mavproxy command in rpi with the laptop ip then run laptop code with 0.0.0.0 so that it will recieve drone heartbeat from any network ip.