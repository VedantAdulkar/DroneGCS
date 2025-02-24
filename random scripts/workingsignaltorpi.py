import time
from pymavlink import mavutil

#-----------------------------------------------



#-----------------------------------------------
class DroneBasicControl:
    def __init__(self):
        # Connect to drone1


        self.master = mavutil.mavlink_connection("udp:0.0.0.0:14550")
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Connected!")


    def set_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        print(f"Mode: {mode}")

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        print("Armed")

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)
        print("Disarmed")

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """ Set RC channel PWM value
        Args:
            channel_id: Channel ID (1-8): 1=roll, 2=pitch, 3=throttle, 4=yaw
            pwm: PWM value (1000-2000)
        """
        if channel_id < 1 or channel_id > 8:
            return
        
        # Create RC channel list (8 channels)
        rc_channel_values = [1500] * 8
        rc_channel_values[channel_id - 1] = pwm
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values)  # Expand the 8 channels

    def send_rc_override(self, roll=1500, pitch=1500, throttle=1000, yaw=1500):
        """Send RC override directly"""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            roll,     # Channel 1
            pitch,    # Channel 2
            throttle, # Channel 3
            yaw,      # Channel 4
            1500,     # Channel 5
            1500,     # Channel 6
            1500,     # Channel 7
            1500      # Channel 8
        )

    def move(self, roll=0, pitch=0, throttle=0, yaw=0):
        """
        Move drone using RC override
        Args:
            roll: -100 to 100 (-100=left, 100=right)
            pitch: -100 to 100 (-100=forward, 100=backward) 
            throttle: 0 to 100 (0=min, 100=max)
            yaw: -100 to 100 (-100=left, 100=right)
        """
        # Convert -100 to 100 range to PWM range (1000-2000)
        roll_pwm = 1500 + (roll * 5)
        pitch_pwm = 1500 + (pitch * 5)
        throttle_pwm = 1000 + (throttle * 10)
        yaw_pwm = 1500 + (yaw * 5)

        # Send RC override
        self.send_rc_override(roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm)
        print(f"Move: roll={roll_pwm}, pitch={pitch_pwm}, throttle={throttle_pwm}, yaw={yaw_pwm}")

    def hover(self):
        """Set all channels to hover position"""
        self.send_rc_override(1500, 1500, 1500, 1500)  # Center all sticks
        print("Hover position")

    def stop(self):
        """Stop all movement"""
        self.send_rc_override(1500, 1500, 1000, 1500)  # Minimum throttle
        print("Stopped")

def main():
    drone = DroneBasicControl()

    try:
        while True:
            cmd = input("""
Enter command:
1: Set STABILIZE mode
2: Arm
3: Disarm
4: Hover
5: Move forward
6: Move backward
7: Move left
8: Move right
9: Yaw left
10: Yaw right
11: Stop
q: Quit
Command: """)

            if cmd == 'q':
                break
            elif cmd == '1':
                drone.set_mode('STABILIZE')
            elif cmd == '2':
                drone.arm()
            elif cmd == '3':
                drone.disarm()
            elif cmd == '4':
                drone.hover()
            elif cmd == '5':
                drone.move(pitch=-50)  # Forward
            elif cmd == '6':
                drone.move(pitch=50)   # Backward
            elif cmd == '7':
                drone.move(roll=-50)   # Left
            elif cmd == '8':
                drone.move(roll=50)    # Right
            elif cmd == '9':
                drone.move(yaw=-50)    # Yaw left
            elif cmd == '10':
                drone.move(yaw=50)     # Yaw right
            elif cmd == '11':
                drone.stop()


    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        drone.stop()
        time.sleep(1)
        drone.disarm()

if __name__ == "__main__":
    main()