# connect_test.py
from robomaster import robot
import time

def main():
    ep_robot = robot.Robot()
    # Use 'sta' and the IP you discovered:
    ep_robot.initialize(conn_type='sta', ip='172.20.10.15')
    print("Connected to RoboMaster S1 over hotspot!")
    for i in range(5):
        pos = ep_robot.chassis.get_position()
        print(f"[{i}] Position: x={pos[0]:.2f}, y={pos[1]:.2f}, yaw={pos[2]:.1f}")
        time.sleep(1)
    ep_robot.close()

if __name__ == '__main__':
    main()
