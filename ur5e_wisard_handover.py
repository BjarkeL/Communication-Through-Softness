#!/home/bjarke/.virtualenvs/ur5e/bin/python
import rtde_receive
import rtde_control
import robotiq_gripper
import time
import serial
import random


# Waypoints (Picking taps on the left side from the robot's perspective).
HOME = [0.0, -1.571, 1.571, -1.571, -1.571, 0.0]
OVER_O1 = [-0.188, -0.862, 0.852, -1.565, -1.55, 0.209]
OVER_O2 = [-0.351, -0.977, 0.959, -1.554, -1.549, 0.046]
OVER_O3 = [-0.528, -1.023, 1.022, -1.567, -1.549, -0.131]
OVER_OBJECT = [OVER_O1, OVER_O2, OVER_O3]
ON_O1 = [-0.188, -0.832, 1.053, -1.797, -1.55, 0.209]
ON_O2 = [-0.351, -0.93, 1.221, -1.863, -1.549, 0.046]
ON_O3 = [-0.528, -0.969, 1.283, -1.882, -1.549, -0.13]
ON_OBJECT = [ON_O1, ON_O2, ON_O3]
WEIGH_O1 = [-0.188, -0.834, 1.047, -1.788, -1.55, 0.209]#[-0.188, -0.833, 1.05, -1.792, -1.55, 0.209]
WEIGH_O2 = [-0.351, -0.933, 1.215, -1.853, -1.549, 0.046]#[-0.351, -0.931, 1.219, -1.86, -1.549, 0.046]
WEIGH_O3 = [-0.528, -0.973, 1.275, -1.869, -1.549, -0.13]#[-0.528, -0.97, 1.28, -1.877, -1.549, -0.13]
WEIGH_OBJECT = [WEIGH_O1, WEIGH_O2, WEIGH_O3]
INTERMEDIATE = [-0.053, -1.632, 1.594, -1.539, -1.55, 0.344]
HAND_OVER = [1.353, -1.228, 0.672, -1.034, -1.574, 1.75]

# Gripper
OPEN = 0
CLOSE = 255
GRIPPER_FORCE = 255
GRIPPER_SPEED = 120

# Payloads
PL_NOTHING = 1.14
PL_LIGHT = 1.29
PL_MEDIUM = 2.02
PL_HEAVY = 2.89
PAYLOAD_OBJECTS = [PL_LIGHT, PL_MEDIUM, PL_HEAVY]
PL_METAL_BLOCK = 2.17

# Center of gravity
CG_NOTHING = [0.003, -0.001, 0.044]
CG_LIGHT = [0.008, 0.001, 0.105]
CG_MEDIUM = [0.028, 0.001, 0.135]
CG_HEAVY = [0.02, 0.003, 0.160]
CG_OBJECTS = [CG_LIGHT, CG_MEDIUM, CG_HEAVY]
CG_METAL_BLOCK = [-0.005, -0.003, 0.079]

ip = "192.168.0.212"

class RobotControl:
    rec = rtde_receive.RTDEReceiveInterface(ip)
    control = rtde_control.RTDEControlInterface(ip)
    gripper = robotiq_gripper.RobotiqGripper()
    writer = serial.Serial('/dev/ttyACM0', 9600)
    inflate = True

    def __init__(self):
        self.control.setPayload(PL_NOTHING, CG_NOTHING)

    def init_for_movement(self):
        self.control.moveJ(HOME, 1, 1)
        self.gripper.connect(ip, 63352)
        self.gripper.activate()
        time.sleep(0.5)
        self.control.zeroFtSensor()
        time.sleep(1)

    def set_inflation(self, set_inflation):
        self.inflate = set_inflation

    def open_gripper(self):
        self.gripper.move_and_wait_for_pos(OPEN, GRIPPER_SPEED, GRIPPER_FORCE)
    
    def close_gripper(self):
        self.gripper.move_and_wait_for_pos(CLOSE, GRIPPER_SPEED, GRIPPER_FORCE)

    def read_switch(self):
        return (self.rec.getActualDigitalInputBits()>>5) & 1

    def get_max_force(self):
        f = self.rec.getActualTCPForce()
        return round(max(map(abs, f[0:3]))*10)

    def send_force(self):
        max_force = self.get_max_force()
        print("Sending force:", max_force)
        self.writer.write(bytes([max_force]))
    
    def print_force(self):
        print(self.get_max_force())

    # def update_payload(self):
    #     force = self.get_max_force()
    #     if (force <= 10):
    #         self.control.setPayload(PL_NOTHING, CG_NOTHING)
    #     if (force > 10 and force < 65):
    #         self.control.setPayload(PL_LIGHT, CG_LIGHT)
    #     if (force > 65 and force < 140):
    #         self.control.setPayload(PL_MEDIUM, CG_MEDIUM)
    #     if (force > 140):
    #         self.control.setPayload(PL_HEAVY, CG_HEAVY)

    def move_to(self, position):
        self.control.moveJ(HOME, 0.5, 0.5)
        self.control.moveJ(position, 0.5, 0.5)

    def pick_object(self, _object):
        self.control.moveJ(OVER_OBJECT[_object], 1, 1)
        self.open_gripper()
        self.control.moveJ(ON_OBJECT[_object], 1, 1)
        self.close_gripper()
        self.control.moveJ(WEIGH_OBJECT[_object], 0.1, 0.1)
        time.sleep(0.5)
        if self.inflate:
            self.send_force()
            force = self.get_max_force()
            if (force < 65):
                time.sleep(2)
            if (force > 65 and force < 140):
                time.sleep(5)
            if (force > 140):
                time.sleep(10)
        # self.update_payload()
        self.control.moveJ(OVER_OBJECT[_object], 0.5, 0.5)
        time.sleep(2)
        
    def place_object(self, _object):
        self.control.moveJ(OVER_OBJECT[_object], 0.5, 0.5)
        self.control.moveJ(ON_OBJECT[_object], 0.2, 0.2)
        self.open_gripper()
        if self.inflate: self.send_force()
        # self.update_payload()
        self.control.moveJ(OVER_OBJECT[_object], 0.5, 0.5)

    def present_object(self):
        self.control.moveJ(INTERMEDIATE, 1, 0.5)
        self.control.moveJ(HAND_OVER, 1, 0.5)

    def hand_object(self):
        self.wait_for_switch()
        self.open_gripper()
        #self.update_payload()
        time.sleep(0.5)
        if self.inflate: self.send_force()
        time.sleep(2)
        self.return_home()
    
    def return_home(self):
        self.control.moveJ(HOME, 1, 1)
        time.sleep(2)

    def wait_for_switch(self):
        # Catch if the switch is accidentally activated
        while self.read_switch():
            print("Switch engaged.")
            time.sleep(0.2)
        while not self.read_switch():
            print("Waiting for switch.")
            time.sleep(0.2)

    def get_position(self):
        q = self.rec.getActualQ()
        print([float(format(x, ".3f")) for x in q])
    
    def test_input(self):
        for i in range(100):
            print(self.rec.getActualDigitalInputBits()>>5 & 1)
            time.sleep(0.1)

    def main(self):
        # Make random order
        object_order = [0, 1, 2]
        random.shuffle(object_order)
        print(object_order)
        for object_to_pick in object_order:
            self.wait_for_switch()
            self.return_home()
            self.pick_object(object_to_pick)
            self.present_object()
            self.hand_object()

if __name__ == "__main__":

    robot = RobotControl()
    robot.init_for_movement()
    robot.set_inflation(True)
    robot.main()
    
    # robot.test_input()
    # robot.get_position()
    