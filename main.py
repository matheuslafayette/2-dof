from robot import RobotArm
from transforms import SE2Transform
from visualization import visualize_trajectory, plot
import math
import numpy as np

def main():
    arm = RobotArm(a1=1, a2=1, max_vel=2, max_acc=0.75, timestamp=0.01)
    
    print("--------- Tests part 1 ---------")
    p = np.array([0.5, 0.5, 1])
    print("Test 1: ", (SE2Transform.xy(1, 0.25) @ p)[:2])
    print("Test 2: ", (SE2Transform.xy(-1, -0.25) @ p)[:2])
    print("Test 3: ", (SE2Transform.xy(1, 0.25) @ SE2Transform.theta(math.radians(45)) @ p)[:2])
    print("Test 4: ", (np.linalg.inv(SE2Transform.xy(1, 0.25) @ SE2Transform.theta(math.radians(45))) @ p)[:2])

    print("--------- Tests part 2 ---------")
    print("--------- Tests: FK ---------")
    print("Test 1: ", arm.fk(0, math.pi/2))
    print("Test 2: ", arm.fk(math.pi/2, math.pi/2))
    print("Test 3: ", arm.fk(math.pi/2, -math.pi/2))
    print("Test 4: ", arm.fk(-math.pi, math.pi))
    
    print("--------- Tests: IK ---------")
    print("Test 1: ", arm.ik(1, 1))
    print("Test 2: ", arm.ik(1, -1))
    print("Test 3: ", arm.ik(-1, 1))
    print("Test 4: ", arm.ik(-1, -1))
    print("Test 5: ", arm.ik(2, 1))
    print("Test 6: ", arm.ik(2, 0))
    print("Test 7: ", arm.ik(0, 2))
    print("Test 8: ", arm.ik(-2, 0))

    print("--------- Tests part 3 ---------")
    trajectory = arm.traj_joint(0, 0, 0.5, 3)
    plot(trajectory[0], arm.trajectory_generator.ts, "traj_joint1")
    plot(trajectory[1], arm.trajectory_generator.ts, "traj_joint2")

    print("--------- Tests: traj_eucl ---------")
    trajectory_eucl = arm.traj_eucl(0, 1, 1, 0)
    visualize_trajectory(trajectory_eucl, filename='RR.gif')

if __name__ == '__main__':
    main()