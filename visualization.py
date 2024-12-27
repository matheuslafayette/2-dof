import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import numpy as np

def visualize_trajectory(trajectory, filename='RR.gif'):
    robot = rtb.models.DH.Planar2()
    nb_steps = len(trajectory)
    qt = np.zeros((nb_steps, 2))
    for i in range(nb_steps):
        qt[i, :] = [trajectory[i][0], trajectory[i][1]]
    robot.plot(qt, backend='pyplot', movie=filename)
    print(f'Trajetória salva em: {filename}')

def plot(trajectory, ts, filename):
    time_steps = [i * ts for i in range(len(trajectory))]
    velocity = [(trajectory[i + 1] - trajectory[i]) / ts for i in range(len(trajectory) - 1)]
    time_steps_vel = time_steps[:-1]
    acceleration = [(velocity[i + 1] - velocity[i]) / ts for i in range(len(velocity) - 1)]
    time_steps_acc = time_steps_vel[:-1]

    plt.figure(figsize=(8, 5))
    plt.plot(time_steps, trajectory, marker='o', linestyle='-', linewidth=2, markersize=6)
    plt.title("Trajetória Gerada")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Posição")
    plt.grid(True)
    plt.savefig(f'{filename}.png')
    plt.close()

    plt.figure(figsize=(8, 5))
    plt.plot(time_steps_vel, velocity, marker='x', linestyle='-', color='red', linewidth=2)
    plt.title("Velocidade Derivada")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Velocidade")
    plt.grid(True)
    plt.savefig(f'{filename}_vel.png')
    plt.close()

    plt.figure(figsize=(8, 5))
    plt.plot(time_steps_acc, acceleration, marker='s', linestyle='-', color='blue', linewidth=2)
    plt.title("Aceleração Derivada")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Aceleração")
    plt.grid(True)
    plt.savefig(f'{filename}_acc.png')
    plt.close()