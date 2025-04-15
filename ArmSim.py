"""
File for visualizing the arm simulator
"""

import tkinter as tk
import numpy as np
import math


class ArmSim:
    """
    Arm simulation class for visualizing robot arms, with integration for use
    of a genetic algorithm to solve the inverse kinematics
    """

    def __init__(self, thetas: list[float], link_lengths: list[float]):
        """
        Initializes an instance of the Arm Simulator

        Args:
            thetas: List of floats for each theta in the arm
            link_lengths: List of floats for each link length in the arm
        """
        self.thetas = thetas
        self.link_lengths = link_lengths

    def fk(self):  # -> list[tuple[float, float]]:
        """
        calculates the forward kinematics of the robot arm. Generates the end
        effector pose based on the theta values for each link

        Returns:
            list of float tuples: positions of each join in the arm
        """
        # do da ting
        # vectors: list[np.ndarray] = [np.transpose(np.array([0, 0, 1]))]
        # for theta, link_length in zip(self.thetas, self.link_lengths):
        #     print(f"{theta=}, {link_length=}")
        #     htm = np.array(
        #         [
        #             [np.cos(theta), -np.sin(theta), link_length],
        #             [np.sin(theta), np.cos(theta), 0],
        #             [0, 0, 1],
        #         ]
        #     )
        #     print(f"{htm=}")
        #     vectors.append(htm @ vectors[-1])
        #     print(f"{vectors=}")

        # return [tuple(vector[:-1]) for vector in vectors]

        # want to return each joint x,y
        joint_positions: list[tuple[float, float]] = []
        theta_total = 0
        for theta, link_length in zip(self.thetas, self.link_lengths):
            theta_total += theta
            x, y = joint_positions[-1] if len(joint_positions) > 0 else (0.0, 0.0)
            x += math.cos(theta_total) * link_length
            y += math.sin(theta_total) * link_length
            joint_positions.append((x, y))

        return joint_positions


sim = ArmSim([np.pi / 3, np.pi / 2], [1, 1])
print(sim.fk())
