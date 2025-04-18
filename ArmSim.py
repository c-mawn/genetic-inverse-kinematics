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

    # The size of the canvas
    width = 1920/2
    height = 1080/2

    # The tkinter objects
    root = tk.Tk()
    canvas = tk.Canvas(root, width=width, height=height)
    canvas.pack()

    # The list of ArmSim instances
    arms: list['ArmSim'] = [] # The list of arms ArmSim type in quotes to forward reference

    @classmethod
    def draw(cls):
        """
        Redraws all of the arms on the class canvas
        """

        # Clear the canvas
        cls.canvas.delete('all')

        # For every instace of an arm sim
        for arm in cls.arms:
            points = arm.fk()

            # Draw a line for each link
            for i, point in enumerate(points):
                
                # Get the position of the previous joint (in display space)
                x0, y0 = cls.point_transform(points[i-1] if i > 0 else (0.0, 0.0))

                # Get the position of the current joint (in display space)
                x1, y1 = cls.point_transform(point)

                # Draw the line
                cls.canvas.create_line(x0, y0, x1, y1, fill=arm.color, width=5)

    @classmethod
    def point_transform(cls, point: tuple[float, float]) -> tuple[float, float]:
        """
        Transforms a point from simulation space to display space (the pixel location on the class canvas)

        Args:
            point: The point to transform

        Returns:
            The transformed point
        """

        # Set transformation parameters
        scale = 100
        x_offset = cls.width/2
        y_offset = cls.height/2

        # Return the transformed point. Flip y so that -y is down
        return scale * point[0] + x_offset, -scale * point[1] + y_offset 

    @classmethod
    def run(cls):
        """
        Run the main display loop and start the simulation
        """
        
        cls.draw()
        cls.root.mainloop()

    def __init__(self, thetas: list[float], link_lengths: list[float], color: str = 'black'):
        """
        Initializes an instance of the Arm Simulator

        Args:
            thetas: List of floats for each theta in the arm
            link_lengths: List of floats for each link length in the arm
        """
        self.thetas = thetas
        self.link_lengths = link_lengths
        self.color = color

        ArmSim.arms.append(self)

    def fk(self):  # -> list[tuple[float, float]]:
        """
        calculates the forward kinematics of the robot arm. Generates the end
        effector pose based on the theta values for each link

        Returns:
            list of float tuples: positions of each join in the arm
        """

        joint_positions: list[tuple[float, float]] = []
        theta_total = 0

        # Loop over joints
        for theta, link_length in zip(self.thetas, self.link_lengths):
            
            # Calculate the current link angle
            theta_total += theta

            # Get the current joint position
            x, y = joint_positions[-1] if len(joint_positions) > 0 else (0.0, 0.0)

            # Calculate the next joint position
            x += math.cos(theta_total) * link_length
            y += math.sin(theta_total) * link_length

            # Add the next position to the list of positions
            joint_positions.append((x, y))

        return joint_positions


sim = ArmSim([np.pi / 3, np.pi / 2], [1, 1])
sim = ArmSim([-np.pi / 4, -np.pi / 2], [2, 1], 'blue')
sim = ArmSim([4 * np.pi / 3, -np.pi / 2, np.pi / 2, -np.pi / 2, 1/6 * np.pi], [0.5, 0.5, 0.5, 0.5, 1], 'green')

print(sim.fk())

ArmSim.run()