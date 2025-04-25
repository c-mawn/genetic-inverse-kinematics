"""
File for visualizing and simulating 2D robot arms
"""

import tkinter as tk
import numpy as np
import math
from weakref import WeakSet
from collections.abc import Callable


class ArmViz:
    """
    Arm visualizer class for displaying ArmSims.
    """

    def __init__(self, width: int = 960, height: int = 540):
        # The size of the canvas
        self.width = width
        self.height = height

        # Tkinter objects
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=self.width, height=self.height)
        self.canvas.pack()
        # Set up close button properly
        self.root.protocol("WM_DELETE_WINDOW", self.stop)

        # Set for storing ArmSim instances assigned to this vizualiser
        self.arms: WeakSet['ArmSim'] = WeakSet()

        self.running: bool = False
        self.callback: Callable[[], None] = lambda: None 

    def draw(self):
        """
        Redraws all of the arms on the canvas
        """

        # Clear the canvas
        self.canvas.delete('all')

        # For every instace of an arm sim
        for arm in self.arms:
            points = arm.fk()

            # Draw a line for each link
            for i, point in enumerate(points):
                
                # Get the position of the previous joint (in display space)
                x0, y0 = self.point_transform(points[i-1] if i > 0 else (0.0, 0.0))

                # Get the position of the current joint (in display space)
                x1, y1 = self.point_transform(point)

                # Draw the line
                self.canvas.create_line(x0, y0, x1, y1, fill=arm.color, width=5)

    def point_transform(self, point: tuple[float, float]) -> tuple[float, float]:
        """
        Transforms a point from simulation space to display space (the pixel location on the class canvas)

        Args:
            point: The point to transform

        Returns:
            The transformed point
        """

        # Set transformation parameters
        scale = 100
        x_offset = self.width/2
        y_offset = self.height/2

        # Return the transformed point. Flip y so that -y is down
        return scale * point[0] + x_offset, -scale * point[1] + y_offset 

    def run(self):
        """
        Run the main loop of the visualizer
        """
        self.running = True        
        while self.running:
            self.update()

    def stop(self):
        """
        Stop the main loop of the vizualizer from running
        """
        self.running = False

    def update(self):
        """
        One tick of the main loop. Resolves the callback, updates the canvas, then handles window events.
        """
        
        # Handle the callback
        self.callback()

        # Update the canvas
        self.draw()

        # Handle window events
        self.root.update_idletasks()
        self.root.update()


class ArmSim:
    """
    Arm simulation class for computing the kinematics of robot arms, with integration for use
    of a genetic algorithm to solve the inverse kinematics
    """

    def __init__(self, thetas: list[float], link_lengths: list[float], viz: ArmViz | None = None, color: str = 'black'):
        """
        Initializes an instance of the Arm Simulator

        Args:
            thetas: List of floats for each theta in the arm
            link_lengths: List of floats for each link length in the arm
        """
        self.thetas = thetas
        self.link_lengths = link_lengths
        self.color = color

        # The visualizer the arm is assigned to
        self._viz: ArmViz | None = None
        self.viz = viz

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
    
    @property
    def viz(self) -> ArmViz | None:
        """
        The visualizer the this arm is assigned to
        """
        return self._viz
    
    @viz.setter
    def viz(self, ArmViz) -> None:
        """
        Change the visualizer that this arm is assigned to
        """
        # If the arm is currently assigned to a visualizer, unassign it
        if self._viz:
            self._viz.arms.remove(self)
        
        # Update the property
        self._viz = ArmViz

        # If the arm needs to be assigned to a new visualizer, assign it
        if self._viz:
            self._viz.arms.add(self)


# Test the classes
if __name__ == '__main__':
    import time

    # Create a visualizer
    viz = ArmViz()

    # Create three arms to test with
    arm1 = ArmSim([np.pi / 3, np.pi / 2], [1, 1])
    arm2 = ArmSim([-np.pi / 4, -np.pi / 2], [2, 1], viz, 'blue')
    arm3 = ArmSim([4 * np.pi / 3, -np.pi / 2, np.pi / 2, -np.pi / 2, 1/6 * np.pi], [0.5, 0.5, 0.5, 0.5, 1], viz, 'green')

    # Get a variable to keep track of how many times the main loop has run
    i = 0

    def timestep():
        """
        What to do ever timestep of the visualizer main loop
        """

        # Handle the iterator variable
        global i
        i += 1

        # Spin the first and last joints of arm 3 a little bit
        arm3.thetas[0] += 0.01
        arm3.thetas[4] += 0.01

        # Assign arm 1 to the visualizer on and off, so that it flashes
        arm1.viz = viz if i % 100 > 50 else None

        # Include a time delay
        time.sleep(0.01)
    
    # Start the visualizer loop
    viz.callback = timestep
    viz.run()
