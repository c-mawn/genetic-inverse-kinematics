from ArmSim import ArmSim, ArmViz
import numpy as np
import time


class Examples():
    def __init__(self) -> None:
        # PICK WHICH EXAMPLE(S) TO RUN HERE
        # self.ArmViz_using_run()
        self.ArmViz_manual_updates()

    def ArmViz_using_run(self):
        self.viz_setup()
        
        # Start the visualizer loop
        self.viz.callback = self.timestep
        self.viz.run()

    def ArmViz_manual_updates(self):
        self.viz_setup()

        while self.viz.running:
            self.timestep()
            self.viz.update()

    def viz_setup(self):
        # Create a visualizer
        self.viz = ArmViz((1, 1))

        # Create three arms to test with
        self.arm1 = ArmSim([np.pi / 3, np.pi / 2], [1, 1])
        self.arm2 = ArmSim([-np.pi / 4, -np.pi / 2], [2, 1], self.viz, 'blue')
        self.arm3 = ArmSim([4 * np.pi / 3, -np.pi / 2, np.pi / 2, -np.pi / 2, 1/6 * np.pi], [0.5, 0.5, 0.5, 0.5, 1], self.viz, 'green')

        # Get a variable to keep track of how many times the main loop has run
        self.i = 0

    def timestep(self):
        """
        What to do every timestep of the visualizer main loop
        """

        # Handle the iterator variable
        self.i += 1

        # Spin the first and last joints of arm 3 a little bit
        self.arm3.thetas[0] += 0.01
        self.arm3.thetas[4] += 0.01

        # Assign arm 1 to the visualizer on and off, so that it flashes
        self.arm1.viz = self.viz if self.i % 100 > 50 else None

        # Include a time delay
        time.sleep(0.01)
        

Examples()