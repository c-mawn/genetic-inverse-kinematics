"""
File that runs a genetic algorithm to solve the inverse kinematics
of a robot arm
"""

import random
import math
import ArmSim


class GeneticAlgorithm:
    """ """

    def __init__(
        self,
        # fitness_func: callable,
        # parent_select: callable,
        # crossover: callable,
        # mutation: callable,
        # survivor_select: callable,
        # termination: callable,
        goal_pose: tuple[float, float],
        link_lengths: list[float],
        cross_prob: float = 1.0,
        mutation_prob: float = 0.3,
        population_size: int = 100,
        num_dof: int = 2,
        bits_per_theta: int = 16,
    ):
        """
        initializes an instance of the GeneticAlgorithm class

        Args:
            fitness_func: callable, determines which fitness function to use
            parent_select: callable, determines which parent_select to use
            crossover: callable, determines which crossover to use
            mutation: callable, determines which mutation to use
            survivor_select: callable, determines which survivor_select to use
            termination: callable, determines which termination to use
            cross_prob: float, probability that a crossover occurs in a generation
            mutation_prob: float, probability that a mutation occurs in a generation
            population_size: int, number of individuals used in each generation
        """
        # self.fitness_func = fitness_func
        # self.parent_select = parent_select
        # self.crossover = crossover
        # self.mutation = mutation
        # self.survivor_select = survivor_select
        # self.termination = termination

        self.goal_pose = goal_pose
        self.link_lengths = link_lengths
        self.cross_prob = cross_prob
        self.mutation_prob = mutation_prob
        self.population_size = population_size
        self.num_dof = num_dof
        self.bits_per_theta = bits_per_theta

        self.population = []

    ### GENETIC ALGORITHM FUNCTIONS ###
    def initial_thetas(self) -> list[list[int]]:
        """
        initializes a set of theta values randomly.
        each theta is represented by a list of bits_per_theta bits,
        which ranges from 0-2pi

        returns:
            thetas: list of list of ints representing each theta in one arm
                configuration
        """
        thetas = []
        for _ in range(self.num_dof):
            bitstring = [random.randint(0, 1) for _ in range(self.bits_per_theta)]
            thetas.append(bitstring)
        return thetas

    def generate_population(self):
        """
        generates an initial set of arm configurations randomly
        """
        for _ in range(self.population_size):
            self.population.append(self.initial_thetas())

    def error(self, configuration) -> float:
        """
        calculates the euclidean error between the goal pose and the current pose

        returns:
            error: float value representing the distance away from the goal pose
                that the current pose is
        """
        arm = ArmSim(self.bitstring_to_rad(configuration), self.link_lengths)
        all_joints_pose = arm.fk()  # all joint poses, last element is ee pose
        ee_pose = all_joints_pose[-1]

        error = math.dist(self.goal_pose, ee_pose)
        return error

    def parent_select(self) -> list[list[list[int]]]:
        """
        given current population, selects 100 new parents using tournament
            selection

        returns:
            selectec_parents: list of arm configurations
        """

        selected_parents = []
        for _ in range(len(self.population)):
            tournament = random.sample(self.population, 3)
            winner = max(tournament, key=lambda x: x[1])[0]
            selected_parents.append(winner)
        return selected_parents

    # TODO: crossover, mutation, survivors, termination

    ### HELPER FUNCTIONS ###
    def bitstring_to_rad(self, thetas: list[list[int]]) -> list[float]:
        """
        converts the theta values from the bitstring repr to the radian repr

        args:
            thetas: list of lists of ints representing the binary values of each
                theta in the robot arm
        returns:
            theta_rad: list of floats representing a radian value for each theta
        """
        theta_rad = []
        for theta in thetas:
            bitstring = "".join(theta)
            dec = int(bitstring, 2)
            ratio = dec / (2**self.bits_per_theta)
            theta_rad.append(ratio * 2 * math.pi)
        return theta_rad

        # return [(int("".join(theta), 2) / (2**self.bits_per_theta) * 2 * math.pi) for theta in thetas]
        # uncomment if ur cool
