"""
File that runs a genetic algorithm to solve the inverse kinematics
of a robot arm
"""

import random
import math
from ArmSim import ArmSim, ArmViz


class GeneticAlgorithm:
    """
    runs a genetic algorithm to solve the inverse kinematics of a robot arm
    """

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
        mutation_prob: float = 0.15,
        population_size: int = 500,
        num_dof: int = 2,
        bits_per_theta: int = 16,
        terminate_tol: float = 0.001,
        max_generations: int = 100,
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
        self.terminate_tol = terminate_tol
        self.max_generations = max_generations

        self.population = []
        self.generation = 0
        self.viz = ArmViz()

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
        config_rad = self.bitstring_to_rad(configuration)
        arm = ArmSim(config_rad, self.link_lengths, self.viz)
        all_joints_pose = arm.fk()  # all joint poses, last element is ee pose
        self.viz.update()
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

            tourny_dict = {}
            for i, entry in enumerate(tournament):
                score = self.error(entry)
                tourny_dict[i] = score

            winner = tournament[max(tourny_dict, key=tourny_dict.get)]
            selected_parents.append(winner)
        return selected_parents

    def crossover(self, parent1: list[list[int]], parent2: list[list[int]]):
        """
        performs a crossover between two parents by swapping one whole joint
        with another parent's joint

        args:
            parent1: list[list[int]]: first arm configuration parent
            parent2: list[list[int]]: second arm configuration parent

        returns:
            parent1: list[list[int]]: parent1 new (possibly unchanged) config
            parent2: list[list[int]]: parent2 new (possibly unchanged) config
        """
        cross_point = random.choice(range(0, self.num_dof))

        perform_crossover = random.choices(
            [True, False], [self.cross_prob, 1 - self.cross_prob]
        )

        if perform_crossover:
            p1_joint = parent1[cross_point]
            p2_joint = parent2[cross_point]
            parent1[cross_point] = p2_joint
            parent2[cross_point] = p1_joint

        return parent1, parent2

    def mutation(self, configuration: list[list[int]]) -> list[list[int]]:
        """
        Performs a mutation based on the mutation probability

        args:
            configuration: list[list[int]]: one arm configuration to be mutated
        """
        # randomly assigns a bit to be flipped
        mut_joint = random.choice(range(0, self.num_dof))
        mut_bit = random.choice(range(0, self.bits_per_theta))

        # randomly (weighted by mutation_prob) decides whether to perform the mutation
        perform_mut = random.choices(
            [True, False], [self.mutation_prob, 1 - self.mutation_prob]
        )

        # perform the bit flip if perform_mut is true
        if perform_mut:
            current_bit = configuration[mut_joint][mut_bit]
            configuration[mut_joint][mut_bit] = 0 if current_bit == 1 else 1

        return configuration

    def survivor_select(
        self, parents: list[list[list[int]]], children: list[list[list[int]]]
    ) -> list[list[list[int]]]:
        """
        determines which of the parent and offspring survive based on fitness.
        the top 90% of children will survive, and the 10% of parents survive

        args:
            parents: list[list[list[int]]]: list of parents
            children: list[list[list[int]]]: list of children

        returns:
            survivors: list[list[list[int]]]: list of arm configurations that
            survive to the next generation
        """
        # sort the parents and children based on their fitness
        parents_sorted = sorted(parents, key=lambda x: self.error(x), reverse=True)

        children_sorted = sorted(children, key=lambda x: self.error(x), reverse=True)

        # select the top 10% of parents
        survivors = [parents_sorted[i] for i in range(int(0.1 * len(parents_sorted)))]
        # select the top 90% of children
        survivors += [
            children_sorted[i] for i in range(int(0.9 * len(children_sorted)))
        ]

        return survivors

    def terminate(self):
        """
        determines whether the GA has gotten close enough to the best solution
        to terminate. Based on generation count and accuracy within a tolerance
        """
        # checks if the generation count has reached the max
        if self.generation >= self.max_generations:
            return True

        # checks if the best solution is within the tolerance
        best_solution = min(self.population, key=lambda x: self.error(x))
        best_error = self.error(best_solution)

        if best_error <= self.terminate_tol:
            return True

        return False

    def run_ga(self, rad: bool = False):
        """
        runs the whole ga based on the parameters passed into the init
        """
        # generate the initial population
        self.generate_population()
        # run the ga until termination
        while not self.terminate():
            # increment the generation count
            print(f"Generation: {self.generation}\n")
            current_best = min(self.population, key=lambda x: self.error(x))
            current_best_error = self.error(current_best)
            print(f"Current Best = {current_best} : {current_best_error} ")
            self.generation += 1

            # select parents
            selected_parents = self.parent_select()
            # generate children
            children = []
            for i in range(0, len(selected_parents), 2):
                parent1 = selected_parents[i]
                parent2 = selected_parents[i + 1]

                child1, child2 = self.crossover(parent1, parent2)

                child1 = self.mutation(child1)
                child2 = self.mutation(child2)

                children.append(child1)
                children.append(child2)

            # select survivors
            self.population = self.survivor_select(selected_parents, children)

        # return the best solution
        best_solution = min(self.population, key=lambda x: self.error(x))
        return self.bitstring_to_rad(best_solution) if rad else best_solution

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
            bitstring = "".join(str(x) for x in theta)
            dec = int(bitstring, 2)
            ratio = dec / (2**self.bits_per_theta)
            theta_rad.append(ratio * 2 * math.pi)
        return theta_rad

        # return [(int("".join(theta), 2) / (2**self.bits_per_theta) * 2 * math.pi) for theta in thetas]
        # uncomment if ur cool


ga = GeneticAlgorithm(goal_pose=(1.0, 1.0), link_lengths=[1.0, 1.0])
best_soln = ga.run_ga()
best_soln = ga.bitstring_to_rad(best_soln)
sim = ArmSim(best_soln, link_lengths=[1.0, 1.0])
pose = sim.fk()
print(f"{best_soln=} pose: {pose}")
