import random
import math
from ArmSim import ArmSim, ArmViz


############################
#       CONSTANTS
############################
# cross_prob = 1.0
# mutation_prob = 0.15
# population_size = 500
# num_dof = 2
# bits_per_theta = 16
# terminate_tol = 0.001
# max_generations = 100
############################

link_lengths = [1.0, 1.0]
goal_pose = [1.0, 1.0]

# TODO: Write other methods of each step (mutation already done)


def initial_thetas(num_dof: int = 2, bits_per_theta: int = 16) -> list[list[int]]:
    """
    initializes a set of theta values randomly.
    each theta is represented by a list of bits_per_theta bits,
    which ranges from 0-2pi

    returns:
        thetas: list of list of ints representing each theta in one arm
            configuration
    """
    thetas = []
    for _ in range(num_dof):
        bitstring = [random.randint(0, 1) for _ in range(bits_per_theta)]
        thetas.append(bitstring)
    return thetas


def generate_population(
    population_size: int = 500,
) -> list[list[list[int]]]:
    """
    generates an initial set of arm configurations randomly

    Returns:
        population: list[list[int]] : initial population set of thetas
    """
    population = []
    for _ in range(population_size):
        population.append(initial_thetas())
    return population


def error(configuration: list[list[int]]) -> float:
    """
    calculates the euclidean error between the goal pose and the current pose

    returns:
        error: float value representing the distance away from the goal pose
            that the current pose is
    """
    config_rad = bitstring_to_rad(configuration)
    arm = ArmSim(config_rad, link_lengths)
    all_joints_pose = arm.fk()  # all joint poses, last element is ee pose
    ee_pose = all_joints_pose[-1]

    error = math.dist(goal_pose, ee_pose)
    return error


def parent_select(population: list[list[list[int]]]) -> list[list[list[int]]]:
    """
    given current population, selects 100 new parents using tournament
        selection

    returns:
        selected_parents: list of arm configurations
    """

    selected_parents = []
    for _ in range(len(population)):
        tournament = random.sample(population, 3)

        tourny_dict = {}
        for i, entry in enumerate(tournament):
            score = error(entry)
            tourny_dict[i] = score

        winner = tournament[max(tourny_dict, key=tourny_dict.get)]
        selected_parents.append(winner)
    return selected_parents


def crossover(
    parent1: list[list[int]],
    parent2: list[list[int]],
    num_dof: int = 2,
    cross_prob: float = 0.85,
):
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
    cross_point = random.choice(range(0, num_dof))

    perform_crossover = random.choices([True, False], [cross_prob, 1 - cross_prob])

    if perform_crossover:
        p1_joint = parent1[cross_point]
        p2_joint = parent2[cross_point]
        parent1[cross_point] = p2_joint
        parent2[cross_point] = p1_joint

    return parent1, parent2


def mutation(
    configuration: list[list[int]],
    num_dof: int = 2,
    bits_per_theta: int = 16,
    mutation_prob: float = 0.35,
) -> list[list[int]]:
    """
    Performs a mutation based on the mutation probability

    args:
        configuration: list[list[int]]: one arm configuration to be mutated
    """
    # randomly assigns a bit to be flipped
    mut_joint = random.choice(range(0, num_dof))
    mut_bit = random.choice(range(0, bits_per_theta))

    # randomly (weighted by mutation_prob) decides whether to perform the mutation
    perform_mut = random.choices([True, False], [mutation_prob, 1 - mutation_prob])

    # perform the bit flip if perform_mut is true
    if perform_mut:
        current_bit = configuration[mut_joint][mut_bit]
        configuration[mut_joint][mut_bit] = 0 if current_bit == 1 else 1

    return configuration


def weighted_mutation(
    configuration: list[list[int]],
    num_dof: int = 2,
    bits_per_theta: int = 16,
    mutation_prob: float = 0.75,
) -> list[list[int]]:
    """
    Decides whether to mutate the configuration
    Once decided, flips a bit in the configuration, where lesser significant
    bits are more likely to be flipped

    Args:
        configuration: list[list[int]]: current arm config to mutate
    """

    # picks joint to mutate
    mut_joint = random.choice(range(0, num_dof))

    # picks which bit to mutate, less likely to mutate larger bits
    bit_weight = [0.5]
    bit_weight += [1 / x**2 for x in range(2, bits_per_theta + 1)]
    mut_bit = random.choices(
        range(0, bits_per_theta),
        weights=bit_weight[::-1],
    )[0]

    perform_mut = random.choices([True, False], [mutation_prob, 1 - mutation_prob])
    if perform_mut:
        current_bit = configuration[mut_joint][mut_bit]
        configuration[mut_joint][mut_bit] = 0 if current_bit == 1 else 1

    return configuration


def survivor_select(
    parents: list[list[list[int]]], children: list[list[list[int]]]
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
    parents_sorted = sorted(parents, key=lambda x: error(x), reverse=True)

    children_sorted = sorted(children, key=lambda x: error(x), reverse=True)

    # select the top 10% of parents
    survivors = [parents_sorted[i] for i in range(int(0.1 * len(parents_sorted)))]
    # select the top 90% of children
    survivors += [children_sorted[i] for i in range(int(0.9 * len(children_sorted)))]

    return survivors


def terminate(
    population: list[list[list[int]]],
    current_generation: int,
    max_generations: int = 500,
    terminate_tol: float = 0.01,
) -> bool:
    """
    determines whether the GA has gotten close enough to the best solution
    to terminate. Based on generation count and accuracy within a tolerance
    """
    # checks if the generation count has reached the max
    if current_generation >= max_generations:
        return True

    # checks if the best solution is within the tolerance
    best_solution = min(population, key=lambda x: error(x))
    best_error = error(best_solution)

    if best_error <= terminate_tol:
        return True

    return False


def bitstring_to_rad(thetas: list[list[int]], bits_per_theta: int = 16) -> list[float]:
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
        ratio = dec / (2**bits_per_theta)
        theta_rad.append(ratio * 2 * math.pi)
    return theta_rad

    # return [(int("".join(theta), 2) / (2**self.bits_per_theta) * 2 * math.pi) for theta in thetas]
    # uncomment if ur cool


# TODO: Update this function to not be in a class.
# Input different functions
# Integrate Sim
# Kill myself
def run_ga(self, rad: bool = False):
    """
    runs the whole ga based on the parameters passed into the init
    """
    # generate the initial population
    generate_population()
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
