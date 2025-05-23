import random
import math
from ArmSim import ArmSim, ArmViz
from typing import Callable
from helpers import Configuration, Fitness_func, angles


# Configuration Type is a type alias for list of ints,
# representing one whole arm configuration

# Fitness_func Type is a type alias for a fitness function, it is of type:
# Callable[[Configuration, list[float], list[float], ArmViz|None], float]


def random_initial_thetas(num_dof: int = 2, bits_per_theta: int = 16) -> Configuration:
    """
    initializes a set of theta values randomly.
    each theta is represented by a list of bits_per_theta bits,
    which ranges from 0-2pi

    Args:
        num_dof (int): The number of degrees of freedom of the desired robot arm
        bits_per_theta (int): The number of bits to use to represent each angle

    Returns:
        list of ints each representing an angle in one arm configuration
    """
    return [random.randint(0, 2**bits_per_theta - 1) for _ in range(num_dof)]


def preset_initial_thetas(num_dof: int = 2, bits_per_theta: int = 16) -> Configuration:
    """
    initializes a set of theta values set to all 0

    Args:
        num_dof (int): The number of degrees of freedom of the desired robot arm
        bits_per_theta (int): The number of bits to use to represent each angle

    Returns:
        thetas: list of ints each representing an angle in one arm configuration
    """
    return [0] * num_dof


def generate_population(
    init_func: Callable,
    population_size: int = 500,
    num_dof: int = 2,
    bits_per_theta: int = 16,
) -> list[Configuration]:
    """
    generates an initial set of arm configurations randomly

    Args:
        init_func: Callable: function to use to generate the initial population
            (either random or preset)
        population_size: int: The number of arm configurations to generate
        num_dof: int: The number of degrees of freedom of the desired robot arm
        bits_per_theta: int: The number of bits to use to represent each angle

    Returns:
        population: list[list[int]] : initial population set of thetas
    """

    population = []
    for _ in range(population_size):
        # appends a new configuration to the population by calling the init func
        # that was passed into the generate function (either random or preset)
        population.append(init_func(num_dof, bits_per_theta))
    return population


def manhattan_error(
    configuration: Configuration,
    goal_pose: list[float],
    link_lengths: list[float],
    viz: ArmViz | None = None,
) -> float:
    """
    calculates the manhattan distance error between the goal pose and the
    current pose

    Args:
        configuration: list[list[int]]: arm configuration to calc error
        goal_pose: list[float]: the goal pose to reach
        link_lengths: list[float]: lengths of each link in the arm
        viz: ArmViz | None: optional visualization object
    Returns:
        error: float value representing the distance away from the goal pose
            that the current pose is
    """
    # finds radian values for the arm configuration
    config_rad = angles(configuration)

    # creates an armsim object using the config and links
    arm = ArmSim(config_rad, link_lengths, viz)

    # performs forward kinematics on the arm, finding the pose of the ee
    # given the current thetas
    all_joints_pose = arm.fk()  # all joint poses, last element is ee pose
    ee_pose = all_joints_pose[-1]

    # calculates manhattan distance error between the goal and current pose
    error = abs(goal_pose[0] - ee_pose[0]) + abs(goal_pose[1] - ee_pose[1])
    return error


def euclidean_error(
    configuration: Configuration,
    goal_pose: list[float],
    link_lengths: list[float],
    viz: ArmViz | None = None,
) -> float:
    """
    calculates the euclidean error between the goal pose and the current pose

    Args:
        configuration: list[list[int]]: arm configuration to calc error
        goal_pose: list[float]: the goal pose to reach
        link_lengths: list[float]: lengths of each link in the arm
        viz: ArmViz | None: optional visualization object
    Returns:
        error: float value representing the distance away from the goal pose
            that the current pose is
    """
    # finds radian values for the arm configuration
    config_rad = angles(configuration)

    # creates an armsim object using the config and links
    arm = ArmSim(config_rad, link_lengths, viz)

    # performs forward kinematics on the arm, finding the pose of the ee
    # given the current thetas
    all_joints_pose = arm.fk()  # all joint poses, last element is ee pose
    ee_pose = all_joints_pose[-1]

    # calculates euclidean error between the goal and current pose
    error = math.dist(goal_pose, ee_pose)
    return error


def tournament_parent_select(
    population: list[Configuration],
    fitness_func: Fitness_func,
    goal_pose: list[float],
    link_lengths: list[float],
) -> list[Configuration]:
    """
    given current population, selects 100 new parents using tournament selection

    Args:
        population: list[list[int]]: current population of arm configurations
        fitness_func: Fitness_func: function to calculate fitness of each config
        goal_pose: list[float]: the goal pose to reach
        link_lengths: list[float]: lengths of each link in the arm
    Returns:
        selected_parents: list of arm configurations
    """

    # sort the population based on fitness
    sorted_population = population.copy()
    sorted_population.sort(
        key=lambda configuration: fitness_func(
            configuration, goal_pose, link_lengths, None
        )
    )

    selected_parents = []
    # select 100 parents using tournament selection
    for _ in range(len(population)):
        tournament_indexes = random.sample(list(range(len(population))), 3)
        winner_index = min(tournament_indexes)
        selected_parents.append(sorted_population[winner_index])
    return selected_parents


def roulette_parent_select(
    population: list[Configuration],
    fitness_func: Fitness_func,
    goal_pose: list[float],
    link_lengths: list[float],
) -> list[Configuration]:
    """
    given current population, selects 100 new parents using roulette selection

    Args:
        population: list[list[int]]: current population of arm configurations
        fitness_func: Fitness_func: function to calculate fitness of each config
        goal_pose: list[float]: the goal pose to reach
        link_lengths: list[float]: lengths of each link in the arm
    returns:
        selected_parents: list of arm configurations
    """
    # calculates fitness of each configuration
    fitnesses = [
        fitness_func(configuration, goal_pose, link_lengths, None)
        for configuration in population
    ]
    # randomly selects a configuration to be a parent proportional to its fitness
    return [random.choices(population, fitnesses)[0] for _ in range(len(population))]


def joint_crossover(
    parent1: Configuration,
    parent2: Configuration,
    num_dof: int = 2,
    cross_prob: float = 0.85,
):
    """
    performs a crossover between two parents by swapping one whole joint
    with another parent's joint

    args:
        parent1: list[list[int]]: first arm configuration parent
        parent2: list[list[int]]: second arm configuration parent
        num_dof: int: The number of degrees of freedom of the desired robot arm
        cross_prob: float: probability of crossover occuring

    returns:
        child1: list[list[int]]: parent1 new (possibly unchanged) config
        child2: list[list[int]]: parent2 new (possibly unchanged) config
    """

    # if the crossover probability is not met, return the parents unchanged
    if not random.random() < cross_prob:
        return parent1, parent2

    # randomly select a crossover point
    cross_point = random.choice(range(0, num_dof - 1))

    # create the children by swapping the values of the parents at the crossover point
    child1 = parent1[:cross_point] + parent2[cross_point:]
    child2 = parent2[:cross_point] + parent1[cross_point:]

    return child1, child2


def uniform_crossover(
    parent1: Configuration,
    parent2: Configuration,
    num_dof: int = 2,
    cross_prob: float = 0.85,
) -> tuple[Configuration, Configuration]:
    """
    performs a crossover between two parents by randomly mixing the values for
    one joint

    args:
        parent1: list[list[int]]: first arm configuration parent
        parent2: list[list[int]]: second arm configuration parent
        num_dof: int: The number of degrees of freedom of the desired robot arm
        cross_prob: float: probability of crossover occuring

    returns:
        child1: list[list[int]]: parent1 new (possibly unchanged) config
        child2: list[list[int]]: parent2 new (possibly unchanged) config
    """

    # if the crossover probability is not met, return the parents unchanged
    if not random.random() < cross_prob:
        return parent1, parent2

    child1 = []
    child2 = []

    # for each dof, randomly select which parent's value to use
    for i in range(num_dof):
        if random.random() < 0.5:
            child1.append(parent2[i])
            child2.append(parent1[i])
        else:
            child1.append(parent1[i])
            child2.append(parent2[i])

    return child1, child2


def mutation(
    configuration: Configuration,
    num_dof: int = 2,
    bits_per_theta: int = 16,
    mutation_prob: float = 0.35,
) -> Configuration:
    """
    Performs a mutation based on the mutation probability

    Args:
        configuration: list[list[int]]: one arm configuration to be mutated
        num_dof: int: The number of degrees of freedom of the desired robot arm
        bits_per_theta: int: The number of bits to use to represent each angle
        mutation_prob: float: probability of mutation occuring

    Returns:
        new_configuration: list[list[int]]: mutated arm configuration
    """

    # calculates probability of each bit being flipped
    mutation_prob_each_bit = 1 - (1 - mutation_prob) ** (1 / (num_dof * bits_per_theta))

    new_configuration: Configuration = []

    # for each joint, randomly select a bit to flip
    for i, joint in enumerate(configuration):
        flip_bit_string = "".join(
            [
                "1" if random.random() < mutation_prob_each_bit else "0"
                for _ in range(bits_per_theta)
            ]
        )

        # flips the bit
        new_configuration.append(joint ^ int(flip_bit_string, 2))

    return new_configuration


def weighted_mutation(
    configuration: Configuration,
    num_dof: int = 2,
    bits_per_theta: int = 16,
    mutation_prob: float = 0.75,
) -> Configuration:
    """
    Decides whether to mutate the configuration
    Once decided, flips a bit in the configuration, where lesser significant
    bits are more likely to be flipped

    Args:
        configuration: list[list[int]]: current arm config to mutate
        num_dof: int: The number of degrees of freedom of the desired robot arm
        bits_per_theta: int: The number of bits to use to represent each angle
        mutation_prob: float: probability of mutation occuring

    Returns:
        new_configuration: list[list[int]]: mutated arm configuration
    """

    # calculates probability of each bit being flipped
    mutation_prob_each_joint = 1 - (1 - mutation_prob) ** (1 / (num_dof))

    new_configuration: Configuration = []

    for i, joint in enumerate(configuration):
        # if the mutation probability is not met, append the unchanged joint
        if not random.random() < mutation_prob_each_joint:
            new_configuration.append(joint)
            continue

        # makes the weighting for each bit
        bit_weight = [i for i in reversed(range(0, bits_per_theta))]
        mutation_magnitude = random.choices(
            range(0, bits_per_theta),
            weights=bit_weight,
        )[0]

        new_configuration.append(joint ^ 2**mutation_magnitude)

    return new_configuration


def numerical_mutation(
    configuration: Configuration,
    num_dof: int = 2,
    bits_per_theta: int = 16,
    mutation_prob: float = 0.75,
) -> Configuration:
    """
    Mutates the arm configuration by randomly adding a small value to each joint

    Args:
        configuration: list[list[int]]: current arm config to mutate
        num_dof: int: The number of degrees of freedom of the desired robot arm
        bits_per_theta: int: The number of bits to use to represent each angle
        mutation_prob: float: probability of mutation occuring

    Returns:
        list[list[int]]: mutated arm configuration
    """

    # if the mutation probability is not met, return the unchanged configuration
    if not random.random() < mutation_prob:
        return configuration

    # randomly adds a nudge to each joint
    return [
        (num + random.randint(-1024, 1024)) % 2**bits_per_theta for num in configuration
    ]


def survivor_select(
    parents: list[Configuration],
    children: list[Configuration],
    fitness_func: Fitness_func,
    goal_pose: list[float],
    link_lengths: list[float],
) -> list[Configuration]:
    """
    determines which of the parent and offspring survive based on fitness.
    the top 90% of children will survive, and the 10% of parents survive

    args:
        parents: list[list[list[int]]]: list of parents
        children: list[list[list[int]]]: list of children
        fitness_func: Fitness_func: function to calculate fitness of each config
        goal_pose: list[float]: the goal pose to reach
        link_lengths: list[float]: lengths of each link in the arm

    returns:
        survivors: list[list[list[int]]]: list of arm configurations that
        survive to the next generation
    """
    # sort the parents and children based on their fitness
    parents_sorted = sorted(
        parents, key=lambda x: fitness_func(x, goal_pose, link_lengths, None)
    )

    children_sorted = sorted(
        children, key=lambda x: fitness_func(x, goal_pose, link_lengths, None)
    )

    # select the top 10% of parents
    survivors = [parents_sorted[i] for i in range(round(0.1 * len(parents_sorted)))]
    # select the top 90% of children
    survivors += [children_sorted[i] for i in range(round(0.9 * len(children_sorted)))]

    return survivors


def elitism(
    parents: list[Configuration],
    children: list[Configuration],
    fitness_func: Fitness_func,
    goal_pose: list[float],
    link_lengths: list[float],
) -> list[Configuration]:
    """
    Selects new parents by sorting all individuals based on fitness and selecting the
    top 50% of them (assuming parents and children have same size)

    Args:
        parents: list[list[int]]]: list of parents
        children: list[list[int]]]: list of children
        fitness_func: Fitness_func: function to calculate fitness of each config
        goal_pose: list[float]: the goal pose to reach
        link_lengths: list[float]: lengths of each link in the arm

    Returns:
        survivors: list[list[int]]]: list of arm configurations that
        survive to the next generation
    """
    # sort the parents and children together based on their fitness
    pop_size = len(parents)
    all_individuals = parents + children
    all_individuals.sort(key=lambda x: fitness_func(x, goal_pose, link_lengths, None))

    # select the top 50% of individuals and returns them
    return all_individuals[:pop_size]


def terminate(
    population: list[Configuration],
    current_generation: int,
    fitness_func: Fitness_func,
    goal_pose: list[float],
    link_lengths: list[float],
    max_generations: int = 500,
    terminate_tol: float = 0.01,
) -> bool:
    """
    determines whether the GA has gotten close enough to the best solution
    to terminate. Based on generation count and accuracy within a tolerance

    Args:
        population: list[list[int]]: current population of arm configurations
        current_generation: int: the current generation count
        fitness_func: Fitness_func: function to calculate fitness of each config
        goal_pose: list[float]: the goal pose to reach
        link_lengths: list[float]: lengths of each link in the arm
        max_generations: int: maximum number of generations to run
        terminate_tol: float: tolerance for the best solution

    Returns:
        bool: True if the GA should terminate, False otherwise
    """
    # checks if the generation count has reached the max
    if current_generation >= max_generations:
        return True

    # checks if the best solution is within the tolerance
    best_error = min(
        [
            fitness_func(configuration, goal_pose, link_lengths, None)
            for configuration in population
        ]
    )
    if best_error <= terminate_tol:
        return True

    return False
