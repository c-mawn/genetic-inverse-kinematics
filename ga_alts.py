import random
import math
from ArmSim import ArmSim, ArmViz
from typing import Callable
from helpers import Configuration, Fitness_func, angles

# TODO: Write other methods of each step
# (need to do survivor select, and terminate)


def random_initial_thetas(
    num_dof: int = 2, bits_per_theta: int = 16
) -> Configuration:
    """
    initializes a set of theta values randomly.
    each theta is represented by a list of bits_per_theta bits,
    which ranges from 0-2pi

    Args:
        num_dof (int): The number of degrees of freedom of the desired robot arm
        bits_per_theta (int): The number of bits to use to represent each angle

    Returns:
        thetas: list of ints each representing an angle in one arm configuration
    """
    return [random.randint(0, 2**bits_per_theta - 1) for _ in range(num_dof)]


def preset_initial_thetas(
    num_dof: int = 2, bits_per_theta: int = 16
) -> Configuration:
    """
    initializes a set of theta values set to all 0

    Args:
        num_dof (int): The number of degrees of freedom of the desired robot arm
        bits_per_theta (int): The number of bits to use to represent each angle

    Returns:
        thetas: list of ints each representing an angle in one arm configuration
    """
    return [0] * num_dof


# TODO: Just make a for look in the GA function
def generate_population(
    init_func: Callable,
    population_size: int = 500,
    num_dof: int = 2,
    bits_per_theta: int = 16,
) -> list[Configuration]:
    """
    generates an initial set of arm configurations randomly

    Returns:
        population: list[list[int]] : initial population set of thetas
    """
    population = []
    for _ in range(population_size):
        population.append(init_func(num_dof, bits_per_theta))
    return population


def error(
    configuration: Configuration,
    goal_pose: list[float],
    link_lengths: list[float],
    viz: ArmViz | None = None,
) -> float:
    """
    calculates the euclidean error between the goal pose and the current pose

    returns:
        error: float value representing the distance away from the goal pose
            that the current pose is
    """
    config_rad = angles(configuration)
    arm = ArmSim(config_rad, link_lengths, viz)
    all_joints_pose = arm.fk()  # all joint poses, last element is ee pose
    ee_pose = all_joints_pose[-1]

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

    Returns:
        selected_parents: list of arm configurations
    """

    sorted_population = population.copy()
    sorted_population.sort(
        key=lambda configuration: fitness_func(configuration, goal_pose, link_lengths, None)
    )

    selected_parents = []
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

    returns:
        selected_parents: list of arm configurations
    """

    fitnesses = [fitness_func(configuration, goal_pose, link_lengths, None) for configuration in population]
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

    returns:
        parent1: list[list[int]]: parent1 new (possibly unchanged) config
        parent2: list[list[int]]: parent2 new (possibly unchanged) config
    """

    if not random.random() < cross_prob:
        return parent1, parent2
    
    cross_point = random.choice(range(0, num_dof-1))

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

    returns:
        child1: list[list[int]]: parent1 new (possibly unchanged) config
        child2: list[list[int]]: parent2 new (possibly unchanged) config
    """

    if not random.random() < cross_prob:
        return parent1, parent2

    child1 = []
    child2 = []

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

    args:
        configuration: list[list[int]]: one arm configuration to be mutated
    """

    mutation_prob_each_bit = 1 - (1-mutation_prob)**(1/(num_dof * bits_per_theta))

    new_configuration: Configuration = []

    for i, joint in enumerate(configuration):
        flip_bit_string = ''.join([
            '1' if random.random() < mutation_prob_each_bit 
            else '0' 
            for _ in range(bits_per_theta)])
        
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
    """

    mutation_prob_each_joint = 1 - (1-mutation_prob)**(1/(num_dof))

    new_configuration: Configuration = []

    for i, joint in enumerate(configuration):
        
        bit_weight = [2**i for i in reversed(range(0, 16))]
        mutation_magnitude = random.choices(
            range(0, 16),
            weights=bit_weight,
        )[0]

        new_configuration.append(joint ^ 2**mutation_magnitude) 

    return new_configuration


def numerical_mutation(configuration: Configuration,
    num_dof: int = 2,
    bits_per_theta: int = 16,
    mutation_prob: float = 0.75,
) -> Configuration:
    
    if not random.random() < mutation_prob:
        return configuration
    
    return [(num + random.randint(-1024, 1024)) % 2**bits_per_theta for num in configuration]


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

    returns:
        survivors: list[list[list[int]]]: list of arm configurations that
        survive to the next generation
    """
    # sort the parents and children based on their fitness
    parents_sorted = sorted(
        parents, key=lambda x: fitness_func(x, goal_pose, link_lengths, None), reverse=True
    )

    children_sorted = sorted(
        children, key=lambda x: fitness_func(x, goal_pose, link_lengths, None), reverse=True
    )

    # select the top 10% of parents
    survivors = [parents_sorted[i] for i in range(round(0.1 * len(parents_sorted)))]
    # select the top 90% of children
    survivors += [children_sorted[i] for i in range(round(0.9 * len(children_sorted)))]

    return survivors


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
    """
    # checks if the generation count has reached the max
    if current_generation >= max_generations:
        return True

    # checks if the best solution is within the tolerance
    best_error = min([fitness_func(configuration, goal_pose, link_lengths, None) for configuration in population])
    if best_error <= terminate_tol:
        return True

    return False
