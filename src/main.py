import genetic_algorithm_functions as ga
from ArmSim import ArmSim, ArmViz
from typing import Callable
from helpers import Configuration, Fitness_func, angles
import time
import matplotlib.pyplot as plt

# Looks a bit messy, but explicitly defining the function args and returns here
# makes it much easier to use them in the function below


def run_ga(
    goal_pose: list[float],
    link_lengths: list[float],
    # theta initialization function:
    theta_init: Callable[[int, int], Configuration],
    # fitness function
    fitness_func: Fitness_func,
    # parent selection method
    parent_select: Callable[
        [
            list[Configuration],
            Fitness_func,
            list[float],
            list[float],
        ],
        list[Configuration],
    ],
    # crossover method
    crossover: Callable[
        [Configuration, Configuration, int, float],
        tuple[Configuration, Configuration],
    ],
    # mutation method
    mutation: Callable[[Configuration, int, int, float], Configuration],
    # survivor selection method
    survivor_select: Callable[
        [
            list[Configuration],
            list[Configuration],
            Fitness_func,
            list[float],
            list[float],
        ],
        list[Configuration],
    ],
    # termination method
    termination: Callable[
        [
            list[Configuration],
            int,
            Fitness_func,
            list[float],
            list[float],
            int,
            float,
        ],
        bool,
    ],
    cross_prob: float,
    mutation_prob: float,
    population_size: int,
    num_dof: int,
    bits_per_theta: int,
    terminate_tol: float,
    max_generations: int,
):
    """
    runs the whole ga based on the parameters passed into the init
    """
    # generate the initial population
    population = [theta_init(num_dof, bits_per_theta) for _ in range(population_size)]
    current_generation = 0
    best_of_gen = []
    viz = ArmViz((goal_pose[0], goal_pose[1]))
    # run the ga until termination
    while not termination(
        population,
        current_generation,
        fitness_func,
        goal_pose,
        link_lengths,
        max_generations,
        terminate_tol,
    ):
        # increment the generation count
        print(f"Generation: {current_generation}\n")
        current_best = min(
            population,
            key=lambda x: fitness_func(
                x,
                goal_pose,
                link_lengths,
                None,
            ),
        )
        current_best_error = fitness_func(current_best, goal_pose, link_lengths, viz)

        arm = ArmSim(angles(current_best), link_lengths, viz)
        viz.update()
        # time.sleep(0.05)

        best_of_gen.append(current_best_error)
        print(f"Current Best = {current_best} : {current_best_error} \n")
        current_generation += 1

        # select parents
        selected_parents = parent_select(
            population, fitness_func, goal_pose, link_lengths
        )
        # generate children
        children = []
        for i in range(0, len(selected_parents), 2):
            parent1 = selected_parents[i]
            parent2 = selected_parents[i + 1]

            child1, child2 = crossover(parent1, parent2, num_dof, cross_prob)

            child1 = mutation(child1, num_dof, bits_per_theta, mutation_prob)
            child2 = mutation(child2, num_dof, bits_per_theta, mutation_prob)

            children.append(child1)
            children.append(child2)

        # select survivors
        population = survivor_select(
            selected_parents, children, fitness_func, goal_pose, link_lengths
        )

    # return the best solution
    best_solution = min(
        population, key=lambda x: fitness_func(x, goal_pose, link_lengths, viz)
    )
    viz.update()
    print(best_of_gen)
    return best_of_gen


# Example usage of the run_ga function
soln = run_ga(
    [1.0, 1.5],
    [1.0, 1.0],
    theta_init=ga.preset_initial_thetas,
    fitness_func=ga.euclidean_error,
    parent_select=ga.tournament_parent_select,
    crossover=ga.joint_crossover,
    mutation=ga.numerical_mutation,
    survivor_select=ga.survivor_select,
    termination=ga.terminate,
    cross_prob=0.85,
    mutation_prob=0.15,
    population_size=50,
    num_dof=2,
    bits_per_theta=16,
    terminate_tol=0.001,
    max_generations=1000,
)

# Plotting the results
plt.plot(range(len(soln)), soln)
plt.title("GA with Mixed Survivor Selection")
plt.xlabel("Generations")
plt.ylabel("Distance to Goal Pose")
plt.show()
