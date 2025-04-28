import helpers
import ga_alts as ga
from ArmSim import ArmSim, ArmViz


def run_ga(
    goal_pose: list[float],
    link_lengths: list[float],
    initialization: callable,
    fitness_func: callable,
    parent_select: callable,
    crossover: callable,
    mutation: callable,
    survivor_select: callable,
    termination: callable,
    rad: bool = False,
    cross_prob: float = 0.85,
    mutation_prob: float = 0.15,
    population_size: int = 500,
    num_dof: int = 2,
    bits_per_theta: int = 16,
    terminate_tol: float = 0.001,
    max_generations: int = 100,
):
    """
    runs the whole ga based on the parameters passed into the init
    """
    # generate the initial population
    population = initialization(population_size, num_dof, bits_per_theta)
    current_generation = 0
    best_of_gen = []
    viz = ArmViz()
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
            ),
        )
        current_best_error = fitness_func(current_best, goal_pose, link_lengths, viz)
        viz.update()
        best_of_gen.append(f"{current_best_error=}")
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
    return helpers.bitstring_to_rad(best_solution) if rad else best_solution


soln = run_ga(
    [1.0, 1.0],
    [1.0, 1.0],
    initialization=ga.generate_population,
    fitness_func=ga.error,
    parent_select=ga.parent_select,
    crossover=ga.crossover,
    mutation=ga.weighted_mutation,
    survivor_select=ga.survivor_select,
    termination=ga.terminate,
    rad=True,
)
