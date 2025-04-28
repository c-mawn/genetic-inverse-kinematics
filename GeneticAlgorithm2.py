import random
import numpy as np
import math
from ArmSim import ArmSim, ArmViz
import time

Configuration = list[int]

joints = 5
bits = 16

link_lengths: list[float] = [1, 1, 1, 1, 1]
goal_position: tuple[float, float] = (2, 1)


def angles(configuration: Configuration) -> list[float]:
    return [num / 2**bits * 2 * np.pi for num in configuration]


def initialize() -> Configuration:
    return [random.randint(0, 2**bits-1) for _ in range(joints)]


def error(configuration: Configuration) -> float:
    arm = ArmSim(angles(configuration), link_lengths)
    joint_positions = arm.fk()
    ee_position = joint_positions[-1]
    return math.dist(goal_position, ee_position)


def parent_selection(population: list[Configuration]) -> tuple[Configuration, Configuration]:
    population.sort(key=error)
    parent1 = population[min([random.randint(0, len(population)-1) for _ in range(3)])]
    parent2 = population[min([random.randint(0, len(population)-1) for _ in range(3)])]

    return parent1, parent2

def crossover(parents: tuple[Configuration, Configuration]) -> tuple[Configuration, Configuration]:
    crossover_point = random.randint(0, joints-1)
    child1 = parents[0][:crossover_point] + parents[1][crossover_point:]
    child2 = parents[1][:crossover_point] + parents[0][crossover_point:]

    return child1, child2

def mutation(configuration: Configuration) -> Configuration:
    if random.random() < 0.5:
        return [(num + random.randint(-1024, 1024)) % 2**bits for num in configuration]
    else:
        return configuration

def genetic_algorithm():

    viz = ArmViz()

    pop_size = 100

    population = [initialize() for _ in range(pop_size)]

    for x in range(100):
        errors = [error(individual) for individual in population]
        min_i = errors.index(min(errors))

        print(population[min_i], errors[min_i])

        arm = ArmSim(angles(population[min_i]), link_lengths, viz)
        viz.update()
        # time.sleep(0.01)
        
        new_population = []

        while len(new_population) < pop_size:
            children = crossover(parent_selection(population))
            new_population.append(mutation(children[0]))
            new_population.append(mutation(children[1]))
        
        population = new_population    

genetic_algorithm()