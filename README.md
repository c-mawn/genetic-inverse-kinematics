# Genetic Inverse Kinematics
Finding inverse kinematic solutions for over-actuated 2D robot arms using genetic algorithms. A project for Advanced Algorithms at Olin College of Engineering by [Charlie Mawn](https://github.com/c-mawn) and [Dominic Salmieri](https://github.com/joloujo).

## Writeup

### Genetic Algorithms
<!-- Introduce the algorithm(s) you are exploring and the background -->
Genetic algorithms are a subset of evolutionary algorithms, and take inspiration from natural selection and genetics<sup>[2]</sup>. They are often used to approximate optimal solutions to optimization problems with especially complex search spaces since they can be faster and more resistant to local minimums than alternatives like gradient descent.

Genetic algorithms treat possible solutions as individuals in a population, and then simulate the natural selection of these individuals over many generations. Each individual is represented by a 'chromosome', which is most commonly a string of bits.

Generically, genetic algorithms look like the following diagram. 

![Genetic algorithm diagram](media/GA_diagram.png)
*Figure 1: Genetic algorithm diagram<sup>[1]</sup>* 
<!-- TODO: Needs citation -->

First, the population is initialized. Then the algorithm enters a loop of selecting parents, creating children, slightly changing those children, and reselecting the population. The algorithm leaves this loop when it meets it's termination criteria.

The core of a genetic algorithm is it's fitness function, which is a measure of how good a solution is. By finding the fitness of all of the individuals in the population, the algorithm can compare individuals and find ones that are closer to a good solution. The fitness function can be used in any of the previously mentioned steps.

<!-- Introduce the context you are exploring your algorithm in -->

### Robot Arms and Genetic Algorithms

Oftentimes, with a robot arm, you want to make the end effector go to a specific location or pose. The problem with this is that you only have control over the angles of the individual joints, so you need to somehow calculate the proper joint angles that will get the end effector to the location you want. This is called inverse kinematics.

There are several ways to solve the inverse kinematics of a robot. You can solve trigonometrically, and get an exact analytical solution, but this is only possible for certain robot arm configurations. Plus, if the number of degrees of freedom are different than your number of joints, you'll either have impossible cases or cases where there are infinite solutions. You can solve numerically by doing gradient descent, but this is susceptible to local minimums. 

We wanted to try using genetic algorithms to find solutions, which we hoped would be faster and more reliable than these other methods. Not only that, our solution would be able to handle over-actuated robot arms, which can have infinite valid solutions for one location.


### Solving YOUR Problem / How It Works
- Explanation of components of the algorithm
- Detailed contextualization of the algorithm to the problem, including detailed descriptions of any named variables
- Detailed explanation of encoding the problem
- At least two diagrams or other form of detailed description

### Results 
- Visualization of results provided in some format
- Any technical reader can garner understanding of what your algorithm’s implementation produced

### Analysis
- Description of what results mean
- Examples:
  - Graphs showing comparisons of varying results
  - Performance metrics
- Shows and analyzes how good the solutions are and how quickly solutions are found
- Comparison of methods used with results to back up

### Next Steps
- Delineation of potential next steps
  - Areas you would have explored with more time
  - How your solutions may scale to other problems

### Citations
<!-- Bibliography contains at least 5 sources total cited in any official format; just be consistent (use IEEE if you cannot think of any) -->
<!-- In-line citations matching selected official format style -->
[1] R. Dave, A. Rattan Singh Sandhu, A. Vinod, and M. Cranor, “Day 0 Genetic Algorithms-2,” in Special Topics in Computing - Advanced Algorithms with Math, Apr. 4, 2025 

[2] GeeksforGeeks, “Genetic algorithms,” GeeksforGeeks, https://www.geeksforgeeks.org/genetic-algorithms/ (accessed Apr. 29, 2025). 

### *Quality*
- *Clear narrative thread throughout the writeup*
- *No/minimal spelling & grammar mistakes*
- *Writing makes sense*
- *Writing is professional*
- *Writeup is portfolio ready!*
- *Clean, accessible to outsiders, highlights hard work*

## Setup

This project was developed and tested in Python 3.10+. We recommend using a virtual environment.

The necessary packages are `numpy` and `matplotlib`. `pip-tools` is an optional but recommended command line tool we use to manage package and subpackage versions.

**To install the necessary requirements, use `pip install -r requirements.txt` in the command line.**

If you change the requirements, you can rerun that command to install the new packages, or use `pip-sync` from pip-tools in the command line to also uninstall unnecessary packages or versions. If you need to add new packages, you can add them to `requirements.txt` directly, or just put the package name in `requirements.in` and use `pip-compile` from pip-tools in the command line to autogenerate a requirements file that ensures all package and subpackage versions work well with each other.

## Usage

After insalling necessary packages, run the algorithm by using `python3 src/main.py` in the command line. 

To change parameters of the genetic algorithm, go to the bottom of `main.py` and alter the arguments passed into the `run_ga` function. 
