# Genetic Inverse Kinematics
Finding inverse kinematic solutions for over-actuated 2D robot arms using genetic algorithms. A project for Advanced Algorithms at Olin College of Engineering by [Charlie Mawn](https://github.com/c-mawn) and [Dominic Salmieri](https://github.com/joloujo).

## Writeup

### Background of Algorithm
- Introduce the algorithm(s) you are exploring and the background
- Introduce the context you are exploring your algorithm in



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
- Bibliography contains at least 5 sources total cited in any official format; just be consistent (use IEEE if you cannot think of any)
- In-line citations matching selected official format style

### *Quality*
- *Clear narrative thread throughout the writeup*
- *No/minimal spelling & grammar mistakes*
- *Writing makes sense*
- *Writing is professional*
- *Writeup is portfolio ready!*
- *Clean, accessible to outsiders, highlights hard work*

## Setup

This project was developed and tested in Python 3.10+. We recommend using a virtual environment.

The only necessary package is `numpy`. `pip-tools` is an optional but recommended command line tool we use to manage package and subpackage versions.

**To install the necessary requirements, use `pip install -r requirements.txt` in the command line.**

If you change the requirements, you can rerun that command to install the new packages, or use `pip-sync` from pip-tools in the command line to also uninstall unnecessary packages or versions. If you need to add new packages, you can add them to `requirements.txt` directly, or just put the package name in `requirements.in` and use `pip-compile` from pip-tools in the command line to autogenerate a requirements file that ensures all package and subpackage versions work well with each other.

## Usage

After insalling necessary packages, run the algorithm by using `python3 src/main.py` in the command line. 

To change parameters of the genetic algorithm, go to the bottom of `main.py` and alter the arguments passed into the `run_ga` function. 
