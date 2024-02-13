Remark: The paper accompanying this repository is under review.

# MCMO

Monte Carlo Multilevel Optimization (MCMO) is a gradient free sampling based optimization algorithm for solving Multilevel Optimization problems that commonly occur in fields such as Controls, Supply Chain and Logistics, Economics, etc. Some of the key features of this algorithms are:
- It can handle problems with unbounded decision variables and, thus, is applicable to a wider class of problems.
- It can handle problems with non-differentiable objectives, as long as the final objective is differentiable.
- It can handle equality constraints present at the final level, unlike other Meta-heuristic algorithms, which fail to handle any equality constraints at all without any reformulations.
- It does not require any reformulations of the objective functions and, thus, can solve problems that can’t be approached via KKT or Value function based reformulations.
- It’s an anytime algorithm and can be tuned to obtain arbitrary accuracy at the expense of computation
  
Further details about the algorithm can be found in the following preprint: https://arxiv.org/abs/2312.03282 .

# Usage
## Constructing a Multilevel Problem
A Multilevel Problem is an instance of the structure `MultilevelProblem` that can be used by including the file `Problems.jl` inside the `Problems` subdirectory. An example usage is shown below:

```julia
include("Problems/Problems.jl")
num_levels = 3
custom_ml_problem = MultiLevelProblem(num_levels)
```
After a MultiLevelProblem has been defined, levels can be added to it by using the function `addLevel!` which takes the parameter a structure of type `Problem`. Each `Problem` consists of objective to be minimized, constraints, index set that the level controls, and number of samples to be generated for the particular problem. All objective and constraints are defined as functions where objective functions take argument `X` and return real `f(X)`. Similarly, all constraint functions take argument `X` and return vector `g(X)` such that constraint satisfaction is indicated by whether `g(X) .>= 0`. For example, adding a level to the multi-level problem defined above would be:

```julia
function f1(X)
  # should return a real number to be minimized
end

function g1(X)
  # should return a vector [g1, g2, g3, ...] where constraint satisfaction occurs if all g1 >=0, g2>=0 ... gn>=0
end

index_set1 = [2, 3]    # this particular level only controls the second and the third variable in the variable set X
problem1 = Problem(f1, g1, index_set1, 5)    # this will generate 5 samples per iteration
problem2 = ...                               # define problem2 and problem3 similarly with its own objective and constraintset and other parameters
problem3 = ...
custom_ml_problem.addLevel!(problem1)
custom_ml_problem.addLevel!(problem2)
custom_ml_problem.addLevel!(problem3)
custom_ml_problem.alpha = 1                  # Global step size
custom_ml_problem.xs = [...]                 # define initial value of X. Must satisfy all constraints to be initially feasible.
custom_ml_problem.MAX_ITER = 100             # define maximum number of iteration
```
Once custom Multilevel Problem has been defined, the Trajectory.jl file has to be updated to inform it of the problem to be optimized. This can be done by including the file where the custom multilevel problem has been defined in Trajectory.jl, and then ensuring that the problem variable `p` points to `custom_ml_problem` as follows:

```julia
# In file Trajectory.jl
...
include("CustomProblem.jl")  # Assuming customproblem is defined here.
...
...
p = custom_ml_problem      # Assign custom_ml_problem to p ensuring that this is the only assignment to p.
...
```

## Running the Optimization
By first entering into the main directory of the repository, start by opening up julia in project mode:
`julia --project=.`

Once Julia starts, press `]` to go into package mode and enter `instantiate`. This will download and precompile the dependencies used by the project. Once the dependencies are ready, come out of the package mode by pressing Backspace. The optimization can then be run by:

`include("debugit.jl")`

The optimization will run and print all intermediate progress. Depending upon the approximation parameter `K` set in `Trajectory.jl`, the final approximation will be printed out after the maximum iteration ends.

