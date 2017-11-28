Run the code:

1. Just open the matlab script called main.m and run it.

Included Matlab scripts:

standardcar.m - Defines a struct containing standard information for the vehicles.

roadsegment.m - Defines a struct containing standard information of the road.

main.m - Defines the scenario you want to run, invokes functions to solve the problem and plot the solution.

init.m - Defines the minimum and maximum lateral limits (i.e., the overtaking window and the critical zone).

yalmipsolve.m - Given the vehicle states, overtaking window and critical zone, calculates the optimal state and control
vectors to perform an overtaking.

plotting.m - Plots the solution, including the positions, velocities and accelerations.

NOTES:

I. This solves the problem only once, i.e., no MPC is applied here.

II. The problem is here modeled using the YALMIP language (is you can see in yalmipsolve is an easy way to formulate
these types of problems), probably you will need to rewrite this on a slightly different form to make it fit in
Simulink.

III. The problem is solved using a variant of so called sequential quadratic programming. Basically this means linearizing
all non-convex constraints (using a first order taylor-expansion) and solving iterativly by updating the linearization with the
latest optimal solution. Extra terms are added in the objective to force faster convergence (called hessian and gradient_term in
the code).