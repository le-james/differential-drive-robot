using LinearAlgebra
using JuMP
using Ipopt
using PyPlot

# build nonliner program (nlp) in Model object using ipopt optimization
DDR = Model(Ipopt.Optimizer)


#= sim parameters =#
h = 0.05                    # 20hz sample time
simT = 20                   # Sim time
tSteps = Int(simT/h)+1      # Num of time steps 
t = [0:h:simT;]             # time increments
#= sim parameters =#


#= initial conditions and goal pose =#
robotRadius = 0.2
# initial state conditions
x0 = [0; 0; 0*pi/180]       # x y θ initial  

# final state conditions
xr = [3; 3; 180*pi/180]       # x y θ reference

# obstacle position
obs = [2.5, 2.5]
obsRadius = 0.25
#= initial conditions and goal pose =#


#= nlp variable setup =#
# not used in solving nlp
x = @variable(DDR, base_name="x")
y = @variable(DDR, base_name="y")
θ = @variable(DDR, base_name="θ")
v = @variable(DDR, base_name="v")
ω = @variable(DDR, base_name="ω")

# store state and control variables in array
states = [x; y; θ]
controls = [v; ω]

# number of states and controls
nStates = length(states)
nControls = length(controls)

# decision variables for ipopt to solve for - used in solving nlp
X = @variable(DDR, X[1:nStates, 1:tSteps])
U = @variable(DDR, U[1:nControls, 1:tSteps])
#= nonlinear program variable setup =#


#= using linear quadratic regulator cost function =#
# state and control weights
Qf = 10
Qnf = 1
R = 1
Q = Diagonal(Qf*ones(nStates, nStates))
Qn = Diagonal(Qnf*ones(nStates, nStates))
R = Diagonal(R*ones(nControls, nControls))

# stage cost only - trying to control the acceleration and deceleration but both obj func seems to have the same results
@objective(DDR, Min, sum(0.5*(X[:,i]-xr)'*Q*(X[:,i]-xr) + 0.5*U[:,i]'*R*U[:,i] for i=1:tSteps))
# objective function with terminal cost included
# @objective(DDR, Min, sum(0.5*(X[:,i]-xr)'*Q*(X[:,i]-xr) + 0.5*U[:,i]'*R*U[:,i] + 0.5*X[:,tSteps]'*Qn*X[:,tSteps] for i=1:tSteps))

# set final conditions - we don't actually need this final bounds, it's taken care of in the obj func
# IT ACTUALLY DOES AFFECT CONVERGENCE - less compute time if not used - not sure why?
# fix(X[1,end], xr[1]; force = true)
# fix(X[2,end], xr[2]; force = true)
# fix(X[3,end], xr[3]; force = true)
#= using linear quadratic regulator cost function =#


#= constaints =#
# inequality constraints
# bounds of x and y position
# set_lower_bound.(X[1:2,:], -5)
# set_upper_bound.(X[1:2,:], 5)
# OR use - same thing
@constraint(DDR, [i = 1:tSteps], -5 <= X[1,i] <= 5)
@constraint(DDR, [i = 1:tSteps], -5 <= X[2,i] <= 5)

# bounds of x and y velocity
# set_lower_bound.(U[1,:], -0.6)
# set_upper_bound.(U[1,:], 0.6)
# OR use - same thing
@constraint(DDR, [i = 1:tSteps], -0.6 <= U[1,i] <= 0.6)

# bounds of angular velocity
# set_lower_bound.(U[2,:], -pi/4)
# set_upper_bound.(U[2,:], pi/4)
# OR use - same thing
@constraint(DDR, [i = 1:tSteps], -pi/4 <= U[2,i] <= pi/4)

# obstacle constraint
for i = 1:tSteps
    @NLconstraint(DDR, -sqrt((X[1, i]-obs[1])^2 + (X[2, i]-obs[2])^2) + (robotRadius + obsRadius) <= 0)
end

# ddr kinematics
@NLexpressions(DDR, 
    begin
        xdot[i = 1:tSteps], U[1,i]*cos(X[3,i])
        ydot[i = 1:tSteps], U[1,i]*sin(X[3,i])
        θdot[i = 1:tSteps], U[2,i]
    end
)

# equality constraints - discretize trajectory using trapezoidal collocation
for i = 2:tSteps
    @NLconstraint(DDR, X[1, i] == X[1, i-1] + 0.5 * h * (xdot[i] + xdot[i-1]))
    @NLconstraint(DDR, X[2, i] == X[2, i-1] + 0.5 * h * (ydot[i] + ydot[i-1]))
    @NLconstraint(DDR, X[3, i] == X[3, i-1] + 0.5 * h * (θdot[i] + θdot[i-1]))
end
#= constaints =#


#= set inital conditions in decision variables - needs to be after constraints or it won't work for some reason =#
# set inital state from above
fix(X[1,1], x0[1]; force = true)
fix(X[2,1], x0[2]; force = true)
fix(X[3,1], x0[3]; force = true)

# set inital control inputs - not needed
fix(U[1,1], 0; force = true)
fix(U[2,1], 0; force = true)
#= set inital conditions in decision variables =#


#= solve the nonlinear program =#
println("Solving...")
optimize!(DDR)

# pull out the optimal states and control input from the solution
stateSol = zeros(nStates,tSteps)
stateSol[:,1:tSteps] = value.(X[:,1:tSteps])

controlSol = zeros(nControls,tSteps)
controlSol[:,1:tSteps] = value.(U[:,1:tSteps])
# = solve the nonlinear program =#


#= Set up visualization - visualize optimal trajectory =#
using TrajOptPlots
using MeshCat
using StaticArrays
using RobotZoo:DubinsCar
using Colors

# launch visualizer
vis = Visualizer()
render(vis)

# instantiate the dubinscar model
uni = DubinsCar()

# add robots to the visualizer
TrajOptPlots.set_mesh!(vis, uni)                                        # first robot
TrajOptPlots.set_mesh!(vis["robot_copy"], uni, color=colorant"red")     # second robot

# put optimal trajectory from ipopt into static array
traj = [SVector{3}(x) for x in eachcol(stateSol)];

# change second robot pose (obstacle)
x = SA[obs[1], obs[2], 0]
visualize!(vis["robot_copy"], uni, x)

# run trajectory in visualizer
visualize!(vis, uni, simT, traj)
#= Set up visualization =#


# plot optimal control inputs
fig, ax = plt.subplots()
ax.plot(t, controlSol[1,:], label="Velocity") 
ax.set_xlabel("Time") 
ax.set_ylabel("Velocity") 
ax.set_title("Optimal Velocity Control Input [m/s]")    # check to statisfy linear velocity constraints of 0.6m/s
ax.legend()
plt.show()