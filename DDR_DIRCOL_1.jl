using LinearAlgebra
using JuMP
using Ipopt
using PyPlot

DDR = Model(Ipopt.Optimizer)

#initial state conditions
x0 = [0; 0; 0*pi/180]       #x y θ initial  

#final state conditions
xr = [2; -2; 90*pi/180]    #x y θ reference

#Sim parameters
h = 0.05                  #20hz sample time
simT = 10               #Sim time
tSteps = Int(simT/h)+1   #Num of time steps 

# didn't work? created individual variables and bounds below and it worked
# create N decision variables, control variables with lower and upper bounds
# @variables(DDR, 
#     begin
#         0 ≤ x[1:tSteps] ≤ 10
#         0 ≤ y[1:tSteps] ≤ 10
#         0 ≤ θ[1:tSteps] ≤ 2*pi
#         0 ≤ v[1:tSteps] ≤ 0.3;           #0.3m/s
#         0 ≤ ω[1:tSteps] ≤ 0.01308997     #0.75deg/s
#     end
# )
x = @variable(DDR, base_name="x")
y = @variable(DDR, base_name="y")
θ = @variable(DDR, base_name="θ")
v = @variable(DDR, base_name="v")
ω = @variable(DDR, base_name="ω")

# #state and control variables in array
states = [x; y; θ]
controls = [v; ω]

#number of states and controls
nStates = length(states)
nControls = length(controls)

#decision variables
X = @variable(DDR, X[1:nStates, 1:tSteps])
U = @variable(DDR, U[1:nControls, 1:tSteps])

#bounds of x and y position
set_lower_bound.(X[1:2,:], -5)
set_upper_bound.(X[1:2,:], 5)

#bounds of velocity and angular velocity
set_lower_bound.(U[1,:], -0.6)
set_upper_bound.(U[1,:], 0.6)

set_lower_bound.(U[2,:], -pi/4)
set_upper_bound.(U[2,:], pi/4)

# constraints this way doesn't work
# @constraints(DDR, begin
#     for i=1:tSteps
#         X[1, i] <= 5
#     end
# end)
# @constraint(DDR, c1, X[1:2,i] for i=1:tSteps >= -5.0)
# @constraint(DDR, c1, -5 <= X[1] <= 5.0)
# @constraint(DDR, c2, -5 <= X[2] <= 5.0)

# THIS WAY WORKS!
# @constraint(DDR, [i = 1:tSteps], -5 <= X[1,i] <= 5)



# all_variables(DDR)



#state and control weights
Qf = 1
Qnf = 1
R = 1
Q = Diagonal(Qf*ones(nStates, nStates))
Qn = Diagonal(Qnf*ones(nStates, nStates))
R = Diagonal(R*ones(nControls, nControls))

#integral cost function - doesnt work, not supported by jump - the below works now
#sum([v[i] ω[i]]*[v[i] ω[i]]'

#stage cost only - trying to control the acceleration and deceleration but both obj func seems to have the same results
# @objective(DDR, Min, sum(0.5*(X[:,i]-xr)'*Q*(X[:,i]-xr) + 0.5*U[:,i]'*R*U[:,i] for i=1:tSteps)) #i starting at 1 or 2 doesn't make a difference
@objective(DDR, Min, sum(0.5*(X[:,i]-xr)'*Q*(X[:,i]-xr) + 0.5*U[:,i]'*R*U[:,i] + 0.5*X[:,tSteps]'*Qn*X[:,tSteps] for i=1:tSteps)) #i starting at 1 or 2 doesn't make a difference
# @objective(DDR, Min, sum((X[:,i]-xr)'*Q*(X[:,i]-xr) + (U[:,i]-U[:,i-1])'*R*(U[:,i]-U[:,i-1]) for i=2:tSteps))

#print the current objective function
# objective_function(DDR)

#set inital state
fix(X[1,1], x0[1]; force = true)
fix(X[2,1], x0[2]; force = true)
fix(X[3,1], x0[3]; force = true)

#set inital control inputs
# fix(U[1,1], 0; force = true)
# fix(U[2,1], 0; force = true)

#set final conditions - we don't actually need this final bounds, it's taken care of in the obj func - IT ACTUALLY DOES AFFECT CONVERGENCE
fix(X[1,end], xr[1]; force = true)
fix(X[2,end], xr[2]; force = true)
fix(X[3,end], xr[3]; force = true)

#dont work
# @NLexpression(DDR, s[i = 1:2], sin(U[2,i]))
# @expression(DDR, f[i = 1:2], [ U[1,i]X[3,i] U[1,i]X[3,i] U[2,i] ])
# @NLexpression(DDR, f[i = 1:2], U[1,i]*cos(X[3,i]) )

# @expression(DDR, f[i = 1:2], [xdot ydot θdot])

# function rhs()
#     xdot[i = 1:2] = U[1,i]*cos(X[3,i])
#     ydot[i = 1:2], U[1,i]*sin(X[3,i])
#     θdot[i = 1:2], U[2,i]
# end

# ddr kinematics
@NLexpressions(DDR, 
    begin
        xdot[i = 1:tSteps], U[1,i]*cos(X[3,i])
        ydot[i = 1:tSteps], U[1,i]*sin(X[3,i])
        θdot[i = 1:tSteps], U[2,i]
    end
)

# creating state variables - using trapezoidal collocation
for i = 2:tSteps
    @NLconstraint(DDR, X[1, i] == X[1, i-1] + 0.5 * h * (xdot[i] + xdot[i-1]))
    @NLconstraint(DDR, X[2, i] == X[2, i-1] + 0.5 * h * (ydot[i] + ydot[i-1]))
    @NLconstraint(DDR, X[3, i] == X[3, i-1] + 0.5 * h * (θdot[i] + θdot[i-1]))
end


# print(DDR)


println("Solving...")
optimize!(DDR)
# solution_summary(model)
# value.(obj)[:]

# @show termination_status(model)
# @show primal_status(model)
# @show dual_status(model)
# @show objective_value(model)
# @show value(x)


# plot(value.(X)[1,:],value.(X)[2,:])

#pull out the optimal states from the solution
stateSol = zeros(nStates,tSteps)
stateSol[:,1:tSteps] = value.(X[:,1:tSteps])

# size(value.(X))

#Set up visualization
using TrajOptPlots
using MeshCat
using StaticArrays
using RobotZoo:DubinsCar
using Colors

vis = Visualizer()
render(vis)

uni = DubinsCar()
TrajOptPlots.set_mesh!(vis, uni)

traj = [SVector{3}(x) for x in eachcol(stateSol)];
visualize!(vis, uni, simT, traj)