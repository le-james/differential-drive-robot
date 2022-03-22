using LinearAlgebra
using ForwardDiff
using ControlSystems

#Sim parameters
h = 0.05   #20hz sample time
simT = 10    #Sim time
tSteps = UInt(simT/h)+1   #Num of time steps 
thist = Array(range(0,h*(tSteps-1), step=h));

#Reference point
xR = 1
yR = 1
θ = 90          #Degree
θR = θ*pi/180   #Degree to radians
refSt = [xR; yR; θR]  #Initial states: x, y, θ

#Reference control
# refU = [3; 0.25*pi/180]      #Control inputs: v, ω
refU = [1; 0*pi/180]      #Control inputs: v, ω

Nx = length(refSt)    #Number of states
Nu = length(refU)  #Number of controls

#DDR Unicycle Kinematics
function unicycle_Model(x, u)
    θ = x[3]
    v = u[1]
    ω = u[2]

    ẋ = v*cos(θ)
    ẏ = v*sin(θ)
    θ̇ = ω
    return [ẋ; ẏ; θ̇ ]
end

#Forward Euler integration
function fwd_Euler(x ,u)
    xn = x + h*unicycle_Model(x, u)
    return xn
end

#RK4 integration with zero-order hold on u
function unicycle_Model_rk4(x,u)
    f1 = unicycle_Model(x, u)
    f2 = unicycle_Model(x + 0.5*h*f1, u)
    f3 = unicycle_Model(x + 0.5*h*f2, u)
    f4 = unicycle_Model(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    return xn
end

# #Linearize system about some state and control point
#partial diff wrt to states: x, y, θ
A = ForwardDiff.jacobian(x -> unicycle_Model_rk4(x, refU), refSt) 
#partial diff wrt to controls: v, ω - B only depends on the state θ not control inputs
B = ForwardDiff.jacobian(u -> unicycle_Model_rk4(refSt, u), refU)

# A = [0 0 -v*sin(0);
#      0 0  v*cos(0);
#      0 0     0    ]
# B = [cos(0) 0;
#      sin(0) 0;
#        0    1 ]


Q = Array(5*I(Nx))        #State weights
R = Array(I(Nu))        #Control weights
#dlqr returns all zeros for some reason - this is because i need to solve for the discrete dynamcics using rk4
K = dlqr(A, B, Q, R)     #uses continuous dynamics

# print(K)

function LQR_controller(x)
        stateErr = [x[1]-xR, x[2]-yR, x[3]-θR]        #K needs to be negative
        # stateErr = [xR-x[1], yR-x[2], θR-x[3]]      #K needs to be positive
        u = -K*stateErr
    return u
end

#Simulation
uhist = zeros(Nu, tSteps)
# uhist = refU
xhist = zeros(Nx, tSteps)
xhist[:, 1] = [0 0 5*pi/180] #+ 1.2*randn(3)  #Random initial condition
# xhist[:, 1] = [-3; -5; 180*pi/180]
# xhist[:, 1] = refSt
# for k = 1:(tSteps-1) #Minus 1 cuz we added x0 into xhist
#     uhist[:,k] = LQR_controller(xhist[:,k])          #Optimal control action
#     xhist[:,k+1] = unicycle_Model_rk4(xhist[:, k], uhist[:,k])  #Optimal state
# end

print(xhist[:, 1])
# print(size(xhist[:, 1]))


# size(xhist)


# #Set up visualization
# using TrajOptPlots
# using MeshCat
# using StaticArrays
# using RobotZoo:DubinsCar
# using Colors

# vis = Visualizer()
# render(vis)

# model = DubinsCar()
# TrajOptPlots.set_mesh!(vis, model)

# X = [SVector{3}(x) for x in eachcol(xhist)];
# visualize!(vis, model, simT, X)



# using Plots
# pyplot()
# Plots.PyPlotBackend()
# plot(xhist[1,:], xhist[2,:])