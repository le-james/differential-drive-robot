using LinearAlgebra
using ForwardDiff
using ControlSystems

#Sim parameters
h = 0.05   #20hz sample time
simT = 1    #Sim time
tSteps = UInt(simT/h)+1   #Num of time steps 
thist = Array(range(0,h*(tSteps-1), step=h));

#Reference point
xR = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]
yR = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]
θR = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45]

refSt = zeros(3, 20)

for k=1:20
    refSt[1,k] = xR[k]
    refSt[2,k] = yR[k]
    refSt[3,k] = θR[k]*pi/180
end

#Reference control
refu = [1; 90*pi/180]      #Control inputs: v, ω

Nx = length(refSt[:,1])    #Number of states
Nu = length(refu)       #Number of controls

#DDR Unicycle Kinematics
function DDR_Kinematics(x, u)
    # xpos = x[1]   #Not used
    # y = x[2]      #Not used
    θ = x[3]
    v = u[1]
    ω = u[2]

    ẋ = v*cos(θ)
    ẏ = v*sin(θ)
    θ̇ = ω
    return [ẋ; ẏ; θ̇ ]
end


#Linearize system
A = zeros(3, 60)
# incStart = 1  #THIS DOESN'T WORK, SCOPE ERROR
# incEnd = 3
incStart = range(1, step=3, stop=60)
incEnd = range(3, step=3, stop=60)
for k=1:20
    A[:,incStart[k]:incEnd[k]] = ForwardDiff.jacobian(x -> DDR_Kinematics(x, refu), refSt[:,k]) 
    # incStart = incStart + 3   #THIS DOESN'T WORK, SCOPE ERROR
    # incEnd = incEnd + 3
end

#can use optimal controls from traj opt as a reference?
B = ForwardDiff.jacobian(u -> DDR_Kinematics(refSt, u), refu) 


Q = Array(I(Nx))        #State weights
R = Array(5*I(Nu))       #Control weights

K = zeros(2, 60)
for k=1:20
    K[:,incStart[k]:incEnd[k]] = lqr(A[:,incStart[k]:incEnd[k]], B, Q, R)    #dlqr returns all zeros for some reason - 2x3
end


#Forward Euler integration
function fwd_Euler(x ,u)
    xn = x + h*DDR_Kinematics(x, u)
    return xn
end

#RK4 integration with zero-order hold on u
function DDR_rk4(x,u)
    f1 = DDR_Kinematics(x, u)
    f2 = DDR_Kinematics(x + 0.5*h*f1, u)
    f3 = DDR_Kinematics(x + 0.5*h*f2, u)
    f4 = DDR_Kinematics(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    return xn
end

function LQR_controller(x, ref, gain)
        xR = ref[1]
        yR = ref[2]
        θR = ref[3]
        K = gain
        stateErr = [x[1]-xR, x[2]-yR, x[3]-θR]      #K needs to be negative
        # stateErr = [xR-x[1], yR-x[2], θR-x[3]]      #K needs to be positive
        u = -K*stateErr
    return u
end

#Simulation
uhist = zeros(Nu, tSteps)
# uhist = refu
xhist = zeros(Nx, tSteps)
xhist[:, 1] = [0; 0; 0*pi/180] #+ 0.5*randn(3)  #Random initial condition
# xhist[:, 1] = refSt
for k = 1:(tSteps-1) #Minus 1 cuz we added x0 into xhist
    uhist[:,k] = LQR_controller(xhist[:,k], refSt[:,k], K[:,incStart[k]:incEnd[k]])          #Optimal control action
    xhist[:,k+1] = DDR_rk4(xhist[:, k], uhist[:,k])  #Optimal state
end

#Set up visualization
using TrajOptPlots
using MeshCat
using StaticArrays
using RobotZoo:DubinsCar
using Colors

vis = Visualizer()
render(vis)

model = DubinsCar()
TrajOptPlots.set_mesh!(vis, model)

X = [SVector{3}(x) for x in eachcol(xhist)];
visualize!(vis, model, simT, X)



using Plots
pyplot()
Plots.PyPlotBackend()
plot(xhist[1,:], xhist[2,:])