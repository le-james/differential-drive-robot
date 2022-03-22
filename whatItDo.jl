using LinearAlgebra


a = [1 2; 3 4]

traj = [SVector{2}(x) for x in eachcol(a)];

traj







#=  =#


# a = [2 2]
# b = [1 1]
# a * b'
# size(a)

# Q = Diagonal([1 2; 3 4]);

# u = [1 5]
# u*u'


#=  =#


# n=3

# for i in 1:n
#     return i+10
# end


#=  =#


#Series of expressions

# begin
#     x = ones(5)
#     y = zeros(5)
#     # z[1] = x
#     # z[2] = y
# end

# print([x[1] y[1]]*5)

# (
#     x = 1;
#     y = 2;
#     z = x+y
# )


#=  =#


# x0 = [-pi/2; 0; 0; 0]
# xguess = kron(ones(10)', x0)
# ones(10)



# a = zeros(3,3)

# length(a[:,1])



# B = range(1, step=3, stop=60)
# B[:,:]


#=  =#


# A = ones(3,3)
# B = ones(3,3)*2


# a = zeros(3, 6)

# a[:,1:3] = A
# a[:,4:6] = B
# a


#=  =#


# A = Vector(1:5)
# B = Vector(6:10)
# C = [A B]
# print(C)

# A = Vector(1:16)
# B = reshape(A, (4, 4))
# C = reshape(B, (1, length(B)))


#=  =#


# xR = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]
# yR = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]
# θR = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45]

# traj = zeros(3, 20)

# for k=1:20
#     traj[1,k] = xR[k]
#     traj[2,k] = yR[k]
#     traj[3,k] = θR[k]
# end

# traj[:,1]
# size(traj)
# x = [1;2;3]


#= Module stuff =#


# push!( LOAD_PATH, "./testMod.jl" )
# using .hello
# hw()


#=  =#


# refu = [5; 90*pi/180].*ones(2)
# θ = 45
# deg2rad = θ*pi/180


#=  =#


# using LinearAlgebra
# # a = Array(I(1))
# a = [1, 2, 3]

# x = length(a)


#= Arrow function in higher order function =#


# squareall(A) = map(x -> x ^ 2, A)
# a = squareall(1:10)
# print(a)


#= Arrary stuff =#


# xhist = zeros(3, 3)
# x0 = [0; 0; 5]
# x1 = [770; 0; 1]
# # xhist[:, 1] = x0
# # xhist[:, 2] = x1
# # xhist
# xhist[:,1] = x0
# xhist


#= Controllability =#


# using ControlSystems

# A = [1 0; 0 1];
# B = [1; 1];

# ctrb(A, B)