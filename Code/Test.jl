using JuMP
using CPLEX

n = 6

K = 23

w = [1 3 5 7 9 11]

p = [1 2 4 5 7 10]


m = Model(CPLEX.Optimizer)


@variable(m, x[i in 1:n], Bin)

@constraint(m, sum(x[i] * w[i] for i = 1:n) <= K)

@objective(m, Max, sum(x[i] * p[i] for i in 1:n))


optimize!(m)


print("Solution : ", value.(x))