using JuMP
using CPLEX

include("Fonctions_Init.jl")

# ------------------------------------------------------------- Lit le modèle
# n, s, t, S, d1, d2, p, ph, Mat = read_data_include("Instances_ECMA/20_USA-road-d.NY.gr")
n, s, t, S, d1, d2, p, ph, Mat = read_data("Instances_ECMA/100_USA-road-d.BAY.gr")


# a est le Nombre d'arêtes
a = Int64(size(Mat)[1])


m = Model(CPLEX.Optimizer)

# ------------------------------------------------------------- Initialise les variables
# 1 si l'arête ij fait partie du chemin
@variable(m, x[i in 1:n, j in 1:n], Bin)

# 1 si le sommet i fait partie du chemin
@variable(m, y[i in 1:n], Bin)

# ------------------------------------------------------------- Contraintes
# Le chemin quitte s
@constraint(m, quitte_s, sum(x[s, Int.(Mat[j, 2])] for j=1:a if Int.(Mat[j,1]) == s) == 1)

# Le chemin arrive en t
@constraint(m, arrive_t, sum(x[Int.(Mat[j, 1]), t] for j=1:a if Int.(Mat[j,2]) == t) == 1)

# Conservation du flot
@constraint(m, flot[v=1:n ; v != s && v != t], sum(x[v, Int.(Mat[j, 2])] for j=1:a if Int.(Mat[j,1]) == v) == sum(x[Int.(Mat[j, 1]), v] for j=1:a if Int.(Mat[j,2]) == v))

# Lien entre y et x
@constraint(m, lien_y_v[v=1:n ; v!=t], y[v] == sum(x[v, Int.(Mat[j, 2])] for j=1:a if Int.(Mat[j,1]) == v))

# @constraint(m, y[t] == sum(x[Int.(Mat[j, 1]), t] for j=1:a if Mat[j,2] == t))
@constraint(m, t_choisi, y[t] == 1)

# Contrainte sur le poids maximal
@constraint(m, cons_poids_max, sum(p[v] * y[v] for v=1:n) <= S)

# --------------------------------------------------------- Objectif
@objective(m, Min, sum(x[Int.(Mat[j, 1]), Int.(Mat[j, 2])] * Mat[j, 3] for j in 1:a))


optimize!(m)
# print(m)
print("sommets du chemin : ", value.(y), "\n")
print("valeur de l'objectif : ", objective_value(m))


# n = 6
# K = 23
# w = [1 3 5 7 9 11]
# p = [1 2 4 5 7 10]

# m = Model(CPLEX.Optimizer)

# @variable(m, x[i in 1:n], Bin)
# @constraint(m, sum(x[i] * w[i] for i = 1:n) <= K)
# @objective(m, Max, sum(x[i] * p[i] for i in 1:n))

# optimize!(m)

# print("Solution : ", value.(x))