using JuMP
using CPLEX

include("Fonctions_Init.jl")

# ------------------------------------------------------------- Lit le modèle

# n, s, t, S, d1, d2, p, ph, Mat = read_data_include("Instances_ECMA/20_USA-road-d.NY.gr")
# n, s, t, S, d1, d2, p, ph, Mat = read_data("Instances_ECMA/20_USA-road-d.BAY.gr")
# n, s, t, S, d1, d2, p, ph, Mat = read_data("Instances_ECMA/400_USA-road-d.BAY.gr")

filename = "40_USA-road-d.BAY.gr"
path = string("./Code/Instances_ECMA/", filename)
# lecture et acquisition des données avec la fonction de Fonctions_Init
n, s, t, S, d1, d2, p, ph, Mat = read_data(path)


function statique(n, s, t, S, p, Mat)
    # nb_aretes est le Nombre d'arêtes
    nb_aretes = Int64(size(Mat)[1])
    
    epsilon = 1e-5
    
    Aretes = Set()
    for e in 1:nb_aretes
        push!(Aretes, [Int.(Mat[e, 1]), Int.(Mat[e, 2])])
    end
    
    Sommets = Set(1:n)
    
    # Inititialise les dij
    d = Dict()
    for e in 1:nb_aretes
        d[[Int.(Mat[e, 1]), Int.(Mat[e, 2])]] = Mat[e, 3]
    end
    
    # Inititialise les Dij
    D = Dict()
    for e in 1:nb_aretes
        D[[Int.(Mat[e, 1]), Int.(Mat[e, 2])]] = Mat[e, 4]
    end
    
    
    m = Model(CPLEX.Optimizer)
    
    # ------------------------------------------------------------- Initialise les variables
    
    # 1 si l'arête ij fait partie du chemin
    @variable(m, x[a in Aretes], Bin)
    # 1 si le sommet i fait partie du chemin
    @variable(m, y[i in Sommets], Bin)
    
    # ------------------------------------------------------------- Contraintes
    # Le chemin quitte s
    @constraint(m, quitte_s, sum(x[a] for a in Aretes if a[1] == s) - sum(x[a] for a in Aretes if a[2] == s) == 1)

    # Le chemin arrive en t
    @constraint(m, arrive_t, sum(x[a] for a in Aretes if a[2] == t) - sum(x[a] for a in Aretes if a[1] == t) == 1)
    
    # Conservation du flot
    @constraint(m, flot[v in Sommets ; v != s && v != t], sum(x[a] for a in Aretes if a[1] == v) == sum(x[a] for a in Aretes if a[2] == v))
    
    # Lien entre y et x
    @constraint(m, lien_y_v[v in Sommets ; v!=t], y[v] == sum(x[a] for a in Aretes if a[1] == v))
    
    @constraint(m, t_choisi, y[t] == 1)
    
    # Contrainte sur le poids maximal
    @constraint(m, cons_poids_max, sum(p[v] * y[v] for v in Sommets) <= S)
    
    # --------------------------------------------------------- Objectif
    @objective(m, Min, sum(x[a] * d[a] for a in Aretes))
    
    
    optimize!(m)

    obj_value = objective_value(m)
    println("Objective value: ", obj_value)
    return obj_value
end

# @benchmark statique(n, s, t, S, p, Mat)
# obj_value = statique(n, s, t, S, p, Mat)
# println("Objective value: ", obj_value)

statique(n, s, t, S, p, Mat)

# print(m)
# print("sommets du chemin : ", value.(y), "\n")
# print("valeur de l'objectif : ", objective_value(m))
