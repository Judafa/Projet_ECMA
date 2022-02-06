using JuMP, CPLEX

include("Fonctions_Init.jl")

# fichier à utiliser
include("Fonctions_Init.jl")

path = "Instances_ECMA/100_USA-road-d.BAY.gr"

# lecture et acquisition des données
n, s, t, S, d1, d2, p, ph, Mat = read_data(path)

######################### DONNEES #########################
# n : nombre de sommets
# s : indice du sommet source
# t : indice du sommet puits
# S : poids maximal admissible
# d1 : limite de l'augmentation totale de la durée des arcs
# d2 : limite de l'augmentation totale des poids du graphe
# p : poids des sommets
# ph : p chapeau, paramètres d'augmentation des poids
# Mat : matrice représentant le graphe 
#   Mat[k] = [i, j, dij, Dij]
###########################################################

# println("n = $n")
# println("s = $s")
# println("t = $t")
# println("S = $S")
# println("d1 = $d1")
# println("d2 = $d2")
# println("p = $p")
# println("ph = $ph")
# println("Mat = $Mat")

nb_arcs = size(Mat)[1]
durees = Mat[:,3]
D = Mat[:,4]


# Définition du modèle
model = Model(CPLEX.Optimizer)

# variables
@variable(model, x[1:nb_arcs], Bin)
@variable(model, y[1:n], Bin)
@variable(model, t1 >= 0)
@variable(model, t2 >= 0)
@variable(model, z[1:nb_arcs] >= 0)
@variable(model, z_p[1:n] >= 0)

# contraintes
@constraint(model, sum(x[Mat[:,1] .== s]) == 1)     # un arc sortant de s
@constraint(model, sum(x[Mat[:,2] .== t]) == 1)     # un arc entrant en t
for v in 1:n
    if v != s && v !=t
        @constraint(model, sum(x[Mat[:,2] .== v]) == sum(x[Mat[:,1] .== v]))     # loi des noeuds
    end
end
for v in 1:n
    if v !=t
        @constraint(model, y[v] == sum(x[Mat[:,1] .== v]))     # loi des noeuds
    end
end
@constraint(model, y[t] == sum(x[Mat[:,2] .== t]))
@constraint(model, sum(p[v]*y[v] + 2*z_p[v] for v in 1:n) + t2*d2 <= S)    # contrainte robuste dualisée
for a in 1:nb_arcs
    @constraint(model, t1 + z[a] >= durees[a]*x[a])
end
for v in 1:n
    @constraint(model, t2 + z_p[v] >= ph[v]*y[v])
end


# objectif
@objective(model, Min, sum(durees[a]*x[a] for a in 1:nb_arcs) + d1*t1 + sum(D[a]*z[a] for a in 1:nb_arcs))

print(model)

JuMP.optimize!(model)

# affichage des resultats
obj_value = JuMP.objective_value(model)
println("Objective value: ", obj_value)

# println(JuMP.value(x))
# println(JuMP.value(y))
