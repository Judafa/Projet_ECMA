using JuMP, CPLEX, BenchmarkTools, DelimitedFiles

include("Fonctions_Init.jl")

# fichier à utiliser


# filename = "60_USA-road-d.BAY.gr"
# path = string("./Code/Instances_ECMA/", filename)


# # lecture et acquisition des données avec la fonction de Fonctions_Init
# n, s, t, S, d1, d2, p, ph, Mat = read_data(path)

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


"""
Résout le problème par dualisation.

Prend en argument les données des fichiers <*.gr>.

verbose = false désactive l'affichage.

time_lim définit une limite pour l'optimisation, time_lim <= 0
désactive cette limite.
"""
function dualisation(n, s, t, S, d1, d2, p, ph, Mat; verbose=false, time_lim = 60.0)
    nb_arcs = size(Mat)[1]
    durees = Mat[:,3]
    D = Mat[:,4]
    
    # Définition du modèle
    model = Model(CPLEX.Optimizer)

    if verbose == false
        set_silent(model)
    end

    if time_lim > 0
        set_time_limit_sec(model, time_lim)
    end
    
    # variables
    @variable(model, x[1:nb_arcs], Bin)
    @variable(model, y[1:n], Bin)
    @variable(model, t1 >= 0)
    @variable(model, t2 >= 0)
    @variable(model, z[1:nb_arcs] >= 0)
    @variable(model, z_p[1:n] >= 0)
    
    # contraintes
    @constraint(model, sum(x[Mat[:,1] .== s]) - sum(x[Mat[:,2] .== s]) == 1)     # un arc sortant de s
    @constraint(model, sum(x[Mat[:,2] .== t]) -  sum(x[Mat[:,1] .== t]) == 1)     # un arc entrant en t
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
    
    JuMP.optimize!(model)
    
    # affichage des resultats
    obj_value = JuMP.objective_value(model)
    x_val = value.(model[:x])
    y_val = value.(model[:y])
    # println("Objective value: ", obj_value)
    return obj_value, x_val, y_val
end


# @benchmark dualisation(n, s, t, S, d1, d2, p, ph, Mat, time_lim = 0)
# @btime dualisation(n, s, t, S, d1, d2, p, ph, Mat, time_lim = 0)


# obj_value, x_val, y_val = dualisation(n, s, t, S, d1, d2, p, ph, Mat, time_lim = 0)
# println("Objective value: ", obj_value)


# sol_filename = string("SOL_", filename, ".txt")
# sol_path = string("Solutions/", sol_filename)
# open(sol_path, "w") do file
#     write(file, "valeur optimale : $obj_value\n")
#     write(file, "######## x :\n")
#     for a in x_val
#         write(file, "$a\n")
#     end
#     write(file, "######## y :\n")
#     for v in y_val
#         write(file, "$v\n")
#     end
# end