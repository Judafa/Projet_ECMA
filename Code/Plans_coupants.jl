using JuMP, CPLEX

include("Fonctions_Init.jl")

# fichier à utiliser
filename = "20_USA-road-d.NY.gr"
path = string("./Instances_ECMA/", filename)


# lecture et acquisition des données avec la fonction de Fonctions_Init
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





function generate_sub_problems(n, nb_arcs, durees, D, d1, d2, p, ph, x_star, y_star)
    SP1 = Model(CPLEX.Optimizer)    # sous-problème lié à U1
    @variable(SP1, 0 <= delta1[a = 1:nb_arcs] <= D[a])
    @constraint(SP1, sum(delta1[a] for a in 1:nb_arcs) <= d1)
    @objective(SP1, Max, sum(durees[a]*(1 + delta1[a])*x_star[a] for a in 1:nb_arcs))
    JuMP.optimize!(SP1)

    SP2 = Model(CPLEX.Optimizer)    # sous-problème lié à U2
    @variable(SP2, 0 <= delta2[1:n] <= 2)
    @constraint(SP2, sum(delta2[v] for v in 1:n) <= d2)
    @objective(SP2, Max, sum((p[v] + delta2[v]*ph[v])*y_star[v] for v in 1:n))
    JuMP.optimize!(SP2)

    z1 = objective_value(SP1)
    z2 = objective_value(SP2)

    delta1_star = value.(SP1[:delta1])
    delta2_star = value.(SP2[:delta2])
    return z1, z2, delta1_star, delta2_star
end


function plans_coupants(n, nb_arcs, durees, D, d1, d2, p, ph)
    # Définition du modèle pour le problème principal
    MP = Model(CPLEX.Optimizer)
    # variables
    @variable(MP, z >= 0)
    @variable(MP, x[1:nb_arcs], Bin)
    @variable(MP, y[1:n], Bin)
    # contraintes
    @constraint(MP, z >= sum(durees[a]*x[a] for a in 1:nb_arcs))     # U1* composé de d_{ij} uniquement donc durees1 = durees
    @constraint(MP, sum(x[Mat[:,1] .== s]) == 1)     # un arc sortant de s
    @constraint(MP, sum(x[Mat[:,2] .== t]) == 1)     # un arc entrant en t
    for v in 1:n
        if v != s && v !=t
            @constraint(MP, sum(x[Mat[:,2] .== v]) == sum(x[Mat[:,1] .== v]))     # loi des noeuds
        end
    end
    for v in 1:n
        if v !=t
            @constraint(MP, y[v] == sum(x[Mat[:,1] .== v]))     # loi des noeuds
        end
    end
    @constraint(MP, y[t] == sum(x[Mat[:,2] .== t]))
    @constraint(MP, sum(p[v]*y[v] for v in 1:n) <= S)       # U2* composé de p uniquement donc p2 = p
    # objectif
    @objective(MP, Min, z)
    # première résolution sans plans coupants
    JuMP.optimize!(MP)
    z_star = objective_value(MP)
    x_star = value.(MP[:x])
    y_star = value.(MP[:y])

    z1, z2, delta1_star, delta2_star = generate_sub_problems(n, nb_arcs, durees, D, d1, d2, p, ph, x_star, y_star)

    while abs(z1 - z_star) > 1e-4 || z2 > S     # tant que ces deux conditions ne sont pas vérifiées
        if abs(z1 - z_star) > 1e-4
            @constraint(MP, z >= sum(durees[a]*(1 + delta1_star[a])*x[a] for a in 1:nb_arcs))
            println("ajout d'une contrainte SP1")
        elseif z2 > S
            @constraint(MP, sum(y[v]*(p[v] + delta2_star[v]*ph[v]) for v in 1:n) <= S)
            println("ajout d'une contrainte SP2")
        end
    
        JuMP.optimize!(MP)      # on relance la recherche de solution de MP
        
        z_star = objective_value(MP)
        x_star = value.(MP[:x])
        y_star = value.(MP[:y])
    
        z1, z2, delta1_star, delta2_star = generate_sub_problems(n, nb_arcs, durees, D, d1, d2, p, ph, x_star, y_star)
    end
    return z_star
end



z_star = plans_coupants(n, nb_arcs, durees, D, d1, d2, p, ph)

println("Objective value: ", z_star)
















