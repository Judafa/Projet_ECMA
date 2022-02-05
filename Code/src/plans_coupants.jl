using JuMP, CPLEX

# fichier à utiliser
path = "20_USA-road-d.BAY.gr"

function read_data(path)
    include(path)
    return n, s, t, S, d1, d2, p, ph, Mat
end

# lecture et acquisition des données
n, s, t, S, d1, d2, p, ph, Mat = read_data("20_USA-road-d.BAY.gr")

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
@variable(model, z >= 0)
@variable(model, x[1:nb_arcs], Bin)
@variable(model, y[1:n], Bin)



# contraintes
@constraint(model, z >= sum(durees[a]*x[a] for a in 1:nb_arcs))     # U1* composé de d_{ij} uniquement donc durees1 = durees
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
@constraint(model, sum(p[v]*y[v] for v in 1:n) <= S)       # U2* composé de p uniquement donc p2 = p


# objectif
@objective(model, Min, z)

# première résolution sans plans coupants
JuMP.optimize!(model)

z_star = objective_value(model)


function my_callback_function(cb_data, z_star)
    x_val = callback_value(cb_data, x)
    y_val = callback_value(cb_data, y)
    println("Called from (x, y) = ($x_val, $y_val)")
    status = callback_node_status(cb_data, model)
    if status == MOI.CALLBACK_NODE_STATUS_FRACTIONAL
        println(" - Solution is integer infeasible!")
    elseif status == MOI.CALLBACK_NODE_STATUS_INTEGER
        println(" - Solution is integer feasible!")
    else
        @assert status == MOI.CALLBACK_NODE_STATUS_UNKNOWN
        println(" - I don't know if the solution is integer feasible :(")
    end

    SP1 = Model(CPLEX.Optimizer)    # sous-problème lié à U1
    @variable(SP1, 0 <= delta1[a = 1:nb_arcs] <= D[a])
    @constraint(SP1, sum(delta1[a] for a in 1:nb_arcs) <= d1)
    @objective(SP1, Max, sum(durees[a]*(1 + delta1[a])*x_val[a] for a in 1:nb_arcs))
    JuMP.optimize!(SP1)

    SP2 = Model(CPLEX.Optimizer)    # sous-problème lié à U2
    @variable(SP2, 0 <= delta2[1:n] <= 2)
    @constraint(SP2, sum(delta2[v] for v in 1:n) <= d2)
    @objective(SP2, Max, sum((p[v] + delta2[v]*ph[v])*y_val[v] for v in 1:n))
    JuMP.optimize!(SP2)

    z1 = objective_value(SP1)
    z2 = objective_value(SP2)

    delta1_star = JuMP.value(delta1)
    delta2_star = JuMP.value(delta2)

    if abs(z1 - z_star) > 1e-4
        con = @build_constraint(z >= sum(durees[a]*(1 + delta1_star[a])*x[a] for a in 1:nb_arcs))
        println("Adding $(con)")
        MOI.submit(model, MOI.LazyConstraint(cb_data), con)
    elseif z2 > S
        con = @build_constraint(sum(y[v]*(p[v] + delta2_star[v]*ph[v])) <= S)
        println("Adding $(con)")
        MOI.submit(model, MOI.LazyConstraint(cb_data), con)
    end
end
MOI.set(model, MOI.LazyConstraintCallback(), my_callback_function)
optimize!(model)
obj_value = JuMP.objective_value(model)
println("Objective value: ", obj_value)
# println("Optimal solution (x, y) = ($(value(x)), $(value(y)))")















