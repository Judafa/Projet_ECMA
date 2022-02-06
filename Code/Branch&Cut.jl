using JuMP
using CPLEX


include("Fonctions_Init.jl")
include("Heuristique.jl")
path = "Instances_ECMA/200_USA-road-d.BAY.gr"

# ------------------------------------------------------------- Lit le modèle
n, s, t, S, d1, d2, p, ph, Mat = read_data(path)

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

# créée les dictionnaires utilisés lors des callback
x_val = Dict()
y_val = Dict()

# Initialise les ensembles U1 et U2 de départ
U1 = Set([d])
U2 = Set([p])

m = Model(CPLEX.Optimizer)
MOI.set(m, MOI.NumberOfThreads(), 1)

# ------------------------------------------------------------- Initialise les variables

# 1 si l'arête ij fait partie du chemin
@variable(m, x[a in Aretes], Bin)
# 1 si le sommet i fait partie du chemin
@variable(m, y[i in Sommets], Bin)
# Variable d'objectif
@variable(m, z >= 0)


# ------------------------------------------------------------- Contraintes
# Le chemin quitte s
@constraint(m, quitte_s, sum(x[a] for a in Aretes if a[1] == s) == 1)

# Le chemin arrive en t
@constraint(m, arrive_t, sum(x[a] for a in Aretes if a[2] == t) == 1)

# Conservation du flot
@constraint(m, flot[v in Sommets ; v != s && v != t], sum(x[a] for a in Aretes if a[1] == v) == sum(x[a] for a in Aretes if a[2] == v))

# Lien entre y et x
@constraint(m, lien_y_v[v in Sommets ; v!=t], y[v] == sum(x[a] for a in Aretes if a[1] == v))

@constraint(m, t_choisi, y[t] == 1)

# Contraintes partielles
@constraint(m, cons_obj[d_init in U1], z >= sum(x[a] * d[a] for a in Aretes))

@constraint(m, cons_sommets[p_init in U2], sum(y[v] * p[v] for v in Sommets) <= S)

# --------------------------------------------------------- Objectif
@objective(m, Min, z)


# Créée le sous problème SP1
SP1 = Model(CPLEX.Optimizer)
@variable(SP1, 0 <= delta1[a in Aretes] <= D[a])
@constraint(SP1, borné_par_d1, sum(delta1[a] for a in Aretes) <= d1)


# Créée le sous problème SP2
SP2 = Model(CPLEX.Optimizer)
@variable(SP2, 0 <= delta2[v in Sommets] <= 2)
@constraint(SP2, borné_par_d2, sum(delta2[v] for v in Sommets) <= d2)

# Créée les dictionnaires pour les heuristiques
delta1_glouton = Dict()
for a in Aretes
    delta1_glouton[a] = 0
end

# ---------------------------------------------------------- Fonction callback
function my_cb_function(cb_data::CPLEX.CallbackContext, context_id::Clong)

    if context_id == CPX_CALLBACKCONTEXT_CANDIDATE || context_id == CPX_CALLBACKCONTEXT_RELAXATION
        CPLEX.load_callback_variable_primal(cb_data, context_id)
        # On récupère les valeur de x, y et z
        for a in Aretes
            x_val[a] = callback_value(cb_data, x[a])
        end

        for v in Sommets
            y_val[v] = callback_value(cb_data, y[v])
        end

        z_val = callback_value(cb_data, z)
        
        # --------------------- Heuristique Gloutonne pour SP1
        traffic_aloue = 0
        aretes_alourdies = []
        trouve_augmentation = true
        while traffic_aloue < d1 && trouve_augmentation
            coeff_max = -Inf64
            a_max = [-1, -1]
            for a in Aretes
                if x_val[a] == 1 && coeff_max < min(d1 - traffic_aloue, D[a]) * d[a] && a ∉ aretes_alourdies
                    coeff_max = min(d1 - traffic_aloue, D[a]) 
                    a_max = a
                end
            end
            if a_max == [-1, -1]
                trouve_augmentation = false
            else
                traffic_aloue += coeff_max
                push!(aretes_alourdies, a_max)
                delta1_glouton[a_max] = coeff_max
            end
        end

        for a in Aretes
            if a ∉ aretes_alourdies
                delta1_glouton[a] = 0
            end
        end
        
        z1_glouton = sum(d[a] * (1 + delta1_glouton[a]) * x_val[a] for a in Aretes)

        if z1_glouton > z_val 
            # Ajoute solution gloutonne
            cons1 = @build_constraint(sum(d[a] * (1 + delta1_glouton[a]) * x[a] for a in Aretes) <= z)
            MOI.submit(m, MOI.LazyConstraint(cb_data), cons1)
        # else
        #     # Résout sous problème SP1
        #     @objective(SP1, Max, sum(d[a] * (1 + delta1[a]) * x_val[a] for a in Aretes))
        #     optimize!(SP1)
        #     z1 = objective_value(SP1)
        #     delta1_val = value.(delta1)

        #     # Ajoute la contrainte de SP1
        #     if z1 > z_val 
        #         cons1 = @build_constraint(sum(d[a] * (1 + delta1_val[a]) * x[a] for a in Aretes) <= z)
        #         MOI.submit(m, MOI.LazyConstraint(cb_data), cons1)
        #     end
        end

        # # Résout sous problème SP1
        # @objective(SP1, Max, sum(d[a] * (1 + delta1[a]) * x_val[a] for a in Aretes))
        # optimize!(SP1)
        # z1 = objective_value(SP1)
        # delta1_val = value.(delta1)

        # # Ajoute la contrainte de SP1
        # if z1 > z_val 
        #     cons1 = @build_constraint(sum(d[a] * (1 + delta1_val[a]) * x[a] for a in Aretes) <= z)
        #     MOI.submit(m, MOI.LazyConstraint(cb_data), cons1)
        # end



        # Résout le sous problème SP2
        @objective(SP2, Max, sum((p[v] + delta2[v] * ph[v]) * y_val[v] for v in Sommets))
        optimize!(SP2)
        z2 = objective_value(SP2)
        delta2_val = value.(delta2)

        if z2 > S
            cons2 = @build_constraint(sum((p[v] + delta2_val[v] * ph[v]) * y[v] for v in Sommets) <= S)
            MOI.submit(m, MOI.LazyConstraint(cb_data), cons2)
        end
    end
end

# On précise que le modèle doit utiliser notre fonction de callback
MOI.set(m, CPLEX.CallbackFunction(), my_cb_function)
optimize!(m)


# print("sommets du chemin : ", value.(y), "\n")
print("valeur de l'objectif Branch&Cut: ", objective_value(m))

