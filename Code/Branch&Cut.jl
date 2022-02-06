using JuMP
using CPLEX
using BenchmarkTools


include("Fonctions_Init.jl")
include("Heuristique.jl")

# ------------------------------------------------------------- Lit le modèle

function branch_and_cut(n, s, t, S, d1, d2, p, ph, Mat; verbose=false, verbose_sous_probleme=false, utilise_heuristique_SP1 = false, utilise_heuristique_SP2=true)

    m = Model(CPLEX.Optimizer)
    MOI.set(m, MOI.NumberOfThreads(), 1)

    if verbose == false
        set_silent(m)
    end

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

    # ---------------------------------------------------------- Initialise SP1 en fonction des options choisies
    if utilise_heuristique_SP1
        # Créée les dictionnaires pour les heuristiques
        delta1_glouton = Dict()
        for a in Aretes
            delta1_glouton[a] = 0
        end
    # sinon créée le sous problème SP1
    else
        SP1 = Model(CPLEX.Optimizer)
        if verbose_sous_probleme == false
            set_silent(SP1)
        end
        @variable(SP1, 0 <= delta1[a in Aretes] <= D[a])
        @constraint(SP1, borné_par_d1, sum(delta1[a] for a in Aretes) <= d1)
        
    end

    # ---------------------------------------------------------- Initialise SP2 en fonction des options choisies
    # Créée le sous problème SP2
    if utilise_heuristique_SP2
        delta2_glouton = Dict()
        for v in Sommets
            delta2_glouton[v] = 0
        end
    else
        SP2 = Model(CPLEX.Optimizer)
        if verbose_sous_probleme == false
            set_silent(SP2)
        end
        @variable(SP2, 0 <= delta2[v in Sommets] <= 2)
        @constraint(SP2, borné_par_d2, sum(delta2[v] for v in Sommets) <= d2)
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
            
            # -------------------------------------------------- Résout le sous problème SP1
            if utilise_heuristique_SP1
                # Heuristique Gloutonne pour SP1
                z1_glouton, delta1_glouton = heuristique_SP1(d, D, d1, Aretes, x_val, delta1_glouton)
                # Ajoute la contrainte de SP1
                if z1_glouton > z_val 
                    cons1 = @build_constraint(sum(d[a] * (1 + delta1_glouton[a]) * x[a] for a in Aretes) <= z)
                    MOI.submit(m, MOI.LazyConstraint(cb_data), cons1)
                end
            else
                # Sinon résout le sous problème SP1 avec cplex
                @objective(SP1, Max, sum(d[a] * (1 + delta1[a]) * x_val[a] for a in Aretes))
                optimize!(SP1)
                z1 = objective_value(SP1)
                delta1_val = value.(delta1)

                # Ajoute la contrainte de SP1
                if z1 > z_val 
                    cons1 = @build_constraint(sum(d[a] * (1 + delta1_val[a]) * x[a] for a in Aretes) <= z)
                    MOI.submit(m, MOI.LazyConstraint(cb_data), cons1)
                end

            end

            # -------------------------------------------------- Résout le sous problème SP2
            if utilise_heuristique_SP2
                # Heuristique Gloutonne pour SP2
                z2_glouton, delta2_glouton = heuristique_SP2(p, ph, d2, Sommets, y_val, delta2_glouton)

                # Ajoute la coupe trouvée au problème maître
                if z2_glouton > S
                    # Ajoute solution gloutonne
                    cons2 = @build_constraint(sum((p[v] + delta2_glouton[v] * ph[v]) * y[v] for v in Sommets) <= S)
                    MOI.submit(m, MOI.LazyConstraint(cb_data), cons2)
                end
            else
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
    end

    # On précise que le modèle doit utiliser notre fonction de callback
    MOI.set(m, CPLEX.CallbackFunction(), my_cb_function)
    optimize!(m)

    obj_value = JuMP.objective_value(m)
    return obj_value
end

path = "Code/Instances_ECMA/1500_USA-road-d.BAY.gr"
n, s, t, S, d1, d2, p, ph, Mat = read_data(path)

print("pour : ", path)
@benchmark branch_and_cut(n, s, t, S, d1, d2, p, ph, Mat, verbose=false)

# obj_value = branch_and_cut(n, s, t, S, d1, d2, p, ph, Mat, verbose=false)
# print("valeur de l'objectif Branch&Cut : $obj_value")

