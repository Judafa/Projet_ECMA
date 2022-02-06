using JuMP
using CPLEX

include("Fonctions_Init.jl")

function coupe_max(n, s, t, S, d1, d2, p, ph, Aretes, Sommets, d, D)

    m_h = Model(CPLEX.Optimizer)

    # ------------------------------------------------------------- Initialise les variables

    # 1 si l'arête ij fait partie de la coupe max
    @variable(m_h, x[a in Aretes], Bin)
    # 1 si le sommet i fait partie du même ensemble que s, 0 sinon
    @variable(m_h, y[i in Sommets], Bin)

    # ------------------------------------------------------------- Contraintes
    # y_s est de valeur 1
    @constraint(m_h, valeur_s, y[s] == 1)

    # y_t est de valeur 0
    @constraint(m_h, valeur_t, y[t] == 0)

    @constraint(m_h, coupe1[a in Aretes], x[a] <= y[a[1]] + y[a[2]])
    @constraint(m_h, coupe2[a in Aretes], x[a] <= 2 - y[a[1]] - y[a[2]])

    # @constraint(m_h, somme_limitée, sum(x[a] * d[a] for a in Aretes) <= d1)

    # --------------------------------------------------------- Objectif
    @objective(m_h, Min, sum(x[a] for a in Aretes))


    optimize!(m_h)

    return value.(x)
end




function chemin_alourdi(n, s, t, S, d1, d2, p, ph, Aretes, Sommets, d, D)

    m_h = Model(CPLEX.Optimizer)

    # ------------------------------------------------------------- Initialise les variables

    # 1 si l'arête ij fait partie du chemin
    @variable(m_h, x[a in Aretes], Bin)
    # 1 si le sommet i fait partie du chemin
    @variable(m_h, y[i in Sommets], Bin)

    # variables d'alourdissement des chemins
    variable(m_h, 0 <= delta1[a in Aretes] <= D[a])
    # variables d'alourdissement des sommets
    variable(m_h, 0 <= delta2[v in Sommets] <= 2)


    # ------------------------------------------------------------- Contraintes
    # Le chemin quitte s
    @constraint(m_h, quitte_s, sum(x[a] for a in Aretes if a[1] == s) == 1)

    # Le chemin arrive en t
    @constraint(m_h, arrive_t, sum(x[a] for a in Aretes if a[2] == t) == 1)

    # Conservation du flot
    @constraint(m_h, flot[v in Sommets ; v != s && v != t], sum(x[a] for a in Aretes if a[1] == v) == sum(x[a] for a in Aretes if a[2] == v))

    # Lien entre y et x
    @constraint(m_h, lien_y_v[v in Sommets ; v!=t], y[v] == sum(x[a] for a in Aretes if a[1] == v))

    @constraint(m_h, t_choisi, y[t] == 1)

    # Contrainte sur le poids maximal
    @constraint(m_h, cons_poids_max, sum(p[v] * y[v] for v in Sommets) <= S)

    # Tous les alourdissements sont égaux
    a1 = [Mat[1, 1], Mat[1, 2]]
    @constraint(m_h, deltas1_égaux[a in Aretes], delta1[a] == delta[a1])

    # --------------------------------------------------------- Objectif
    @objective(m_h, Min, sum(x[a] * d[a] for a in Aretes))


    optimize!(m_h)
    # print(m)
    print("sommets du chemin : ", value.(y), "\n")
    print("valeur de l'objectif : ", objective_value(m_h))


end