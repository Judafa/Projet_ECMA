using JuMP
using CPLEX

# --------------------- Heuristique Gloutonne pour SP1
function heuristique_SP1(d, D, d1, Aretes, x_val, delta1_glouton)

        traffic_aloue = 0
        aretes_alourdies = []
        trouve_augmentation = true

        ajout_traffic = 0

        while traffic_aloue < d1 && trouve_augmentation
            coeff_max = -Inf64
            a_max = [-1, -1]
            for a in Aretes
                # if x_val[a] == 1 && coeff_max < min(d1 - traffic_aloue, D[a]) * d[a] && a ∉ aretes_alourdies
                #     coeff_max = min(d1 - traffic_aloue, D[a]) 
                #     a_max = a
                # end
                if x_val[a] == 1 && coeff_max < D[a] * d[a] && a ∉ aretes_alourdies
                    coeff_max = D[a] * d[a]
                    ajout_traffic = min(d1 - traffic_aloue, D[a])
                    a_max = a
                end
            end
            if a_max == [-1, -1]
                trouve_augmentation = false
            else
                traffic_aloue += ajout_traffic
                push!(aretes_alourdies, a_max)
                delta1_glouton[a_max] = ajout_traffic
            end
        end

        for a in Aretes
            if a ∉ aretes_alourdies
                delta1_glouton[a] = 0
            end
        end
        
        z1_glouton = sum(d[a] * (1 + delta1_glouton[a]) * x_val[a] for a in Aretes)

    return(z1_glouton, delta1_glouton)
end


function heuristique_SP2(p, ph, d2, Sommets, y_val, delta2_glouton)
    poids_aloue = 0
    sommets_alourdis = []
    poids_min = Inf64
    sommet_min = -1
    trouve_augmentation = true

    ajout_poids = 0

    for v in Sommets
        if y_val[v] == 1 
            if poids_aloue < d2

                poids_aloue += ph[v]
                if poids_min > p[v]
                    poids_min = p[v]
                    sommet_min = v
                end
                push!(sommets_alourdis, v)

            # Si le poids max est dépassé, ajoute le sommet courant si on gagne à le faire
            elseif poids_min < p[v]
                sommets_alourdis = replace(sommets_alourdis, v => sommet_min)
                poids_min = Inf64

                # Recalcule le sommet de poids min
                for v_temp in sommets_alourdis
                    if poids_min > p[v_temp]
                        poids_min = p[v_temp]
                        sommet_min = v_temp
                    end
                end
            end
        end
    end

    # Ajoute les poids aux sommets alourdis
    for v in Sommets
        if v in sommets_alourdis && v != sommet_min
            delta2_glouton[v] = ph[v]
        elseif v == sommet_min
            delta2_glouton[v] = d2 - sum(ph[s] for s in sommets_alourdis) + ph[s]
        else
            delta2_glouton[v] = 0
        end
    end

    z2_glouton = sum((p[v] + delta2_glouton[v] * ph[v]) * y_val[v] for v in Sommets)

    return z2_glouton, delta2_glouton
end


function heuristique_lourde_SP2(p, ph, d2, Sommets, y_val)
    while poids_aloue < d2 && trouve_augmentation
            coeff_max = -Inf64
            v_max = -1
            for v in Sommets
                if y_val[v] == 1 && coeff_max < p[v] && v ∉ sommets_alourdis
                    coeff_max = ph[v]
                    ajout_poids = min(d2 - poids_aloue, ph[v])
                    v_max = v
                end
            end
            if v_max == -1
                trouve_augmentation = false
            else
                poids_aloue += ajout_poids
                push!(sommets_alourdis, v_max)
                delta2_glouton[v_max] = ajout_poids
            end
        end

        for v in Sommets
            if v ∉ sommets_alourdis
                delta2_glouton[v] = 0
            end
        end
    
    z2_glouton = sum((p[v] + delta2_glouton[v] * ph[v]) * y_val[v] for v in Sommets)
    return z2_glouton, delta2_glouton
end