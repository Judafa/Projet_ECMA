

include("Plans_coupants.jl")
include("Branch&Cut.jl")
include("Dualisation.jl")


instances = [20, 40, 60, 80, 100,
            120, 140, 160, 180, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000,
            1100, 1200, 1300, 1400, 1500, 1600, 1700]

# instances = [20, 40]

path_dualisation = "Rapport_Final/Résultats/résultats_dualisation.txt"
benchmark_dualisation(instances, path_dualisation)

path_plans_coupants = "Rapport_Final/Résultats/résultats_plans_coupants.txt"
benchmark_plans_coupants(instances, path_plans_coupants)

path_b_and_cut = "Rapport_Final/Résultats/résultats_b_and_cut.txt"
benchmark_b_and_cut(instances, path_b_and_cut)