

include("Plans_coupants.jl")
include("Branch&Cut.jl")
include("Dualisation.jl")

function read_data_benchmark(path)
    println("Attempting to find file.")
    if isfile(path)
        println("File found, loading.")
        # L’ouvrir
        myFile = open(path)
        # Lire toutes les lignes d’un fichier
        data = readlines(myFile)

        s = ""
        # Pour chaque ligne du fichier
        for (i, line) in enumerate(data)
            if i <= 8
                # Afficher la ligne
                eval(Meta.parse(line))
            else
                s = s * line
            end

        end

        eval(Meta.parse(s))
        

        # Mat = readdlm(myFile, ' ', Float64, ';', skipstart=10)

        # Fermer le fichier
        close(myFile)
    else
        print("fichier pas trouvé")
    end


    return n, s, t, S, d1, d2, p, ph, Mat
end

function benchmark_plans_coupants(instances, path_fichier)
    touch(path_fichier)
    for nb in instances

        path_instance = "Code/Instances_ECMA/$(nb)_USA-road-d.BAY.gr"
        n, s, t, S, d1, d2, p, ph, Mat = read_data_benchmark(path_instance)
        temps = @belapsed plans_coupants(n, s, t, S, d1, d2, p, ph, Mat)
        print("Pour nb = $(nb) avec plans coupants, temps = $(temps)\n")

        fichier = open(path_fichier,"a")
        write(fichier, "$(nb) $(temps)\n")
        close(fichier)
    end
end

function benchmark_b_and_cut(instances, path_fichier)
    touch(path_fichier)
    for nb in instances

        path_instance = "Code/Instances_ECMA/$(nb)_USA-road-d.BAY.gr"
        n, s, t, S, d1, d2, p, ph, Mat = read_data_benchmark(path_instance)

        temps = @belapsed branch_and_cut(n, s, t, S, d1, d2, p, ph, Mat)
        print("Pour nb = $(nb) avec branch & cut, temps = $(temps)\n")

        fichier = open(path_fichier,"a")
        write(fichier, "$(nb) $(temps)\n")
        close(fichier)
    end
end

function benchmark_dualisation(instances, path_fichier)
    touch(path_fichier)
    for nb in instances

        path_instance = "Code/Instances_ECMA/$(nb)_USA-road-d.BAY.gr"
        n, s, t, S, d1, d2, p, ph, Mat = read_data_benchmark(path_instance)


        temps = @belapsed dualisation(n, s, t, S, d1, d2, p, ph, Mat)
        print("Pour nb = $(nb) avec dualisation, temps = $(temps)\n")

        fichier = open(path_fichier,"a")
        write(fichier, "$(nb) $(temps)\n")
        close(fichier)
    end
end


# instances = [20, 40, 60, 80, 100,
#             120, 140, 160, 180, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000,
#             1100, 1200, 1300, 1400, 1500, 1600, 1700]

instances = [20, 40, 60]

path_dualisation = "Rapport_Final/Résultats/résultats_dualisation.txt"
benchmark_dualisation(instances, path_dualisation)

path_plans_coupants = "Rapport_Final/Résultats/résultats_plans_coupants.txt"
benchmark_plans_coupants(instances, path_plans_coupants)

path_b_and_cut = "Rapport_Final/Résultats/résultats_b_and_cut.txt"
benchmark_b_and_cut(instances, path_b_and_cut)