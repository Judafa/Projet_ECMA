using DelimitedFiles

function read_data_include(path)

    include(path)

    return n, s, t, S, d1, d2, p, ph, Mat
end


function read_data(path)
    println("Attempting to find file.")
    if isfile(path)
        println("File found, loading.")
        # L’ouvrir
        myFile = open(path)
        # Lire toutes les lignes d’un fichier
        data = readlines(myFile)

        # Pour chaque ligne du fichier
        for i in 1:8
            # Afficher la ligne
            eval(Meta.parse(data[i]))
        end

        # # Pour la matrice Mat
        # Mat = zeros(size(data)[1] - 9, 4)
        # mat_lines = data[10:size(data)[1]]

        # compteur = 1
        # for line in mat_lines
        #     # Enlève les ; et ]
        #     line = replace(line,";" => "" )
        #     line = replace(line,"]" => "" )
        #     # sépare la ligne par coefficients
        #     coeffs = split(line, " ")
        #     Mat[compteur, 1] = tryparse(Int64, coeffs[1])
        #     Mat[compteur, 2] = tryparse(Int64, coeffs[2])
        #     Mat[compteur, 3] = tryparse(Int64, coeffs[3])
        #     Mat[compteur, 4] = tryparse(Float64, coeffs[4])

        #     compteur += 1
        #     end

        Mat = readdlm(myFile, ' ', Float64, ';', skipstart=10)

        # Fermer le fichier
        close(myFile)
    else
        print("fichier pas trouvé")
    end


    return n, s, t, S, d1, d2, p, ph, Mat
end

