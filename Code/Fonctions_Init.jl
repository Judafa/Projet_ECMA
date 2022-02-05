


function read_data_include(path)

    include(path)

    return n, s, t, S, d1, d2, p, ph, Mat
end


function read_data(path)

    if isfile(path)
        # L’ouvrir
        myFile = open(path)
        # Lire toutes les lignes d’un fichier
        data = readlines(myFile)

        # Pour chaque ligne du fichier
        
        for i in 1:8
            # Afficher la ligne
            eval(Meta.parse(data[i]))
        end

        # Pour la matrice Mat
        string_mat = ""
        for i in 9:size(data)[1]
            string_mat = string_mat * data[i]
            end
        eval(Meta.parse(string_mat))
        # Fermer le fichier
        close(myFile)
    end

    return n, s, t, S, d1, d2, p, ph, Mat
end