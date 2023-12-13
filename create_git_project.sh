#inserisci il nome del progetto git
echo "Inserisci il nome del progetto git:"
read git_name
#crea la cartella con il nome del progetto git
mkdir "$git_name"
cd "$git_name"
#inserisci il link del progetto git
echo "Inserisci il link del progetto git:"
read git_link
git clone "$git_link"
cd ..


# questo parte da src 
# Stampa la struttura di folder del progetto git
echo "La struttura di folder del progetto git è:"
tree -d "$git_name"

# Chiedi il percorso del progetto git
echo "Inserisci il percorso del progetto git:"
read git_folder



# Verifica se il percorso inserito è valido
#la condizione è se il path inserito dall'utente git_folder esiste a partire da gitname 
#se non esiste chiedi di inserire un percorso valido
#se esiste crea il link simbolico


while [ ! -d "$git_name/$git_folder" ]; do
    echo "Il percorso $git_folder non è valido. Inserisci un percorso valido:"
    read git_folder
done


echo "Hai inserito il percorso: $git_folder"

# Crea il link simbolico al progetto git nella cartella src 
ln -s "$git_name/$git_folder" "$git_name"

echo "Link simbolico creato con successo."






