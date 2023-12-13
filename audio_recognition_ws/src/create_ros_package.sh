# Chiedi il nome del pacchetto ROS
echo "Inserisci il nome del pacchetto ROS:"
read package_name

# Dichiarazione delle dipendenze predefinite
default_dependencies=("std_msgs" "message_generation" "rospy" "rosconsole" "roscpp")

# Stampa le dipendenze predefinite
echo "Le dipendenze predefinite sono: ${default_dependencies[@]}"

# Chiedi all'utente se vuole aggiungere altre dipendenze
echo "Vuoi aggiungere altre dipendenze? (y/n):"
read add_more_dependencies

# Se l'utente vuole aggiungere altre dipendenze, chiedi e aggiungile al file "dependencies"
if [[ "$add_more_dependencies" == "y" || "$add_more_dependencies" == "Y" ]]; then
    echo "Inserisci le dipendenze aggiuntive (separate da spazi):"
    read -a additional_dependencies
    all_dependencies=("${default_dependencies[@]}" "${additional_dependencies[@]}")
else
    all_dependencies=("${default_dependencies[@]}")
fi


# Crea il pacchetto ROS con le dipendenze specificate
catkin create pkg "$package_name" --catkin-deps "${all_dependencies[@]}"

mkdir "$package_name"/scripts

#crea lo script per creare i nodi del mio pacchetto
# crea uno script bash per il nodo 
touch "$package_name"/scripts/create_node.sh
# lo script bash deve essere eseguibile
chmod u+x "$package_name"/scripts/create_node.sh

echo "
#!/bin/bash
echo 'Inserisci il nome del nodo senza .py:'
read node_name

# Crea il file .py e imposta i permessi di esecuzione
touch "\$node_name.py"
chmod u+x "\$node_name.py"

# Chiedi all'utente quale template desidera utilizzare
echo 'Quale tipo di template desideri utilizzare?'
echo '1. Template per il Listener'
echo '2. Template per il Publisher'
echo '3. Nessun template'
read choice

# Aggiungi il shebang (#!/usr/bin/env python3) al file .py
echo '#!/usr/bin/env python3' >> "\$node_name.py"
echo 'import rospy' >> "\$node_name.py"
echo '#from std_msgs.msg import YourType' >> "\$node_name.py"
echo "#le dependencies per questo nodo sono le seguenti: ${default_dependencies[@]} aggiungi gli import che ti servono nel nodo" >> "\$node_name.py"

# Aggiungi il template in base alla scelta dell'utente
case \$choice in
    1) # Template per il Listener
        echo 'Aggiungo il template per il Listener al file...'
        cat <<EOL >> "\$node_name.py"


def callback(data):
    # Implementa qui la logica del listener
    pass

def listener():
    rospy.init_node('\$node_name', anonymous=True)
    rospy.Subscriber("topic_name", message_type, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
EOL
        ;;
    2) # Template per il Publisher
        echo 'Aggiungo il template per il Publisher al file...'
        cat <<EOL >> "\$node_name.py"


def publisher():
    rospy.init_node('\$node_name', anonymous=True)
    pub = rospy.Publisher("topic_name", message_type, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        # Implementa qui la logica del publisher
        pass
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
EOL
        ;;
    3) # Nessun template
        echo 'Nessun template aggiunto.'
        ;;
    *) # Scelta non valida
        echo 'Scelta non valida. Nessun template aggiunto.'
        ;;
esac

echo 'File \$node_name.py creato con il template desiderato. Vuoi aprirlo con un Code Editor? (y/n):'
read open_file
if [[ "\$open_file" == "y" || "\$open_file" == "Y" ]]; then
    code "\$node_name.py"
fi

" >> "$package_name"/scripts/create_node.sh

#chiedi se vuole creare un Ros node
echo "Vuoi creare un Ros node? (y/n):"
read create_node
#se si crea il nodo
while [[ "$create_node" == "y" || "$create_node" == "Y" ]]; do
    cd "$package_name"/scripts
    ./create_node.sh
    echo "Vuoi creare un Ros node? (y/n):"
    read create_node
done