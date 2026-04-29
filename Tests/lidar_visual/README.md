- lancer la rpi
- la connecter en ssh à un reseau (ssid : "."; mdp : "toipouvoirs")
- se connecter sur son pc sur le même réseau que la pi
- sur laptop trouver l'ip de la pi : "nmap -sn MONIP/24" (trouver son ip avec "ip a")
- se connecter en ssh à la pi : "ssh wall-ii@IP_PI"

- envoyer le code de la teensy si elle l'a oublié (utiliser platformIO ou arduino IDE)

Une fois que tout est fait :
- en ssh sur rpi, lancer depuis lidar_visual : "make serve"
- en ssh sur rpi, lancer dans une autre terminal depuis lidar_visual : "./test_lidar_visual"