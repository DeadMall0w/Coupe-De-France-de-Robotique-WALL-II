import serial
from glob import glob


def detect_lidar_port() -> str:
	candidates = sorted(glob('/dev/ttyUSB*') + glob('/dev/ttyACM*'))
	if not candidates:
		raise RuntimeError('Aucun port série détecté (/dev/ttyUSB* ou /dev/ttyACM*)')
	return candidates[0]


# Cela force le signal DTR à False, ce qui souvent arrête le moteur, 
# et True pour le démarrer sur les adaptateurs USB Slamtec
port = detect_lidar_port()
print(f'Port détecté: {port}')
ser = serial.Serial(port, 115200)
ser.dtr = False # Essayer True ou False selon le modèle