class_name KFGearboxConfig
extends Resource

enum TransmissionMode {
	MANUAL,
	SEQUENTIAL,
	AUTOMATIC
}

@export_group("Mode & Timing")
## Tipo de transmissão.
@export var mode: TransmissionMode = TransmissionMode.AUTOMATIC
## Tempo de troca de marcha em segundos (desengate -> engate).
@export var shift_time: float = 0.18
## Eficiência mecânica da transmissão (ex: 0.96 = 4% de perdas por atrito).
@export var efficiency: float = 0.96

@export_group("Gear Ratios")
## Relação da marcha à ré (valor negativo).
@export var reverse_ratio: float = -3.4
## Relações das marchas à frente (1ª, 2ª, 3ª, 4ª, 5ª, 6ª...).
@export var forward_ratios: PackedFloat32Array = [
	3.50, # 1st
	2.15, # 2nd
	1.50, # 3rd
	1.15, # 4th
	0.92, # 5th
	0.75  # 6th
]

@export_group("Clutch Configuration")
## Torque máximo suportado pela embreagem antes de patinar em N·m.
@export var clutch_max_torque: float = 650.0
## Rigidez torsional do acoplamento da embreagem em N·m/rad.
@export var clutch_stiffness: float = 8500.0
## Amortecimento torsional da embreagem em N·m·s/rad.
@export var clutch_damping: float = 120.0
## Limiar de velocidade relativa para regime de lock em rad/s.
@export var clutch_lock_threshold: float = 2.0

@export_group("Automatic Logic")
## Rotação para subir marcha (upshift) em WOT (RPM).
@export var auto_upshift_rpm: float = 6200.0
## Rotação para reduzir marcha (downshift) (RPM).
@export var auto_downshift_rpm: float = 2200.0
## Tempo de histerese mínimo entre trocas consecutivas em segundos.
@export var auto_shift_delay: float = 0.6
## Velocidade máxima por marcha antes de forçar upshift (km/h). O índice 0
## corresponde à primeira, índice 1 à segunda, etc.
@export var auto_upshift_speeds_kmh: PackedFloat32Array = [35.0, 65.0, 100.0, 140.0, 180.0]
## Velocidade máxima para permitir redução até cada marcha (km/h).
## Índice 0 controla 2ª -> 1ª, índice 1 controla 3ª -> 2ª, etc.
@export var auto_downshift_speeds_kmh: PackedFloat32Array = [18.0, 45.0, 75.0, 110.0, 150.0]
@export var auto_post_upshift_hold: float = 1.25
