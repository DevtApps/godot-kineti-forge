class_name KFDifferentialConfig
extends Resource

enum DifferentialType {
	OPEN,
	LOCKED,
	CLUTCH_LSD,
	TORSEN,
	VISCOUS
}

@export_group("General")
## Tipo de diferencial do eixo.
@export var type: DifferentialType = DifferentialType.CLUTCH_LSD
## Relação de redução final (Final Drive / Coroa e Pinhão).
@export var final_drive: float = 3.90
## Eficiência mecânica do diferencial.
@export var efficiency: float = 0.98

@export_group("Limited Slip / Lock")
## Pré-carga de bloqueio em N·m (torque de travamento residual sem aceleração).
@export var preload_torque: float = 60.0
## Ganho de bloqueio em aceleração (Ramp Gain em % do torque de entrada).
@export var power_ramp_gain: float = 0.50
## Ganho de bloqueio em desaceleração / freio motor (Coast Ramp Gain).
@export var coast_ramp_gain: float = 0.25
## Torque máximo absoluto de bloqueio em N·m.
@export var max_lock_torque: float = 750.0
## Torque Bias Ratio (para diferenciais tipo Torsen, ex: 3.0:1).
@export var torque_bias_ratio: float = 3.0
## Coeficiente viscoso em N·m / (rad/s) para diferenciais viscosos/LSD.
@export var viscous_coefficient: float = 25.0

