class_name KFDifferential
extends RefCounted

var config: KFDifferentialConfig

var carrier_omega: float = 0.0
var input_omega: float = 0.0

var left_torque: float = 0.0
var right_torque: float = 0.0
var lock_torque: float = 0.0

func _init(p_config: KFDifferentialConfig = null) -> void:
	config = p_config if p_config != null else KFDifferentialConfig.new()

## Resolve a distribuição de torque entre as rodas baseado no modelo KinetiForge C++
func solve(
	input_torque: float,
	left_omega: float,
	right_omega: float,
	dt: float = 0.00833,
	left_inertia: float = 1.35,
	right_inertia: float = 1.35
) -> void:
	carrier_omega = (left_omega + right_omega) * 0.5
	input_omega = carrier_omega * config.final_drive
	
	var base_torque: float = (input_torque * config.final_drive * config.efficiency) * 0.5
	var omega_diff: float = left_omega - right_omega
	
	# Inércia reduzida dos dois lados para torque de bloqueio exato em 1 passo
	var reduced_inertia: float = (left_inertia * right_inertia) / maxf(left_inertia + right_inertia, 0.001)
	var max_transfer_torque: float = (reduced_inertia * omega_diff) / maxf(dt, 0.0001)
	
	lock_torque = 0.0
	
	match config.type:
		KFDifferentialConfig.DifferentialType.OPEN:
			lock_torque = 0.0
			
		KFDifferentialConfig.DifferentialType.LOCKED:
			# Trava total (Spool / 100% lock)
			lock_torque = clampf(max_transfer_torque, -config.max_lock_torque, config.max_lock_torque)
			
		KFDifferentialConfig.DifferentialType.CLUTCH_LSD:
			var is_drive: bool = (base_torque * carrier_omega) >= 0.0
			var ramp_gain: float = config.power_ramp_gain if is_drive else config.coast_ramp_gain
			var lock_capacity: float = minf(config.preload_torque + absf(input_torque) * ramp_gain, config.max_lock_torque)
			var lock_ratio: float = clampf(lock_capacity / maxf(config.max_lock_torque, 1.0), 0.1, 0.95)
			lock_torque = clampf(max_transfer_torque * lock_ratio, -lock_capacity, lock_capacity)
			
		KFDifferentialConfig.DifferentialType.TORSEN:
			var tbr: float = maxf(config.torque_bias_ratio, 1.0)
			var max_bias_torque: float = absf(base_torque) * ((tbr - 1.0) / (tbr + 1.0))
			lock_torque = clampf(max_transfer_torque * 0.7, -max_bias_torque, max_bias_torque)
			
		KFDifferentialConfig.DifferentialType.VISCOUS:
			lock_torque = clampf(-omega_diff * config.viscous_coefficient, -config.max_lock_torque, config.max_lock_torque)
	
	# Quando a roda esquerda gira mais rápido (omega_diff > 0), o diferencial transfere torque
	# DA roda esquerda PARA a direita:
	left_torque = base_torque - lock_torque
	right_torque = base_torque + lock_torque
