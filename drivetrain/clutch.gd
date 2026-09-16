class_name KFClutch
extends RefCounted

var config: KFGearboxConfig
var pedal: float = 0.0 # 0.0 = acoplada, 1.0 = desengatada
var engagement: float = 1.0

var relative_angle: float = 0.0
var delta_omega: float = 0.0
var transmitted_torque: float = 0.0
var is_locked: bool = false

func _init(p_config: KFGearboxConfig = null) -> void:
	config = p_config if p_config != null else KFGearboxConfig.new()

## Resolve a transmissão de torque pela embreagem com modelo implícito de Euler
func step(
	engine_omega: float,
	gearbox_input_omega: float,
	dt: float,
	throttle: float = 0.0,
	is_automatic: bool = true,
	engine_inertia: float = 0.22,
	reflected_gearbox_inertia: float = 0.25,
	force_open: bool = false
) -> float:
	var engine_rpm: float = engine_omega * KFMath.RAD_S_TO_RPM
	if force_open:
		engagement = 0.0
		relative_angle = 0.0
		delta_omega = engine_omega - gearbox_input_omega
		transmitted_torque = 0.0
		is_locked = false
		return 0.0
	
	# Controle de embreagem automática / conversor de torque virtual
	if is_automatic:
		if engine_rpm < 950.0:
			# Marcha lenta: leve creep, nunca sobrecarrega o motor
			engagement = clampf(0.02 + throttle * 0.15, 0.0, 0.2)
		elif engine_rpm < 2200.0:
			# Faixa de arrancada progressiva: permite ao motor subir de giro
			var launch_factor: float = clampf((engine_rpm - 950.0) / 1250.0, 0.0, 1.0)
			# Curva suave de acoplamento
			var curve: float = launch_factor * launch_factor
			engagement = clampf(curve * (0.4 + 0.6 * throttle), 0.05, 1.0)
		else:
			engagement = 1.0
	else:
		engagement = clampf(1.0 - pedal, 0.0, 1.0)
	
	delta_omega = engine_omega - gearbox_input_omega
	
	if engagement <= 0.01:
		relative_angle = 0.0
		transmitted_torque = 0.0
		is_locked = false
		return 0.0
	
	var max_capacity: float = config.clutch_max_torque * engagement
	
	# Inércia reduzida do sistema para integração estável (KinetiForge)
	var j_g: float = maxf(reflected_gearbox_inertia, 0.01)
	var j_e: float = maxf(engine_inertia, 0.01)
	var j_total: float = (j_g * j_e) / (j_g + j_e)
	
	var k_clutch: float = config.clutch_stiffness
	var d_clutch: float = config.clutch_damping
	
	# Formulação implícita de Backward Euler do KinetiForge C++
	# Denominador = 1.0 + (K*dt + D)*dt / J_total
	var term: float = k_clutch * dt + d_clutch
	var num: float = k_clutch * relative_angle + term * delta_omega * engagement
	var den: float = 1.0 + (term * dt) / j_total
	
	var spring_torque: float = num / maxf(den, 0.001)
	
	if absf(spring_torque) > max_capacity:
		transmitted_torque = signf(spring_torque) * max_capacity
		is_locked = false
	else:
		transmitted_torque = spring_torque
		relative_angle += delta_omega * engagement * dt
		relative_angle = clampf(relative_angle, -0.15, 0.15)
		is_locked = (absf(delta_omega) < config.clutch_lock_threshold)
	
	return transmitted_torque
