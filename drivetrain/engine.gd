class_name KFEngine
extends RefCounted

signal engine_started
signal engine_stalled
signal rev_limiter_hit
signal backfire(intensity: float)

var config: KFEngineConfig
var crankshaft: KFRotationalBody

var is_running: bool = false
var starter_active: bool = false
var ignition_enabled: bool = true
var fuel_enabled: bool = true

var throttle_input: float = 0.0
var turbo_boost: float = 0.0
var exhaust_heat: float = 0.0
var unburned_fuel: float = 0.0
var backfire_intensity: float = 0.0
var previous_throttle: float = 0.0

var combustion_torque: float = 0.0
var friction_torque: float = 0.0
var starter_torque: float = 0.0
var net_engine_torque: float = 0.0

var is_rev_limiting: bool = false

func _init(p_config: KFEngineConfig = null) -> void:
	config = p_config if p_config != null else KFEngineConfig.new()
	crankshaft = KFRotationalBody.new(config.flywheel_inertia, config.idle_rpm * KFMath.RPM_TO_RAD_S)
	is_running = true

func start() -> void:
	ignition_enabled = true
	fuel_enabled = true
	starter_active = true

func stop() -> void:
	is_running = false
	starter_active = false
	ignition_enabled = false
	fuel_enabled = false

func step(dt: float, clutch_reaction_torque: float = 0.0) -> void:
	var rpm: float = crankshaft.get_rpm()
	var abs_rpm: float = absf(rpm)
	var omega: float = crankshaft.omega
	
	# 1. Rotação do motor de arranque
	starter_torque = 0.0
	if starter_active:
		if abs_rpm < config.starter_max_rpm:
			starter_torque = config.starter_torque
		else:
			starter_active = false
			if not is_running:
				is_running = true
				engine_started.emit()
	
	# 2. Push-Start (tranco) se o motor estiver sendo girado externamente
	if not is_running and ignition_enabled and fuel_enabled:
		if abs_rpm >= config.push_start_min_rpm and crankshaft.omega > 0.0:
			is_running = true
			engine_started.emit()
	
	# 3. Limitador de Rotação (Rev Limiter)
	if abs_rpm >= config.limiter_rpm:
		is_rev_limiting = true
		rev_limiter_hit.emit()
	elif abs_rpm < config.redline_rpm:
		is_rev_limiting = false
	
	# 4. Turbocompressor
	if config.turbo_enabled:
		var target_boost: float = config.turbo_max_boost * throttle_input * clampf(abs_rpm / config.redline_rpm, 0.0, 1.0)
		if config.anti_lag_enabled and throttle_input < 0.15 and abs_rpm >= config.anti_lag_min_rpm:
			target_boost = maxf(target_boost, config.turbo_max_boost * config.anti_lag_target_boost_ratio)
		var response: float = 1.0 - exp(-dt / maxf(config.turbo_time_constant, 0.05))
		turbo_boost = lerpf(turbo_boost, target_boost, response)
	else:
		turbo_boost = 0.0
	
	# 5. Torque de Combustão
	combustion_torque = 0.0
	if is_running and ignition_enabled and fuel_enabled and not is_rev_limiting:
		var base_torque: float = config.get_torque_at_rpm(abs_rpm)
		var boost_mult: float = 1.0 + turbo_boost * config.turbo_torque_gain
		
		# Regulador de marcha lenta (Idle Governor) ativo
		var effective_throttle: float = throttle_input
		if effective_throttle < 0.1 and abs_rpm < config.idle_rpm and crankshaft.omega >= 0.0:
			var idle_error: float = clampf((config.idle_rpm - abs_rpm) / config.idle_rpm, 0.0, 1.0)
			effective_throttle = maxf(effective_throttle, idle_error * 0.5 + 0.12)
		
		combustion_torque = base_torque * effective_throttle * boost_mult
	
	# 6. Fricção Interna e Freio Motor
	var omega_abs: float = absf(omega)
	var friction_mag: float = (
		config.friction_static +
		config.friction_linear * omega_abs +
		config.friction_quadratic * omega_abs * omega_abs
	)
	
	if throttle_input < 0.05:
		friction_mag += config.engine_braking_torque * (abs_rpm / config.redline_rpm)
	
	var friction_sign: float = signf(omega) if omega_abs > 1.0 else (omega / 1.0)
	friction_torque = -friction_sign * friction_mag
	
	# 7. Balanço total de torque no virabrequim
	net_engine_torque = combustion_torque + friction_torque + starter_torque + clutch_reaction_torque
	crankshaft.torque = net_engine_torque
	
	# Integra a rotação do virabrequim
	crankshaft.integrate(dt)
	
	# Stall real: o eixo permanece livre e pode ser girado novamente pelo drivetrain.
	if is_running and not starter_active and absf(crankshaft.get_rpm()) < config.stall_rpm:
		var available_torque: float = combustion_torque + starter_torque
		if available_torque + clutch_reaction_torque <= friction_mag:
			is_running = false
			engine_stalled.emit()

	_update_exhaust(dt, abs_rpm)
	previous_throttle = throttle_input

func _update_exhaust(dt: float, rpm: float) -> void:
	var fuel_flow: float = throttle_input * clampf(rpm / maxf(config.redline_rpm, 1.0), 0.0, 1.2)
	var anti_lag_active: bool = config.anti_lag_enabled and throttle_input < 0.15 and rpm >= config.anti_lag_min_rpm
	if anti_lag_active:
		fuel_flow += 0.25
	var target_heat: float = clampf(fuel_flow, 0.0, 1.0)
	exhaust_heat = move_toward(exhaust_heat, target_heat, (config.exhaust_heat_rate if target_heat > exhaust_heat else config.exhaust_cooling_rate) * dt)
	unburned_fuel = maxf(0.0, unburned_fuel - dt * 0.8)
	if previous_throttle - throttle_input > 0.35 or anti_lag_active:
		unburned_fuel = minf(1.0, unburned_fuel + fuel_flow * dt + (previous_throttle - throttle_input) * 0.2)
	backfire_intensity = 0.0
	if exhaust_heat >= config.backfire_heat_threshold and unburned_fuel >= config.backfire_fuel_threshold:
		backfire_intensity = clampf(exhaust_heat * unburned_fuel, 0.0, 1.0)
		unburned_fuel *= 0.35
		backfire.emit(backfire_intensity)
