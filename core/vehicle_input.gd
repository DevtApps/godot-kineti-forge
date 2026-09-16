class_name KFVehicleInput
extends RefCounted

## Valores brutos de entrada [0, 1] ou [-1, 1]
var raw_throttle: float = 0.0
var raw_brake: float = 0.0
var raw_steer: float = 0.0
var raw_handbrake: float = 0.0
var raw_clutch: float = 0.0

## Valores filtrados (após rate limit e input smoothing)
var throttle: float = 0.0
var brake: float = 0.0
var steer: float = 0.0
var handbrake: float = 0.0
var clutch: float = 0.0

## Solicitações de troca de marcha
var shift_up_requested: bool = false
var shift_down_requested: bool = false
var reset_requested: bool = false
var external_control: bool = false

## Configurações de suavização de entrada
var throttle_rise_rate: float = 8.0
var throttle_fall_rate: float = 12.0
var brake_rise_rate: float = 10.0
var brake_fall_rate: float = 14.0
var steer_rise_rate: float = 6.0
var steer_fall_rate: float = 10.0
var handbrake_rise_rate: float = 12.0
var handbrake_fall_rate: float = 12.0
var clutch_rise_rate: float = 8.0
var clutch_fall_rate: float = 8.0

func poll_inputs() -> void:
	if external_control:
		return
	raw_throttle = Input.get_action_strength("vehicle_accelerate")
	raw_brake = Input.get_action_strength("vehicle_brake")
	raw_steer = Input.get_action_strength("vehicle_steer_left") - Input.get_action_strength("vehicle_steer_right")
	raw_handbrake = Input.get_action_strength("vehicle_handbrake")
	
	if Input.is_action_just_pressed("vehicle_reset"):
		reset_requested = true
	if InputMap.has_action("vehicle_shift_up") and Input.is_action_just_pressed("vehicle_shift_up"):
		shift_up_requested = true
	if InputMap.has_action("vehicle_shift_down") and Input.is_action_just_pressed("vehicle_shift_down"):
		shift_down_requested = true

func set_external_controls(p_throttle: float, p_brake: float, p_steer: float, p_handbrake: float, p_clutch: float) -> void:
	external_control = true
	raw_throttle = clampf(p_throttle, 0.0, 1.0)
	raw_brake = clampf(p_brake, 0.0, 1.0)
	raw_steer = clampf(p_steer, -1.0, 1.0)
	raw_handbrake = clampf(p_handbrake, 0.0, 1.0)
	raw_clutch = clampf(p_clutch, 0.0, 1.0)

func update_smoothing(dt: float, vehicle_speed_ms: float = 0.0, speed_curve: Curve = null) -> void:
	# Ajuste de sensibilidade de esterçamento com base na velocidade
	var steer_scale: float = 1.0
	if speed_curve != null:
		var speed_norm: float = clampf(absf(vehicle_speed_ms) / 50.0, 0.0, 1.0) # 0 a 180 km/h
		steer_scale = speed_curve.sample_baked(speed_norm)
	
	var target_steer: float = raw_steer * steer_scale
	steer = KFFilters.approach_rate(steer, target_steer, steer_rise_rate, steer_fall_rate, dt)
	throttle = KFFilters.approach_rate(throttle, raw_throttle, throttle_rise_rate, throttle_fall_rate, dt)
	brake = KFFilters.approach_rate(brake, raw_brake, brake_rise_rate, brake_fall_rate, dt)
	handbrake = KFFilters.approach_rate(handbrake, raw_handbrake, handbrake_rise_rate, handbrake_fall_rate, dt)
	clutch = KFFilters.approach_rate(clutch, raw_clutch, clutch_rise_rate, clutch_fall_rate, dt)

func clear_transient_requests() -> void:
	shift_up_requested = false
	shift_down_requested = false
	reset_requested = false
