class_name KFAutoGearbox
extends RefCounted

enum DirectionState {
	FORWARD,
	BRAKING_TO_REVERSE,
	REVERSE,
	BRAKING_TO_FORWARD
}

var enabled: bool = true
var gearbox: KFGearbox
var engine: KFEngine

var upshift_rpm: float = 6000.0
var downshift_rpm: float = 2300.0
var shift_delay: float = 0.55
var shift_cooldown_timer: float = 0.0
var downshift_lock_timer: float = 0.0
var post_upshift_hold: float = 1.25
var direction_state: DirectionState = DirectionState.FORWARD

func _init(p_gearbox: KFGearbox, p_engine: KFEngine, p_config: KFGearboxConfig = null) -> void:
	gearbox = p_gearbox
	engine = p_engine
	if p_config != null:
		upshift_rpm = p_config.auto_upshift_rpm
		downshift_rpm = p_config.auto_downshift_rpm
		shift_delay = p_config.auto_shift_delay
		post_upshift_hold = p_config.auto_post_upshift_hold
	gearbox.gear_changed.connect(_on_gear_changed)

func _on_gear_changed(_old_gear: int, new_gear: int) -> void:
	direction_state = DirectionState.REVERSE if new_gear == -1 else DirectionState.FORWARD
	if new_gear > _old_gear and _old_gear >= 1:
		downshift_lock_timer = post_upshift_hold

## Converte os dois pedais arcade em comandos físicos inequívocos. Somente
## esta classe decide quando W/S significa propulsão ou frenagem.
func resolve_drive_controls(
	throttle_pedal: float,
	brake_pedal: float,
	vehicle_forward_speed: float,
	raw_throttle: float = -1.0,
	raw_brake: float = -1.0
) -> Dictionary:
	var throttle_intent: float = throttle_pedal if raw_throttle < 0.0 else raw_throttle
	var brake_intent: float = brake_pedal if raw_brake < 0.0 else raw_brake
	var speed_abs: float = absf(vehicle_forward_speed)
	# O estado direcional nunca pode permanecer armado depois que o usuário
	# soltou os dois pedais. Valores suavizados não comandam a state machine.
	if throttle_intent < 0.01 and brake_intent < 0.01:
		direction_state = DirectionState.REVERSE if gearbox.current_gear == -1 else DirectionState.FORWARD
		return {"throttle": 0.0, "brake": 0.0}
	if gearbox.is_shifting:
		# Durante uma troca para frente a embreagem abre e o motor deixa de
		# tracionar, mas o acelerador jamais deve virar freio de servico.
		# A implementacao anterior aplicava 100% de freio em trocas a plena
		# carga, causando uma grande queda de velocidade entre as marchas.
		return {"throttle": 0.0, "brake": 0.0}

	if gearbox.current_gear == -1:
		if direction_state != DirectionState.BRAKING_TO_FORWARD and throttle_intent > 0.1:
			direction_state = DirectionState.BRAKING_TO_FORWARD
		if direction_state == DirectionState.BRAKING_TO_FORWARD:
			if throttle_intent < 0.05:
				direction_state = DirectionState.REVERSE
			elif speed_abs <= 0.35:
				gearbox.shift_to(1)
				return {"throttle": 0.0, "brake": 1.0}
			return {"throttle": 0.0, "brake": throttle_pedal}
		return {"throttle": brake_pedal, "brake": 0.0}

	if direction_state != DirectionState.BRAKING_TO_REVERSE and brake_intent > 0.1:
		direction_state = DirectionState.BRAKING_TO_REVERSE
	if direction_state == DirectionState.BRAKING_TO_REVERSE:
		if brake_intent < 0.05:
			direction_state = DirectionState.FORWARD
		elif speed_abs <= 0.35:
			gearbox.shift_to(-1)
			return {"throttle": 0.0, "brake": 1.0}
		return {"throttle": 0.0, "brake": brake_pedal}
	return {"throttle": throttle_pedal, "brake": brake_pedal}

func step(dt: float, throttle: float, _vehicle_forward_speed: float, _brake_input: float) -> void:
	downshift_lock_timer = maxf(0.0, downshift_lock_timer - dt)
	if not enabled or gearbox == null or engine == null or gearbox.is_shifting:
		return
	
	if shift_cooldown_timer > 0.0:
		shift_cooldown_timer -= dt
		return
	
	var engine_rpm: float = absf(engine.crankshaft.get_rpm())
	var road_rpm: float = absf(gearbox.input_shaft.get_rpm())
	# Durante slip da embreagem, o RPM cinemático impede que o câmbio fique
	# preso numa marcha curta apesar da alta velocidade das rodas.
	var shift_rpm: float = maxf(engine_rpm, road_rpm)
	var current_gear: int = gearbox.current_gear
	var speed_kmh: float = absf(_vehicle_forward_speed) * KFMath.MS_TO_KMH
	# Lógica de troca em movimento para frente
	if current_gear >= 1:
		var speed_forces_upshift: bool = false
		var upshift_speed_index: int = current_gear - 1
		if upshift_speed_index < gearbox.config.auto_upshift_speeds_kmh.size():
			speed_forces_upshift = speed_kmh >= gearbox.config.auto_upshift_speeds_kmh[upshift_speed_index]
		if (shift_rpm >= upshift_rpm or speed_forces_upshift) and current_gear < gearbox.get_gear_count():
			gearbox.shift_up()
			shift_cooldown_timer = shift_delay
		elif current_gear > 1 and downshift_lock_timer <= 0.0:
			var current_ratio: float = absf(gearbox.config.forward_ratios[current_gear - 1])
			var lower_ratio: float = absf(gearbox.config.forward_ratios[current_gear - 2])
			var predicted_lower_rpm: float = road_rpm * lower_ratio / maxf(current_ratio, 0.001)
			var safe_to_downshift: bool = predicted_lower_rpm < engine.config.redline_rpm * 0.92
			var downshift_speed_index: int = current_gear - 2
			if downshift_speed_index < gearbox.config.auto_downshift_speeds_kmh.size():
				safe_to_downshift = safe_to_downshift and speed_kmh <= gearbox.config.auto_downshift_speeds_kmh[downshift_speed_index]
			var low_rpm_request: bool = shift_rpm <= downshift_rpm
			var kickdown_request: bool = throttle > 0.9 and shift_rpm < 2800.0
			if safe_to_downshift and (low_rpm_request or kickdown_request):
				gearbox.shift_down()
				shift_cooldown_timer = shift_delay
