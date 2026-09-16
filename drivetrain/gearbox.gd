class_name KFGearbox
extends RefCounted

signal gear_changed(old_gear: int, new_gear: int)
signal shift_started(from_gear: int, to_gear: int)
signal shift_completed(gear: int)

var config: KFGearboxConfig
var input_shaft: KFRotationalBody
var output_shaft: KFRotationalBody

var current_gear: int = 1 # -1 = Ré, 0 = Neutro, 1..N = Marchas à frente
var target_gear: int = 1
var is_shifting: bool = false
var shift_timer: float = 0.0

func _init(p_config: KFGearboxConfig = null) -> void:
	config = p_config if p_config != null else KFGearboxConfig.new()
	input_shaft = KFRotationalBody.new(0.08, 0.0)
	output_shaft = KFRotationalBody.new(0.12, 0.0)

func get_gear_count() -> int:
	return config.forward_ratios.size()

func get_current_ratio() -> float:
	if is_shifting or current_gear == 0:
		return 0.0
	if current_gear == -1:
		return config.reverse_ratio
	if current_gear >= 1 and current_gear <= config.forward_ratios.size():
		return config.forward_ratios[current_gear - 1]
	return 0.0

func shift_to(new_gear: int) -> void:
	if new_gear < -1 or new_gear > config.forward_ratios.size() or new_gear == current_gear:
		return
	
	target_gear = new_gear
	if config.shift_time > 0.0:
		is_shifting = true
		shift_timer = config.shift_time
		shift_started.emit(current_gear, target_gear)
	else:
		_apply_gear_change(target_gear)

func shift_up() -> void:
	if current_gear == -1:
		shift_to(0)
	elif current_gear == 0:
		shift_to(1)
	elif current_gear < config.forward_ratios.size():
		shift_to(current_gear + 1)

func shift_down() -> void:
	if current_gear > 1:
		shift_to(current_gear - 1)
	elif current_gear == 1:
		shift_to(0)
	elif current_gear == 0:
		shift_to(-1)

func _apply_gear_change(new_gear: int) -> void:
	var old_gear: int = current_gear
	current_gear = new_gear
	is_shifting = false
	gear_changed.emit(old_gear, current_gear)
	shift_completed.emit(current_gear)

func step(dt: float) -> void:
	if is_shifting:
		shift_timer -= dt
		if shift_timer <= 0.0:
			_apply_gear_change(target_gear)
