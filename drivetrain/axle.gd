class_name KFAxle
extends RefCounted

var left_wheel: KFWheel
var right_wheel: KFWheel
var differential: KFDifferential

var is_front: bool = false
var is_steered: bool = false
var is_driven: bool = true
var anti_roll_rate: float = 12000.0
var anti_roll_damping: float = 1600.0

func _init(
	p_left_wheel: KFWheel,
	p_right_wheel: KFWheel,
	p_is_front: bool,
	p_diff_config: KFDifferentialConfig = null,
	p_anti_roll_rate: float = 12000.0,
	p_anti_roll_damping: float = 1600.0
) -> void:
	left_wheel = p_left_wheel
	right_wheel = p_right_wheel
	is_front = p_is_front
	is_steered = p_is_front
	anti_roll_rate = p_anti_roll_rate
	anti_roll_damping = p_anti_roll_damping
	differential = KFDifferential.new(p_diff_config)

func solve_anti_roll() -> void:
	KFAntiRollBar.apply_anti_roll(
		left_wheel.state,
		right_wheel.state,
		anti_roll_rate,
		anti_roll_damping
	)

func distribute_torque(drive_torque: float, dt: float = 0.00833) -> void:
	if not is_driven:
		left_wheel.state.drive_torque = 0.0
		right_wheel.state.drive_torque = 0.0
		return
	
	var i_left: float = left_wheel.tire_config.rotational_inertia if left_wheel.tire_config != null else 1.35
	var i_right: float = right_wheel.tire_config.rotational_inertia if right_wheel.tire_config != null else 1.35
	
	differential.solve(drive_torque, left_wheel.state.omega, right_wheel.state.omega, dt, i_left, i_right)
	left_wheel.state.drive_torque = differential.left_torque
	right_wheel.state.drive_torque = differential.right_torque
