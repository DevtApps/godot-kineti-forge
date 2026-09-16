class_name KFBrake
extends RefCounted

var max_brake_torque: float = 3400.0
var front_bias: float = 0.65
var handbrake_torque: float = 2400.0

func _init(p_max_torque: float = 3400.0, p_front_bias: float = 0.65, p_handbrake_torque: float = 2400.0) -> void:
	max_brake_torque = p_max_torque
	front_bias = p_front_bias
	handbrake_torque = p_handbrake_torque

func apply_brakes(
	brake_input: float,
	handbrake_input: float,
	front_axle: KFAxle,
	rear_axle: KFAxle
) -> void:
	var total_service_torque: float = brake_input * max_brake_torque
	var front_torque_per_wheel: float = (total_service_torque * front_bias) * 0.5
	var rear_torque_per_wheel: float = (total_service_torque * (1.0 - front_bias)) * 0.5
	
	var rear_handbrake_per_wheel: float = (handbrake_input * handbrake_torque) * 0.5
	
	if front_axle != null:
		front_axle.left_wheel.state.service_brake_torque = front_torque_per_wheel
		front_axle.right_wheel.state.service_brake_torque = front_torque_per_wheel
		front_axle.left_wheel.state.handbrake_torque = 0.0
		front_axle.right_wheel.state.handbrake_torque = 0.0
		front_axle.left_wheel.state.brake_torque = front_torque_per_wheel
		front_axle.right_wheel.state.brake_torque = front_torque_per_wheel
	
	if rear_axle != null:
		rear_axle.left_wheel.state.service_brake_torque = rear_torque_per_wheel
		rear_axle.right_wheel.state.service_brake_torque = rear_torque_per_wheel
		rear_axle.left_wheel.state.handbrake_torque = rear_handbrake_per_wheel
		rear_axle.right_wheel.state.handbrake_torque = rear_handbrake_per_wheel
		rear_axle.left_wheel.state.brake_torque = rear_torque_per_wheel + rear_handbrake_per_wheel
		rear_axle.right_wheel.state.brake_torque = rear_torque_per_wheel + rear_handbrake_per_wheel
