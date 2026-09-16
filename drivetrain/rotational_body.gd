class_name KFRotationalBody
extends RefCounted

var omega: float = 0.0 # Velocidade angular em rad/s
var inertia: float = 0.20 # Momento de inércia em kg·m²
var torque: float = 0.0 # Torque resultante em N·m

func _init(p_inertia: float = 0.20, p_initial_omega: float = 0.0) -> void:
	inertia = maxf(p_inertia, 0.001)
	omega = p_initial_omega

func get_rpm() -> float:
	return omega * KFMath.RAD_S_TO_RPM

func set_rpm(rpm: float) -> void:
	omega = rpm * KFMath.RPM_TO_RAD_S

func integrate(dt: float) -> void:
	var alpha: float = torque / inertia
	omega += alpha * dt

