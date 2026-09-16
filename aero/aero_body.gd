class_name KFAeroBody
extends RefCounted

var config: KFVehicleConfig

var current_drag_force: Vector3 = Vector3.ZERO
var current_downforce: Vector3 = Vector3.ZERO

func _init(p_config: KFVehicleConfig = null) -> void:
	config = p_config

func step(body_state: PhysicsDirectBodyState3D, dt: float) -> void:
	if config == null:
		return
	
	var v: Vector3 = body_state.linear_velocity
	var speed: float = v.length()
	if speed < 0.5:
		current_drag_force = Vector3.ZERO
		current_downforce = Vector3.ZERO
		return
	
	# 1. Arrasto Aerodinâmico (Aerodynamic Drag)
	# Fdrag = -0.5 * rho * Cd * A * |v| * v
	var drag_magnitude: float = 0.5 * config.air_density * config.drag_coefficient * config.frontal_area * speed * speed
	var drag_dir: Vector3 = -v.normalized()
	current_drag_force = drag_dir * drag_magnitude
	
	body_state.apply_central_force(current_drag_force)
	
	# 2. Downforce Aerodinâmico (Sustentação negativa / aderência em alta velocidade)
	# Fdown = 0.5 * rho * Cl * A * v²
	var forward_speed: float = -body_state.transform.basis.z.dot(v)
	if forward_speed > 1.0:
		var speed_sq: float = forward_speed * forward_speed
		var q: float = 0.5 * config.air_density * config.frontal_area * speed_sq
		
		var down_dir: Vector3 = -body_state.transform.basis.y.normalized()
		
		var front_down: float = q * config.front_downforce_coeff
		var rear_down: float = q * config.rear_downforce_coeff
		
		var front_force: Vector3 = down_dir * front_down
		var rear_force: Vector3 = down_dir * rear_down
		current_downforce = front_force + rear_force
		var com_world: Vector3 = body_state.transform * body_state.center_of_mass
		var front_world: Vector3 = body_state.transform * config.front_aero_position
		var rear_world: Vector3 = body_state.transform * config.rear_aero_position
		body_state.apply_central_force(current_downforce)
		body_state.apply_torque((front_world - com_world).cross(front_force))
		body_state.apply_torque((rear_world - com_world).cross(rear_force))
	else:
		current_downforce = Vector3.ZERO

	# AeroVolume: damping anisotrópico nos eixos locais do veículo.
	var inverse_basis: Basis = body_state.transform.basis.inverse()
	var local_velocity: Vector3 = inverse_basis * body_state.linear_velocity
	var local_angular_velocity: Vector3 = inverse_basis * body_state.angular_velocity
	var local_damping_force: Vector3 = -local_velocity * config.aero_linear_damping
	var local_damping_torque: Vector3 = -local_angular_velocity * config.aero_angular_damping
	body_state.apply_central_force(body_state.transform.basis * local_damping_force)
	body_state.apply_torque(body_state.transform.basis * local_damping_torque)
