class_name KFWheel
extends RefCounted

var wheel_index: int = 0
var name: String = "Wheel"

## Referências de nós
var mount_node: Node3D
var visual_node: Node3D # Nó da roda / pneu (recebe Steer + Camber + Spin)
var hub_node: Node3D # Nó do cubo / pinça de freio (recebe Steer + Camber, SEM Spin)
var local_mount_position: Vector3 = Vector3.ZERO

## Configurações
var tire_config: KFTireConfig
var suspension_config: KFSuspensionConfig

## Componentes
var state: KFWheelState
var suspension: KFSuspension
var contact_shape: SphereShape3D

## Atributos funcionais
var is_front: bool = false
var is_left: bool = true
var is_steered: bool = false
var is_driven: bool = true

func _init(
	p_index: int,
	p_name: String,
	p_is_front: bool,
	p_is_left: bool,
	p_tire_config: KFTireConfig,
	p_susp_config: KFSuspensionConfig
) -> void:
	wheel_index = p_index
	name = p_name
	is_front = p_is_front
	is_left = p_is_left
	is_steered = p_is_front
	
	tire_config = p_tire_config if p_tire_config != null else KFTireConfig.new()
	suspension_config = p_susp_config if p_susp_config != null else KFSuspensionConfig.new()
	
	state = KFWheelState.new()
	suspension = KFSuspension.new(suspension_config)
	state.suspension_length = suspension_config.rest_length
	contact_shape = SphereShape3D.new()
	contact_shape.radius = tire_config.radius

## Obtém a posição global do ponto de fixação da suspensão
func get_mount_global_position(vehicle_transform: Transform3D) -> Vector3:
	if mount_node != null:
		return mount_node.global_position
	return vehicle_transform * local_mount_position

## 1. Passo de detecção de contato com o solo
func step_contact(
	body_state: PhysicsDirectBodyState3D,
	vehicle_rid: RID,
	road_mask: int
) -> bool:
	var mount_pos: Vector3 = get_mount_global_position(body_state.transform)
	var susp_down: Vector3 = -body_state.transform.basis.y.normalized()
	
	var has_contact: bool = KFContactSolver.cast_wheel_shape(
		body_state.get_space_state(),
		vehicle_rid,
		mount_pos,
		susp_down,
		maxf(state.suspension_length, suspension_config.rest_length),
		0.0,
		contact_shape,
		tire_config.shape_cast_margin,
		road_mask,
		state
	)
	if has_contact:
		var vehicle_up: Vector3 = body_state.transform.basis.y.normalized()
		var contact_alignment: float = state.contact_normal.dot(vehicle_up)
		if contact_alignment < suspension_config.contact_normal_min_dot:
			state.in_contact = false
			state.suspension_contact_factor = 0.0
			state.compression = 0.0
			state.normal_load = 0.0
			state.longitudinal_force = 0.0
			state.lateral_force = 0.0
			return false
		state.suspension_contact_factor = smoothstep(
			suspension_config.contact_normal_min_dot,
			suspension_config.contact_normal_full_force_dot,
			contact_alignment
		)
	else:
		state.suspension_contact_factor = 0.0
	return has_contact

## 2. Passo de resolução da suspensão
func step_suspension(body_state: PhysicsDirectBodyState3D, dt: float) -> float:
	var mount_pos: Vector3 = get_mount_global_position(body_state.transform)
	return suspension.solve(mount_pos, tire_config.radius, state, dt)

## 3. Passo de construção dos eixos da roda (Forward, Right, Up)
func update_wheel_basis(
	vehicle_basis: Basis,
	target_steer_angle: float
) -> void:
	state.steer_angle = target_steer_angle if is_steered else 0.0
	
	# Rotação em torno do eixo Y local (esterçamento)
	var toe_sign: float = 1.0 if is_left else -1.0
	var steer_basis: Basis = Basis(Vector3.UP, state.steer_angle + state.toe_angle * toe_sign)
	
	# Camber (inclinação em torno do eixo Z)
	var camber_sign: float = -1.0 if is_left else 1.0
	var camber_rad: float = state.camber_angle * camber_sign
	var camber_basis: Basis = Basis(Vector3.FORWARD, camber_rad)
	var caster_basis: Basis = Basis(Vector3.RIGHT, state.caster_angle)
	
	# Combina orientação do chassi com esterçamento e camber
	var combined_basis: Basis = vehicle_basis * caster_basis * steer_basis * camber_basis
	
	# Vetor normal do solo (se em contato) ou UP do veículo
	var n: Vector3 = state.contact_normal if state.in_contact else vehicle_basis.y.normalized()
	
	# Forward da roda (na convenção Godot, -Z é para frente)
	var raw_forward: Vector3 = -combined_basis.z.normalized()
	
	# Projeta forward no plano do solo
	var forward_proj: Vector3 = (raw_forward - n * raw_forward.dot(n)).normalized()
	if forward_proj.length_squared() < 0.001:
		forward_proj = -vehicle_basis.z.normalized()
	
	var right_proj: Vector3 = forward_proj.cross(n).normalized()
	
	state.wheel_forward = forward_proj
	state.wheel_right = right_proj
	state.wheel_up = n

## 4. Passo de resolução do pneu
func step_tire(body_state: PhysicsDirectBodyState3D, dt: float) -> void:
	step_tire_kinematics(body_state, dt)
	step_tire_forces(dt)

func step_tire_kinematics(body_state: PhysicsDirectBodyState3D, dt: float) -> void:
	if not state.in_contact:
		state.normal_load = 0.0
		state.longitudinal_force = 0.0
		state.lateral_force = 0.0
		state.total_force_world = Vector3.ZERO
		state.relaxed_slip_ratio = 0.0
		state.relaxed_slip_angle = 0.0
		return
	
	# PhysicsDirectBodyState3D não expõe a massa do corpo. A massa efetiva
	# instantânea da roda é derivada da carga vertical, incluindo transferência
	# de carga e ação da barra estabilizadora.
	state.effective_sprung_mass = maxf(
		state.normal_load / KFMath.GRAVITY_CONSTANT,
		1.0
	)
	
	# Calcula a velocidade do ponto de contato
	var com_world: Vector3 = body_state.transform * body_state.center_of_mass
	var r: Vector3 = state.contact_point - com_world
	var point_vel: Vector3 = body_state.linear_velocity + body_state.angular_velocity.cross(r)
	
	# 4a. Calcula slips
	KFSlipSolver.solve_slip(
		point_vel,
		state.wheel_forward,
		state.wheel_right,
		state.omega,
		tire_config.radius,
		state
	)
	
	# 4b. Relaxation length com transport speed regularizado
	KFRelaxationSolver.update_relaxation(tire_config, dt, state)
	

func step_tire_forces(dt: float) -> void:
	KFTireModel.solve_forces(tire_config, state, dt)

## 5. Aplica as forças da suspensão e dos pneus no RigidBody3D seguindo o KinetiForge C++
func apply_forces_to_body(body_state: PhysicsDirectBodyState3D) -> void:
	if not state.in_contact or state.normal_load <= 0.0:
		return
	
	var com_world: Vector3 = body_state.transform * body_state.center_of_mass
	
	# 5a. Força da suspensão projetada ao longo da normal de contato (ImpactNormal)
	# No KinetiForge C++: SuspensionForceProj = ImpactNormal * ForceAlongImpactNormal
	# Isso cancela perfeitamente a gravidade sem gerar vetores parasitas de deslocamento horizontal
	var susp_force_world: Vector3 = state.contact_normal * state.normal_load
	
	# 5b. Forças coplanares do pneu (Tração/Frenagem Fx e Lateral Fy)
	var tire_planar_force: Vector3 = (
		state.wheel_forward * state.longitudinal_force +
		state.wheel_right * state.lateral_force
	)
	
	# Braços de alavanca em relação ao Centro de Massa
	var r_contact: Vector3 = state.contact_point - com_world
	
	var total_force: Vector3 = susp_force_world + tire_planar_force
	# Todas as reações externas do solo atuam no patch de contato. Aplicar a
	# suspensão no centro da roda remove parte do braço de alavanca e distorce
	# roll/pitch em inclinações e obstáculos.
	var total_torque: Vector3 = r_contact.cross(total_force)
	
	body_state.apply_central_force(total_force)
	body_state.apply_torque(total_torque)

## 6. Integração da rotação da roda com o algoritmo WheelAcceleration do KinetiForge C++
func integrate_rotation(dt: float) -> void:
	var eff_inertia: float = maxf(state.effective_inertia, 0.05)
	var eff_inertia_inv: float = 1.0 / eff_inertia
	
	# Torque de atrito longitudinal gerado pelo contato pneu-solo
	var friction_torque: float = state.longitudinal_force * tire_config.radius
	
	# Torque necessário para inverter o sinal da rotação relativa pneu-solo
	var angular_long_slip: float = state.omega - (state.vx / maxf(tire_config.radius, 0.01))
	var max_friction_torque: float = absf((angular_long_slip * eff_inertia / maxf(dt, 0.0001)) + state.drive_torque)
	
	# Limita o torque de atrito para que NUNCA ultrapasse a velocidade da pista (elimina oscilação)
	var clamped_friction_torque: float = clampf(friction_torque, -max_friction_torque, max_friction_torque)
	var excess_friction_torque: float = clampf(friction_torque - clamped_friction_torque, -state.brake_torque, state.brake_torque)
	
	# Integração da velocidade angular sob torque do motor e atrito do solo
	state.omega += (state.drive_torque - clamped_friction_torque) * eff_inertia_inv * dt
	var ang_vel_sign: float = signf(state.omega)
	
	# Aplicação do torque de freio se opondo ao sentido de rotação
	var actual_brake_torque: float = -ang_vel_sign * state.brake_torque
	state.omega += (actual_brake_torque - excess_friction_torque) * eff_inertia_inv * dt
	
	# Zero cross check: se o freio reduziu a rotação a zero, crava em zero
	if ang_vel_sign * state.omega <= 0.0 and state.brake_torque > 0.05:
		state.omega = 0.0
	
	# Resistência ao rolamento suave
	if absf(state.omega) > 0.05:
		var roll_decay: float = (state.rolling_torque * eff_inertia_inv) * dt
		state.omega = move_toward(state.omega, 0.0, absf(roll_decay))

	# Sem reação do solo a roda pode acelerar indefinidamente. O arrasto de
	# rolamentos/ar limita a energia acumulada antes do próximo contato.
	if not state.in_contact:
		state.omega *= exp(-maxf(tire_config.airborne_angular_drag, 0.0) * dt)
		var max_airborne_omega: float = tire_config.max_wheel_rpm * KFMath.RPM_TO_RAD_S
		state.omega = clampf(state.omega, -max_airborne_omega, max_airborne_omega)
	
	# Atualiza RPM
	state.rpm = state.omega * KFMath.RAD_S_TO_RPM
	
	# Atualiza o ângulo visual acumulado (spin)
	state.spin_angle = fposmod(state.spin_angle + state.omega * dt, TAU)

## 7. Atualiza o transform dos nós visuais (Cubo e Roda)
func update_visual(vehicle_transform: Transform3D) -> void:
	var mount_pos: Vector3 = get_mount_global_position(vehicle_transform)
	var susp_down: Vector3 = -vehicle_transform.basis.y.normalized()
	
	# Deslocamento da suspensão a partir do topo
	var current_dist: float = state.suspension_length
	var wheel_center_pos: Vector3 = mount_pos + susp_down * current_dist
	
	var toe_sign: float = 1.0 if is_left else -1.0
	var steer_quat := Quaternion(Vector3.UP, state.steer_angle + state.toe_angle * toe_sign)
	var camber_sign: float = -1.0 if is_left else 1.0
	var camber_quat := Quaternion(Vector3.FORWARD, state.camber_angle * camber_sign)
	var caster_quat := Quaternion(Vector3.RIGHT, state.caster_angle)
	var spin_quat := Quaternion(Vector3.RIGHT, -state.spin_angle)
	
	# 7a. Atualiza cubo / manga de eixo (apenas Steer + Camber, SEM Spin)
	if hub_node != null:
		hub_node.global_position = wheel_center_pos
		hub_node.global_basis = vehicle_transform.basis * Basis(caster_quat * steer_quat * camber_quat)
	
	# 7b. Atualiza pneu / roda (Steer + Camber + Spin)
	if visual_node != null:
		visual_node.global_position = wheel_center_pos
		visual_node.global_basis = vehicle_transform.basis * Basis(caster_quat * steer_quat * camber_quat * spin_quat)
