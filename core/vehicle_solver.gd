class_name KFVehicleSolver
extends RefCounted

var body: RigidBody3D
var config: KFVehicleConfig

var input: KFVehicleInput
var state: KFVehicleState

var wheels: Array[KFWheel] = []
var front_axle: KFAxle
var rear_axle: KFAxle

var engine: KFEngine
var clutch: KFClutch
var gearbox: KFGearbox
var brake: KFBrake
var drive_assembly: KFDriveAssembly

var abs_assist: KFAssistABS
var tcs_assist: KFAssistTCS
var auto_gearbox: KFAutoGearbox
var aero: KFAeroBody

var is_initialized: bool = false
var previous_linear_velocity: Vector3 = Vector3.ZERO

func initialize(
	p_body: RigidBody3D,
	p_config: KFVehicleConfig,
	mount_nodes: Dictionary = {}, # "FL", "FR", "RL", "RR"
	visual_nodes: Dictionary = {},
	hub_nodes: Dictionary = {}
) -> void:
	body = p_body
	config = p_config if p_config != null else KFVehicleConfig.new()
	
	input = KFVehicleInput.new()
	input.steer_rise_rate = config.steer_response_rate
	input.steer_fall_rate = config.steer_response_rate
	state = KFVehicleState.new()
	
	# 1. Configuração do RigidBody3D (massa, centro de massa e inércia)
	body.mass = config.total_mass
	body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	body.center_of_mass = config.center_of_mass_offset
	# O KinetiForge ja calcula explicitamente arrasto aerodinamico,
	# resistencia ao rolamento e amortecimento angular. O damping global da
	# Godot em modo COMBINE adicionava outro arrasto proporcional a velocidade
	# (massa * default_linear_damp * velocidade), suficiente para cancelar a
	# tracao nas marchas longas.
	body.linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	body.linear_damp = 0.0
	body.angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	body.angular_damp = 0.0
	
	var calculated_inertia: Vector3 = KFMath.calculate_box_inertia(
		config.total_mass,
		config.dimensions,
		config.inertia_multiplier
	)
	body.inertia = calculated_inertia
	
	# 2. Inicialização dos Pneus e Rodas
	var front_tire_cfg: KFTireConfig = config.front_tire_config if config.front_tire_config != null else KFTireConfig.new()
	var rear_tire_cfg: KFTireConfig = config.rear_tire_config if config.rear_tire_config != null else KFTireConfig.new()
	var front_susp_cfg: KFSuspensionConfig = config.front_suspension_config if config.front_suspension_config != null else KFSuspensionConfig.new()
	var rear_susp_cfg: KFSuspensionConfig = config.rear_suspension_config if config.rear_suspension_config != null else KFSuspensionConfig.new()
	
	var wheel_fl := KFWheel.new(0, "FL", true, true, front_tire_cfg, front_susp_cfg)
	var wheel_fr := KFWheel.new(1, "FR", true, false, front_tire_cfg, front_susp_cfg)
	var wheel_rl := KFWheel.new(2, "RL", false, true, rear_tire_cfg, rear_susp_cfg)
	var wheel_rr := KFWheel.new(3, "RR", false, false, rear_tire_cfg, rear_susp_cfg)
	
	# Posições de montagem padrão se não houver nós dedicados
	wheel_fl.local_mount_position = Vector3(-0.865, 0.88, -1.699)
	wheel_fr.local_mount_position = Vector3(0.868, 0.88, -1.699)
	wheel_rl.local_mount_position = Vector3(-0.854, 0.88, 1.715)
	wheel_rr.local_mount_position = Vector3(0.857, 0.88, 1.715)
	
	if mount_nodes.has("FL"): wheel_fl.mount_node = mount_nodes["FL"]
	if mount_nodes.has("FR"): wheel_fr.mount_node = mount_nodes["FR"]
	if mount_nodes.has("RL"): wheel_rl.mount_node = mount_nodes["RL"]
	if mount_nodes.has("RR"): wheel_rr.mount_node = mount_nodes["RR"]
	
	if visual_nodes.has("FL"): wheel_fl.visual_node = visual_nodes["FL"]
	if visual_nodes.has("FR"): wheel_fr.visual_node = visual_nodes["FR"]
	if visual_nodes.has("RL"): wheel_rl.visual_node = visual_nodes["RL"]
	if visual_nodes.has("RR"): wheel_rr.visual_node = visual_nodes["RR"]
	
	if hub_nodes.has("FL"): wheel_fl.hub_node = hub_nodes["FL"]
	if hub_nodes.has("FR"): wheel_fr.hub_node = hub_nodes["FR"]
	if hub_nodes.has("RL"): wheel_rl.hub_node = hub_nodes["RL"]
	if hub_nodes.has("RR"): wheel_rr.hub_node = hub_nodes["RR"]
	
	wheels = [wheel_fl, wheel_fr, wheel_rl, wheel_rr]
	
	var is_rwd: bool = config.layout == KFVehicleConfig.DrivetrainLayout.RWD or config.layout == KFVehicleConfig.DrivetrainLayout.AWD
	var is_fwd: bool = config.layout == KFVehicleConfig.DrivetrainLayout.FWD or config.layout == KFVehicleConfig.DrivetrainLayout.AWD
	wheel_fl.is_driven = is_fwd
	wheel_fr.is_driven = is_fwd
	wheel_rl.is_driven = is_rwd
	wheel_rr.is_driven = is_rwd
	
	# 3. Eixos e Powertrain
	front_axle = KFAxle.new(wheel_fl, wheel_fr, true, config.front_diff_config, front_susp_cfg.anti_roll_rate, front_susp_cfg.anti_roll_damping)
	rear_axle = KFAxle.new(wheel_rl, wheel_rr, false, config.rear_diff_config, rear_susp_cfg.anti_roll_rate, rear_susp_cfg.anti_roll_damping)
	front_axle.is_driven = is_fwd
	rear_axle.is_driven = is_rwd
	
	engine = KFEngine.new(config.engine_config)
	clutch = KFClutch.new(config.gearbox_config)
	gearbox = KFGearbox.new(config.gearbox_config)
	brake = KFBrake.new(config.max_brake_torque, config.front_brake_bias, config.handbrake_torque)
	
	drive_assembly = KFDriveAssembly.new(
		engine,
		clutch,
		gearbox,
		brake,
		front_axle,
		rear_axle,
		config.layout,
		config.awd_front_split,
		config.center_diff_config
	)
	drive_assembly.driveshaft_stiffness = config.driveshaft_stiffness
	drive_assembly.driveshaft_damping = config.driveshaft_damping
	drive_assembly.driveshaft_max_torque = config.driveshaft_max_torque
	gearbox.gear_changed.connect(func(old_gear: int, new_gear: int) -> void:
		if (old_gear == -1) != (new_gear == -1):
			drive_assembly.synchronize_after_direction_change()
	)
	
	# 4. Assistências e Aerodinâmica
	abs_assist = KFAssistABS.new()
	abs_assist.enabled = config.abs_enabled
	abs_assist.target_slip = config.abs_target_slip
	
	tcs_assist = KFAssistTCS.new()
	tcs_assist.enabled = config.tcs_enabled
	tcs_assist.target_slip = config.tcs_target_slip
	
	auto_gearbox = KFAutoGearbox.new(gearbox, engine, config.gearbox_config)
	auto_gearbox.enabled = (config.gearbox_config != null and config.gearbox_config.mode == KFGearboxConfig.TransmissionMode.AUTOMATIC)
	
	aero = KFAeroBody.new(config)
	
	is_initialized = true

## Executado dentro de _integrate_forces(state) a 120 Hz
func physics_step(body_state: PhysicsDirectBodyState3D) -> void:
	if not is_initialized:
		return
	
	var dt: float = body_state.step
	if dt <= 0.0:
		dt = 1.0 / 120.0
	
	# 1. Leitura e suavização de inputs do jogador
	input.poll_inputs()
	input.update_smoothing(dt, state.forward_speed, config.steer_speed_reduction)
	if input.shift_up_requested:
		gearbox.shift_up()
	if input.shift_down_requested:
		gearbox.shift_down()
	
	# Trata reset do veículo
	if input.reset_requested:
		input.reset_requested = false
		body_state.transform.origin.y += 1.5
		body_state.transform.basis = Basis()
		body_state.linear_velocity = Vector3.ZERO
		body_state.angular_velocity = Vector3.ZERO
		for wheel in wheels:
			wheel.state.omega = 0.0
	
	# 2. Cálculo dos ângulos de esterçamento (Ackermann)
	_apply_steering()
	
	# 3. Consulta de contato físico de todas as rodas (Raycast)
	var vehicle_rid: RID = body.get_rid()
	var road_mask: int = config.road_collision_mask
	for wheel in wheels:
		wheel.step_contact(body_state, vehicle_rid, road_mask)
	
	# 4. Resolução de compressão e força de suspensão individual
	for wheel in wheels:
		wheel.step_suspension(body_state, dt)
	
	# 5. Aplicação da Barra Estabilizadora (Anti-Roll Bar) em cada eixo
	front_axle.solve_anti_roll()
	rear_axle.solve_anti_roll()
	
	# 6. Atualização da orientação/base da roda no espaço global
	var vehicle_basis: Basis = body_state.transform.basis
	for wheel in wheels:
		wheel.update_wheel_basis(vehicle_basis, wheel.state.steer_angle)
	
	# 7. Transmissão Automática e Powertrain
	var drive_throttle: float = input.throttle
	var service_brake: float = input.brake
	if auto_gearbox.enabled:
		var resolved_controls: Dictionary = auto_gearbox.resolve_drive_controls(
			input.throttle,
			input.brake,
			state.forward_speed,
			input.raw_throttle,
			input.raw_brake
		)
		drive_throttle = float(resolved_controls.throttle)
		service_brake = float(resolved_controls.brake)
		auto_gearbox.step(dt, drive_throttle, state.forward_speed, service_brake)
		# Automáticos não exigem comando manual de partida após um stall.
		if drive_throttle > 0.05 and not engine.is_running and not engine.starter_active:
			engine.start()
	
	# Calcula o slip do tick atual antes das assistências.
	for wheel in wheels:
		wheel.step_tire_kinematics(body_state, dt)
	state.tcs_active = tcs_assist.solve(wheels, dt)
	var effective_throttle: float = drive_throttle * tcs_assist.torque_multiplier
	
	# Simulação do Drivetrain 1D (Engine, Clutch, Gearbox, Diff)
	drive_assembly.step(dt, effective_throttle, service_brake, input.handbrake, input.clutch)
	
	# 8. ABS modula os torques de freio preparados pelo drivetrain.
	state.abs_active = abs_assist.solve(wheels, dt)
	
	# 9. Resolução de Forças do Pneu (Pacejka + Combined Slip)
	for wheel in wheels:
		wheel.step_tire_forces(dt)
	
	# 10. Aplicação de Forças das Rodas ao Chassi (RigidBody3D)
	for wheel in wheels:
		wheel.apply_forces_to_body(body_state)
	_apply_grounded_roll_damping(body_state)
	
	# 11. Aerodinâmica (Arrasto e Downforce)
	aero.step(body_state, dt)
	
	# 12. Integração da Inércia Rotacional das Rodas
	for wheel in wheels:
		wheel.integrate_rotation(dt)
	
	# 13. Atualização do Estado Global do Veículo e Telemetria
	_update_vehicle_state(body_state, dt)
	input.clear_transient_requests()

func _apply_grounded_roll_damping(body_state: PhysicsDirectBodyState3D) -> void:
	var grounded_count: int = 0
	for wheel in wheels:
		if wheel.state.in_contact and wheel.state.normal_load > 1.0:
			grounded_count += 1
	if grounded_count < 2:
		return
	var forward_axis: Vector3 = -body_state.transform.basis.z.normalized()
	var roll_rate: float = body_state.angular_velocity.dot(forward_axis)
	var damping_torque_magnitude: float = clampf(
		-roll_rate * config.grounded_roll_damping,
		-config.max_roll_damping_torque,
		config.max_roll_damping_torque
	)
	body_state.apply_torque(forward_axis * damping_torque_magnitude)

func _apply_steering() -> void:
	var base_steer_rad: float = deg_to_rad(config.max_steer_angle) * input.steer
	var ackermann: float = config.ackermann_factor
	
	if absf(base_steer_rad) > 0.001:
		var steer_sign: float = signf(base_steer_rad)
		var abs_steer: float = absf(base_steer_rad)
		
		var wheelbase: float = maxf(config.wheelbase, 0.1)
		var track: float = maxf(config.front_track, 0.1)
		var turn_radius: float = wheelbase / maxf(tan(abs_steer), 0.0001)
		var ideal_inner: float = atan(wheelbase / maxf(turn_radius - track * 0.5, 0.05))
		var ideal_outer: float = atan(wheelbase / (turn_radius + track * 0.5))
		var inner_angle: float = lerpf(abs_steer, ideal_inner, ackermann)
		var outer_angle: float = lerpf(abs_steer, ideal_outer, ackermann)
		
		if steer_sign > 0.0: # Esquerda
			wheels[0].state.steer_angle = inner_angle # FL
			wheels[1].state.steer_angle = outer_angle # FR
		else: # Direita
			wheels[0].state.steer_angle = -outer_angle # FL
			wheels[1].state.steer_angle = -inner_angle # FR
	else:
		wheels[0].state.steer_angle = 0.0
		wheels[1].state.steer_angle = 0.0

func _update_vehicle_state(body_state: PhysicsDirectBodyState3D, dt: float) -> void:
	var v: Vector3 = body_state.linear_velocity
	state.speed_ms = v.length()
	state.speed_kmh = state.speed_ms * KFMath.MS_TO_KMH
	
	var forward_vec: Vector3 = -body_state.transform.basis.z.normalized()
	var right_vec: Vector3 = body_state.transform.basis.x.normalized()
	var up_vec: Vector3 = body_state.transform.basis.y.normalized()
	
	state.forward_speed = v.dot(forward_vec)
	state.lateral_speed = v.dot(right_vec)
	state.vertical_speed = v.dot(up_vec)
	
	var accel_vec: Vector3 = (v - previous_linear_velocity) / maxf(dt, 0.0001)
	previous_linear_velocity = v
	state.linear_acceleration = accel_vec
	state.longitudinal_g = accel_vec.dot(forward_vec) / KFMath.GRAVITY_CONSTANT
	state.lateral_g = accel_vec.dot(right_vec) / KFMath.GRAVITY_CONSTANT
	state.vertical_g = accel_vec.dot(up_vec) / KFMath.GRAVITY_CONSTANT
	
	var ang_vel: Vector3 = body_state.angular_velocity
	state.yaw_rate = ang_vel.dot(up_vec)
	state.pitch_rate = ang_vel.dot(right_vec)
	state.roll_rate = ang_vel.dot(forward_vec)
	
	state.engine_running = engine.is_running
	state.engine_rpm = engine.crankshaft.get_rpm()
	state.engine_torque = engine.net_engine_torque
	state.combustion_torque = engine.combustion_torque
	state.friction_torque = engine.friction_torque
	state.turbo_boost_bar = engine.turbo_boost
	
	state.current_gear = gearbox.current_gear
	state.is_shifting = gearbox.is_shifting
	state.clutch_engagement = clutch.engagement
	state.clutch_slip = clutch.delta_omega
	state.update_gear_name()
	
	state.driveshaft_torque = drive_assembly.driveshaft_torque
	state.aero_drag_force = aero.current_drag_force
	state.aero_downforce = aero.current_downforce

## Atualiza a interpolação visual das 4 rodas
func update_visuals(vehicle_transform: Transform3D) -> void:
	for wheel in wheels:
		wheel.update_visual(vehicle_transform)
