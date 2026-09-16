class_name KFDriveAssembly
extends RefCounted

var engine: KFEngine
var clutch: KFClutch
var gearbox: KFGearbox
var brake: KFBrake

var front_axle: KFAxle
var rear_axle: KFAxle

var layout: KFVehicleConfig.DrivetrainLayout = KFVehicleConfig.DrivetrainLayout.RWD
var awd_front_split: float = 0.40
var substeps: int = 4
var center_differential: KFDifferential
var driveshaft_stiffness: float = 1800.0
var driveshaft_damping: float = 45.0
var driveshaft_max_torque: float = 6000.0
var driveshaft_twist: float = 0.0
var coast_disconnected_in_air: bool = false
var propulsion_requested: bool = false

var transmitted_clutch_torque: float = 0.0
var driveshaft_torque: float = 0.0

func _init(
	p_engine: KFEngine,
	p_clutch: KFClutch,
	p_gearbox: KFGearbox,
	p_brake: KFBrake,
	p_front_axle: KFAxle,
	p_rear_axle: KFAxle,
	p_layout: KFVehicleConfig.DrivetrainLayout = KFVehicleConfig.DrivetrainLayout.RWD,
	p_awd_split: float = 0.40,
	p_center_diff_config: KFDifferentialConfig = null
) -> void:
	engine = p_engine
	clutch = p_clutch
	gearbox = p_gearbox
	brake = p_brake
	front_axle = p_front_axle
	rear_axle = p_rear_axle
	layout = p_layout
	awd_front_split = p_awd_split
	if p_center_diff_config != null:
		center_differential = KFDifferential.new(p_center_diff_config)

func step(dt: float, throttle: float, brake_input: float, handbrake_input: float, clutch_pedal: float) -> void:
	var sub_dt: float = dt / float(substeps)
	var effective_throttle: float = throttle
	var effective_brake: float = brake_input
	# O freio de mão também corta progressivamente o torque motriz; sem isso,
	# a multiplicação da primeira/ré pode superar o freio traseiro.
	effective_throttle *= 1.0 - clampf(handbrake_input, 0.0, 1.0)
	propulsion_requested = (
		effective_throttle > 0.01
		and effective_brake <= 0.01
		and handbrake_input <= 0.01
	)
	var driven_contact: bool = false
	match layout:
		KFVehicleConfig.DrivetrainLayout.RWD:
			driven_contact = rear_axle.left_wheel.state.in_contact or rear_axle.right_wheel.state.in_contact
		KFVehicleConfig.DrivetrainLayout.FWD:
			driven_contact = front_axle.left_wheel.state.in_contact or front_axle.right_wheel.state.in_contact
		KFVehicleConfig.DrivetrainLayout.AWD:
			driven_contact = (
				front_axle.left_wheel.state.in_contact
				or front_axle.right_wheel.state.in_contact
				or rear_axle.left_wheel.state.in_contact
				or rear_axle.right_wheel.state.in_contact
			)
	coast_disconnected_in_air = not driven_contact and effective_throttle <= 0.01
	if coast_disconnected_in_air:
		driveshaft_twist = 0.0
		driveshaft_torque = 0.0
	
	engine.throttle_input = effective_throttle
	var automated_shift_clutch: bool = gearbox.is_shifting and gearbox.config.mode != KFGearboxConfig.TransmissionMode.MANUAL
	clutch.pedal = 1.0 if automated_shift_clutch else clutch_pedal
	
	# Distribuição de freios
	brake.apply_brakes(effective_brake, handbrake_input, front_axle, rear_axle)
	
	for i in range(substeps):
		_substep(sub_dt, effective_throttle)

func synchronize_after_direction_change() -> void:
	driveshaft_twist = 0.0
	driveshaft_torque = 0.0
	var wheel_omega: float = 0.0
	var final_drive: float = 1.0
	if gearbox.current_gear == -1 or layout == KFVehicleConfig.DrivetrainLayout.RWD:
		wheel_omega = (rear_axle.left_wheel.state.omega + rear_axle.right_wheel.state.omega) * 0.5
		final_drive = rear_axle.differential.config.final_drive
	else:
		wheel_omega = (front_axle.left_wheel.state.omega + front_axle.right_wheel.state.omega) * 0.5
		final_drive = front_axle.differential.config.final_drive
	gearbox.output_shaft.omega = wheel_omega * final_drive
	gearbox.input_shaft.omega = gearbox.output_shaft.omega * gearbox.get_current_ratio()

func _substep(dt: float, throttle: float) -> void:
	gearbox.step(dt)
	var gear_ratio: float = gearbox.get_current_ratio()
	
	# 1. Calcula a velocidade angular média das rodas motrizes
	var driven_wheels_omega: float = 0.0
	match layout:
		KFVehicleConfig.DrivetrainLayout.RWD:
			driven_wheels_omega = (rear_axle.left_wheel.state.omega + rear_axle.right_wheel.state.omega) * 0.5
		KFVehicleConfig.DrivetrainLayout.FWD:
			driven_wheels_omega = (front_axle.left_wheel.state.omega + front_axle.right_wheel.state.omega) * 0.5
		KFVehicleConfig.DrivetrainLayout.AWD:
			var front_w: float = (front_axle.left_wheel.state.omega + front_axle.right_wheel.state.omega) * 0.5
			var rear_w: float = (rear_axle.left_wheel.state.omega + rear_axle.right_wheel.state.omega) * 0.5
			driven_wheels_omega = front_w * awd_front_split + rear_w * (1.0 - awd_front_split)
	
	var final_drive: float = 3.73
	if layout == KFVehicleConfig.DrivetrainLayout.FWD:
		final_drive = front_axle.differential.config.final_drive
	elif rear_axle != null:
		final_drive = rear_axle.differential.config.final_drive
	if absf(gear_ratio) > 0.001:
		gearbox.input_shaft.omega = gearbox.output_shaft.omega * gear_ratio
	elif not gearbox.is_shifting:
		# Em neutro real o eixo de entrada fica livre. Durante uma troca,
		# entretanto, preservar sua velocidade evita zerar artificialmente o
		# RPM cinemático antes de a próxima relação ser engatada.
		gearbox.input_shaft.omega = 0.0
	
	# Inércia das rodas refletida na entrada do câmbio
	var wheel_i: float = rear_axle.left_wheel.tire_config.rotational_inertia if rear_axle != null else 1.35
	var gear_total_ratio: float = gear_ratio * final_drive
	var gear_sq_inv: float = 1.0 / maxf(gear_total_ratio * gear_total_ratio, 0.01)
	var reflected_gearbox_inertia: float = (wheel_i * 2.0) * gear_sq_inv
	var engine_i: float = engine.config.flywheel_inertia if engine.config != null else 0.22
	
	# 2. Resolução da embreagem com controle de arrancada automático
	var is_auto: bool = (gearbox.config != null and gearbox.config.mode == KFGearboxConfig.TransmissionMode.AUTOMATIC)
	var stall_protection: bool = is_auto and (
		not engine.is_running
		or engine.starter_active
		or absf(engine.crankshaft.get_rpm()) < maxf(
			engine.config.stall_rpm * 1.25,
			engine.config.idle_rpm * 0.8
		)
	)
	# A troca automática precisa realmente abrir a embreagem. Alterar somente
	# clutch.pedal não surtia efeito porque KFClutch usa sua estratégia
	# automática e ignora o pedal nessa modalidade.
	stall_protection = stall_protection or coast_disconnected_in_air or gearbox.is_shifting
	transmitted_clutch_torque = clutch.step(
		engine.crankshaft.omega,
		gearbox.input_shaft.omega,
		dt,
		throttle,
		is_auto,
		engine_i,
		reflected_gearbox_inertia,
		stall_protection
	)
	
	# O motor sente a reação da embreagem no sentido oposto
	engine.step(dt, -transmitted_clutch_torque)
	
	# 3. Torque na saída do câmbio e acoplamento torsional do cardã.
	var gearbox_output_torque: float = 0.0
	if coast_disconnected_in_air:
		driveshaft_torque = 0.0
		driveshaft_twist = 0.0
		gearbox.output_shaft.omega = driven_wheels_omega * final_drive
	elif absf(gear_ratio) > 0.001 and not gearbox.is_shifting:
		gearbox_output_torque = transmitted_clutch_torque * gear_ratio * gearbox.config.efficiency
		var carrier_omega: float = driven_wheels_omega * final_drive
		var relative_speed: float = gearbox.output_shaft.omega - carrier_omega
		driveshaft_twist = clampf(driveshaft_twist + relative_speed * dt, -1.0, 1.0)
		driveshaft_torque = clampf(
			driveshaft_stiffness * driveshaft_twist + driveshaft_damping * relative_speed,
			-driveshaft_max_torque,
			driveshaft_max_torque
		)
		# Sem comando do motorista, o cardã pode produzir freio-motor, mas
		# nunca continuar impulsionando as rodas com energia torsional antiga.
		if not propulsion_requested:
			driveshaft_torque = remove_uncommanded_propulsion(
				driveshaft_torque,
				driven_wheels_omega
			)
			driveshaft_twist = move_toward(driveshaft_twist, 0.0, dt * 8.0)
	else:
		driveshaft_torque = 0.0
		driveshaft_twist = move_toward(driveshaft_twist, 0.0, dt * 4.0)
	gearbox.output_shaft.torque = 0.0 if coast_disconnected_in_air else gearbox_output_torque - driveshaft_torque
	gearbox.output_shaft.integrate(dt)
	
	# 4. Inércia do motor refletida para cada roda motriz
	var reflected_i_to_each_wheel: float = (engine_i * (gear_total_ratio * gear_total_ratio) * 0.5) * clutch.engagement
	
	# 5. Distribuição do torque para os eixos
	match layout:
		KFVehicleConfig.DrivetrainLayout.RWD:
			front_axle.left_wheel.state.effective_inertia = front_axle.left_wheel.tire_config.rotational_inertia
			front_axle.right_wheel.state.effective_inertia = front_axle.right_wheel.tire_config.rotational_inertia
			rear_axle.left_wheel.state.effective_inertia = rear_axle.left_wheel.tire_config.rotational_inertia + reflected_i_to_each_wheel
			rear_axle.right_wheel.state.effective_inertia = rear_axle.right_wheel.tire_config.rotational_inertia + reflected_i_to_each_wheel
			front_axle.distribute_torque(0.0, dt)
			rear_axle.distribute_torque(driveshaft_torque, dt)
		KFVehicleConfig.DrivetrainLayout.FWD:
			front_axle.left_wheel.state.effective_inertia = front_axle.left_wheel.tire_config.rotational_inertia + reflected_i_to_each_wheel
			front_axle.right_wheel.state.effective_inertia = front_axle.right_wheel.tire_config.rotational_inertia + reflected_i_to_each_wheel
			rear_axle.left_wheel.state.effective_inertia = rear_axle.left_wheel.tire_config.rotational_inertia
			rear_axle.right_wheel.state.effective_inertia = rear_axle.right_wheel.tire_config.rotational_inertia
			front_axle.distribute_torque(driveshaft_torque, dt)
			rear_axle.distribute_torque(0.0, dt)
		KFVehicleConfig.DrivetrainLayout.AWD:
			front_axle.left_wheel.state.effective_inertia = front_axle.left_wheel.tire_config.rotational_inertia + reflected_i_to_each_wheel * awd_front_split
			front_axle.right_wheel.state.effective_inertia = front_axle.right_wheel.tire_config.rotational_inertia + reflected_i_to_each_wheel * awd_front_split
			rear_axle.left_wheel.state.effective_inertia = rear_axle.left_wheel.tire_config.rotational_inertia + reflected_i_to_each_wheel * (1.0 - awd_front_split)
			rear_axle.right_wheel.state.effective_inertia = rear_axle.right_wheel.tire_config.rotational_inertia + reflected_i_to_each_wheel * (1.0 - awd_front_split)
			var front_torque: float = driveshaft_torque * awd_front_split
			var rear_torque: float = driveshaft_torque * (1.0 - awd_front_split)
			if center_differential != null:
				var front_omega: float = (front_axle.left_wheel.state.omega + front_axle.right_wheel.state.omega) * 0.5
				var rear_omega: float = (rear_axle.left_wheel.state.omega + rear_axle.right_wheel.state.omega) * 0.5
				center_differential.solve(driveshaft_torque, front_omega, rear_omega, dt)
				front_torque = center_differential.left_torque
				rear_torque = center_differential.right_torque
			front_axle.distribute_torque(front_torque, dt)
			rear_axle.distribute_torque(rear_torque, dt)

static func remove_uncommanded_propulsion(torque: float, wheel_omega: float) -> float:
	# Potência positiva (T * omega > 0) acelera no sentido atual, inclusive
	# quando ambos são negativos na marcha à ré.
	if torque * wheel_omega > 0.0:
		return 0.0
	return torque
