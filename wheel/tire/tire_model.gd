class_name KFTireModel
extends RefCounted

## Calcula todas as forças de atrito do pneu com Pacejka, sensibilidade à carga e combined slip
## Incorpora o modelo de constraint force e limitação de atrito do KinetiForge C++
static func solve_forces(
	tire_config: KFTireConfig,
	out_state: KFWheelState,
	dt: float = 0.00833
) -> void:
	if not out_state.in_contact or out_state.normal_load <= 0.0:
		out_state.friction_capacity = 0.0
		out_state.pure_longitudinal_force = 0.0
		out_state.pure_lateral_force = 0.0
		out_state.longitudinal_force = 0.0
		out_state.lateral_force = 0.0
		out_state.rolling_torque = 0.0
		out_state.reaction_torque = 0.0
		out_state.total_force_world = Vector3.ZERO
		return
	
	# 1. Capacidade máxima de atrito com sensibilidade à carga vertical
	var capacity: float = KFMath.friction_capacity(
		out_state.normal_load,
		tire_config.reference_load,
		tire_config.friction_coefficient,
		tire_config.load_exponent
	)
	capacity *= maxf(out_state.surface_friction_multiplier, 0.0)
	out_state.friction_capacity = capacity
	
	# 2. Força de vínculo longitudinal (Constraint Force / Stick condition do KinetiForge)
	var r: float = maxf(tire_config.radius, 0.01)
	var r_inv: float = 1.0 / r
	var drive_force: float = out_state.drive_torque * r_inv
	
	# Torque de interação com o solo necessário para manter rotação pura
	var angular_slip: float = out_state.omega - out_state.vx * r_inv
	var eff_inertia: float = maxf(out_state.effective_inertia, 0.05)
	var torque_ground: float = (eff_inertia / maxf(dt, 0.0001)) * angular_slip
	var force_ground: float = torque_ground * r_inv
	
	# Força para desacelerar o movimento da roda sob frenagem
	var force_to_stop: float = -(out_state.vx * (out_state.effective_sprung_mass / maxf(dt, 0.0001)) + drive_force + force_ground)
	var max_brake_force: float = out_state.brake_torque * r_inv
	var signed_brake_force: float = clampf(force_to_stop, -max_brake_force, max_brake_force)
	
	var constraint_long: float = drive_force + signed_brake_force + force_ground
	var is_free_rolling: bool = (
		absf(out_state.drive_torque) < 0.5
		and out_state.brake_torque < 0.5
	)
	var free_roll_limit: float = capacity
	# Uma roda sem torque motriz e sem freio precisa somente de força suficiente
	# para corrigir sua inercia rotacional. O constraint instantaneo pode ficar
	# muito alto apos a remocao brusca do torque (por exemplo, embreagem aberta
	# numa troca) e transformar as quatro rodas em freios no limite de aderencia.
	if is_free_rolling:
		free_roll_limit = capacity * clampf(
			tire_config.free_rolling_force_ratio,
			0.01,
			0.5
		)
		constraint_long = clampf(constraint_long, -free_roll_limit, free_roll_limit)
	out_state.constraint_fx = constraint_long
	
	# 3. Força de vínculo lateral (Constraint Lat Force)
	var constraint_lat: float = -(out_state.vy * (out_state.effective_sprung_mass / maxf(dt, 0.0001)))
	out_state.constraint_fy = constraint_lat
	
	# 4. Avaliação das curvas de Pacejka Magic Formula
	var norm_fx: float = KFMath.magic_formula(
		out_state.relaxed_slip_ratio,
		tire_config.bx,
		tire_config.cx,
		1.0,
		tire_config.ex
	)
	var pacejka_fx: float = norm_fx * capacity
	
	var norm_fy: float = KFMath.magic_formula(
		out_state.relaxed_slip_angle,
		tire_config.by,
		tire_config.cy,
		1.0,
		tire_config.ey
	)
	var pacejka_fy: float = -norm_fy * capacity
	
	# 5. Acoplamento de forças:
	# Em regime de rolamento aderente, a força acompanha a força de vínculo física (Constraint Force).
	# Quando o escorregamento cresce além da aderência, a curva de Pacejka limita a força e dita a transição para atrito cinético.
	var slip_long_abs: float = absf(out_state.relaxed_slip_ratio)
	var raw_fx: float = 0.0
	if slip_long_abs < 0.12:
		# Regime de alta aderência: força de vínculo limitada pela capacidade de atrito
		raw_fx = clampf(constraint_long, -capacity, capacity)
	else:
		# Regime de escorregamento / burnout / travamento de roda: transição para Pacejka puro
		var blend_t: float = clampf((slip_long_abs - 0.12) / 0.08, 0.0, 1.0)
		var adherent_fx: float = clampf(constraint_long, -capacity, capacity)
		raw_fx = lerpf(adherent_fx, pacejka_fx, blend_t)
	if is_free_rolling:
		raw_fx = clampf(raw_fx, -free_roll_limit, free_roll_limit)
	
	var slip_lat_abs: float = absf(out_state.relaxed_slip_angle)
	var raw_fy: float = 0.0
	if slip_lat_abs < 0.08:
		# Regime lateral aderente: segue a força de vínculo centrípeta necessária
		raw_fy = clampf(constraint_lat, -capacity, capacity)
	else:
		# Regime de sobreesterço / subesterço / drift: transição para Pacejka lateral
		var blend_lat: float = clampf((slip_lat_abs - 0.08) / 0.06, 0.0, 1.0)
		var adherent_fy: float = clampf(constraint_lat, -capacity, capacity)
		raw_fy = lerpf(adherent_fy, pacejka_fy, blend_lat)
	
	out_state.pure_longitudinal_force = raw_fx
	out_state.pure_lateral_force = raw_fy
	
	# 6. Acoplamento de Combined Slip (Elipse de atrito)
	KFCombinedSlip.apply_combined_slip(
		raw_fx,
		raw_fy,
		capacity,
		tire_config.combined_slip_power,
		out_state
	)
	
	# 7. Torque de resistência ao rolamento (Rolling Resistance)
	var rolling_coeff: float = out_state.surface_rolling_resistance if out_state.surface_rolling_resistance >= 0.0 else tire_config.rolling_resistance_coeff
	var rolling_mag: float = rolling_coeff * out_state.normal_load * r
	out_state.rolling_torque = -signf(out_state.omega) * rolling_mag if absf(out_state.omega) > 0.05 else 0.0
	
	# 8. Torque de reação do pneu sobre o eixo da roda
	out_state.reaction_torque = -out_state.longitudinal_force * r
	
	# 9. Vetor de força total em coordenadas de mundo (para telemetria/depuração)
	out_state.total_force_world = (
		out_state.wheel_forward * out_state.longitudinal_force +
		out_state.wheel_right * out_state.lateral_force +
		out_state.contact_normal * out_state.normal_load
	)
