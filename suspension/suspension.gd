class_name KFSuspension
extends RefCounted

var config: KFSuspensionConfig
var previous_compression: float = 0.0
var current_length: float = 0.0

func _init(p_config: KFSuspensionConfig = null) -> void:
	config = p_config if p_config != null else KFSuspensionConfig.new()
	current_length = config.rest_length

## Resolve as forças de mola, amortecedor assimétrico e batente
func solve(
	mount_pos: Vector3,
	tire_radius: float,
	out_state: KFWheelState,
	dt: float
) -> float:
	if not out_state.in_contact:
		var previous_length: float = current_length
		var max_droop_length: float = config.rest_length + config.droop_travel
		current_length = move_toward(
			current_length,
			max_droop_length,
			maxf(config.droop_extension_rate, 0.0) * dt
		)
		out_state.suspension_length = current_length
		out_state.suspension_extension_velocity = (
			current_length - previous_length
		) / maxf(dt, 0.0001)
		out_state.compression = 0.0
		out_state.compression_velocity = 0.0
		out_state.suspension_force = 0.0
		out_state.suspension_contact_factor = 0.0
		out_state.normal_load = 0.0
		out_state.anti_roll_force = 0.0
		out_state.motion_ratio = 1.0
		out_state.camber_angle = deg_to_rad(config.static_camber)
		out_state.toe_angle = deg_to_rad(config.static_toe)
		out_state.caster_angle = deg_to_rad(config.caster_angle)
		out_state.geometry_lateral_offset = 0.0
		out_state.geometry_longitudinal_offset = 0.0
		if config.geometry_lookup != null and config.geometry_type != KFSuspensionConfig.GeometryType.STRAIGHT:
			var droop_geometry: Dictionary = config.geometry_lookup.sample_geometry(0.0)
			out_state.camber_angle += deg_to_rad(float(droop_geometry.camber))
			out_state.toe_angle += deg_to_rad(float(droop_geometry.toe))
			out_state.caster_angle += deg_to_rad(float(droop_geometry.caster))
			out_state.geometry_lateral_offset = float(droop_geometry.lateral_offset)
			out_state.geometry_longitudinal_offset = float(droop_geometry.longitudinal_offset)
		previous_compression = 0.0
		return 0.0
	
	# Distância do mount até o centro do pneu quando em contato com o solo
	var min_length: float = maxf(config.rest_length - config.travel, 0.0)
	var max_length: float = config.rest_length + config.droop_travel
	var measured_length: float = out_state.contact_distance - tire_radius
	current_length = clampf(measured_length, min_length, max_length)
	out_state.suspension_extension_velocity = (
		current_length - out_state.suspension_length
	) / maxf(dt, 0.0001)
	out_state.suspension_length = current_length
	
	# Compressão: no repouso (current_len = rest_length), compressão = 0
	# Quando o chassi desce (current_len < rest_length), compressão > 0
	var raw_compression: float = config.rest_length - current_length
	var clamped_compression: float = clampf(raw_compression, 0.0, config.travel)
	out_state.compression = clamped_compression
	var compression_ratio: float = clamped_compression / maxf(config.travel, 0.001)
	out_state.motion_ratio = 1.0
	out_state.geometry_lateral_offset = 0.0
	out_state.geometry_longitudinal_offset = 0.0
	out_state.camber_angle = deg_to_rad(config.static_camber)
	out_state.toe_angle = deg_to_rad(config.static_toe)
	out_state.caster_angle = deg_to_rad(config.caster_angle)
	if config.geometry_lookup != null and config.geometry_type != KFSuspensionConfig.GeometryType.STRAIGHT:
		var geometry: Dictionary = config.geometry_lookup.sample_geometry(compression_ratio)
		out_state.camber_angle += deg_to_rad(float(geometry.camber))
		out_state.toe_angle += deg_to_rad(float(geometry.toe))
		out_state.caster_angle += deg_to_rad(float(geometry.caster))
		out_state.geometry_lateral_offset = float(geometry.lateral_offset)
		out_state.geometry_longitudinal_offset = float(geometry.longitudinal_offset)
		out_state.motion_ratio = float(geometry.motion_ratio)
	
	# Velocidade de compressão (m/s)
	var comp_vel: float = clampf(
		(clamped_compression - previous_compression) / maxf(dt, 0.0001),
		-config.max_compression_velocity,
		config.max_compression_velocity
	)
	out_state.compression_velocity = comp_vel
	previous_compression = clamped_compression
	
	# 1. Força de mola linear com pré-carga (KinetiForge)
	var spring_preload: float = config.spring_preload if clamped_compression > 0.001 else 0.0
	var effective_compression: float = clamped_compression * out_state.motion_ratio
	var spring_force: float = (config.spring_rate * effective_compression + spring_preload) * out_state.motion_ratio
	
	# 2. Amortecimento assimétrico (Bump vs Rebound)
	var damping_coeff: float = config.bump_damping if comp_vel > 0.0 else config.rebound_damping
	var damper_force: float = damping_coeff * comp_vel * out_state.motion_ratio * out_state.motion_ratio
	
	# 3. Força progressiva do batente (Bump Stop)
	var bump_stop_force: float = 0.0
	var bump_start: float = config.travel * config.bump_stop_threshold
	if clamped_compression > bump_start:
		var bump_zone_length: float = maxf(config.travel - bump_start, 0.001)
		var bump_compression: float = clamped_compression - bump_start
		var u: float = clampf(bump_compression / bump_zone_length, 0.0, 1.0)
		# bump_stop_rate e uma rigidez em N/m. A implementacao anterior
		# usava a rigidez diretamente como forca e produzia dezenas de kN
		# extras por roda no fim do curso. A forca maxima correta para esta
		# curva e rate * comprimento da zona do batente.
		bump_stop_force = config.bump_stop_rate * bump_zone_length * pow(
			u,
			maxf(config.bump_stop_exponent, 1.0)
		)
	
	# Força total resultante da suspensão
	var total_force: float = clampf(
		(spring_force + damper_force + bump_stop_force) * out_state.suspension_contact_factor,
		0.0,
		config.max_suspension_force
	)
	out_state.suspension_force = total_force
	return total_force
