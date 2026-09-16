class_name KFRelaxationSolver
extends RefCounted

## Atualiza a dinâmica transitória de deformação da carcaça do pneu (Relaxation Length)
static func update_relaxation(
	tire_config: KFTireConfig,
	dt: float,
	out_state: KFWheelState
) -> void:
	if not out_state.in_contact:
		out_state.relaxed_slip_ratio = 0.0
		out_state.relaxed_slip_angle = 0.0
		return
	
	# A velocidade de transporte do relaxation longitudinal considera também a velocidade da roda (omega * R)
	# para resposta imediata na arrancada quando Vx ~ 0
	var abs_vx: float = absf(out_state.vx)
	var abs_wr: float = absf(out_state.surface_speed)
	var long_transport_speed: float = maxf(maxf(abs_vx, abs_wr), 0.1)
	var lat_transport_speed: float = maxf(abs_vx, 0.1)
	
	# Filtra kappa longitudinal
	out_state.relaxed_slip_ratio = KFFilters.relax_value(
		out_state.relaxed_slip_ratio,
		out_state.slip_ratio,
		tire_config.longitudinal_relaxation,
		long_transport_speed,
		dt,
		0.1
	)
	
	# Filtra alpha lateral
	out_state.relaxed_slip_angle = KFFilters.relax_value(
		out_state.relaxed_slip_angle,
		out_state.slip_angle,
		tire_config.lateral_relaxation,
		lat_transport_speed,
		dt,
		0.1
	)

