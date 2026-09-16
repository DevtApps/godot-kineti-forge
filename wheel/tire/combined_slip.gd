class_name KFCombinedSlip
extends RefCounted

## Aplica a elipse de atrito / acoplamento de slip combinado estilo TMeasy
static func apply_combined_slip(
	pure_fx: float,
	pure_fy: float,
	capacity: float,
	power_exponent: float,
	out_state: KFWheelState
) -> void:
	if capacity <= 0.001:
		out_state.longitudinal_force = 0.0
		out_state.lateral_force = 0.0
		out_state.combined_scale = 1.0
		return
	
	var nx: float = pure_fx / capacity
	var ny: float = pure_fy / capacity
	
	var scale: float = KFMath.combined_slip_scale(nx, ny, power_exponent)
	out_state.combined_scale = scale
	out_state.longitudinal_force = pure_fx * scale
	out_state.lateral_force = pure_fy * scale

