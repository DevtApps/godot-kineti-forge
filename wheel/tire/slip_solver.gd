class_name KFSlipSolver
extends RefCounted

## Calcula as velocidades no ponto de contato e os slips instantâneos
static func solve_slip(
	point_velocity: Vector3,
	wheel_forward: Vector3,
	wheel_right: Vector3,
	wheel_omega: float,
	tire_radius: float,
	out_state: KFWheelState
) -> void:
	# Projeta a velocidade do ponto nos eixos do pneu
	var vx: float = point_velocity.dot(wheel_forward)
	var vy: float = point_velocity.dot(wheel_right)
	
	out_state.point_velocity = point_velocity
	out_state.vx = vx
	out_state.vy = vy
	out_state.surface_speed = wheel_omega * tire_radius
	
	# Slip ratio longitudinal κ = (ωR - Vx) / max(|Vx|, |ωR|, Vmin)
	out_state.slip_ratio = KFMath.calc_slip_ratio(vx, wheel_omega, tire_radius)
	
	# Slip angle lateral α = atan2(Vy, max(|Vx|, Vmin))
	out_state.slip_angle = KFMath.calc_slip_angle(vx, vy)

