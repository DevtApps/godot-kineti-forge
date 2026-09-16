class_name KFFilters
extends RefCounted

## Integração de Relaxation Length de primeira ordem para pneu
## τ = Lrelax / max(|Vx|, Vmin)
## alpha = 1 - exp(-dt / τ)
static func relax_value(
	current: float,
	target: float,
	relaxation_length: float,
	speed: float,
	dt: float,
	min_speed: float = 0.5
) -> float:
	var tau: float = relaxation_length / maxf(absf(speed), min_speed)
	var alpha: float = 1.0 - exp(-dt / maxf(tau, 0.0001))
	return lerpf(current, target, alpha)

## Filtro exponencial estável e independente de framerate
static func exp_smooth(current: float, target: float, rate: float, dt: float) -> float:
	return lerpf(current, target, 1.0 - exp(-rate * dt))

## Suavizador com taxas assimétricas de subida (rise) e descida (fall)
static func approach_rate(
	value: float,
	target: float,
	rise_rate: float,
	fall_rate: float,
	dt: float
) -> float:
	var rate: float = rise_rate if target > value else fall_rate
	return move_toward(value, target, rate * dt)
