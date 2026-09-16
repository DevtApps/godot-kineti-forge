class_name KFMath
extends RefCounted

const RPM_TO_RAD_S: float = TAU / 60.0
const RAD_S_TO_RPM: float = 60.0 / TAU
const KMH_TO_MS: float = 1.0 / 3.6
const MS_TO_KMH: float = 3.6
const GRAVITY_CONSTANT: float = 9.80665

## Cálculo de Slip Ratio Longitudinal κ (kappa)
## κ > 0: Tração / aceleração
## κ < 0: Frenagem
## κ = 0: Rolamento livre
static func calc_slip_ratio(vx: float, omega: float, radius: float, min_speed: float = 0.5) -> float:
	var wheel_speed: float = omega * radius
	var denom: float = maxf(maxf(absf(vx), absf(wheel_speed)), min_speed)
	return (wheel_speed - vx) / denom

## Cálculo de Slip Angle Lateral α (alpha) em radianos
## α é o ângulo entre a direção para onde a roda aponta e o vetor de velocidade do contato
static func calc_slip_angle(vx: float, vy: float, min_speed: float = 0.5) -> float:
	return atan2(vy, maxf(absf(vx), min_speed))

## Magic Formula de Pacejka compacta (B, C, D, E)
## Retorna o valor de força normalizada ou pico de força
static func magic_formula(x: float, b: float, c: float, d: float, e: float) -> float:
	var bx: float = b * x
	var inner_atan: float = atan(bx)
	return d * sin(c * atan(bx - e * (bx - inner_atan)))

## Capacidade de atrito do pneu com sensibilidade não-linear à carga vertical (Load Sensitivity)
## Para exponent < 1.0 (ex: 0.9), dobrar Fz gera menos que o dobro da força máxima de atrito
static func friction_capacity(normal_load: float, reference_load: float, mu0: float, exponent: float) -> float:
	if normal_load <= 0.0:
		return 0.0
	var ratio: float = normal_load / maxf(reference_load, 1.0)
	return mu0 * reference_load * pow(ratio, exponent)

## Fator de escala da Elipse de Atrito / Combined Slip estilo TMeasy
## nx e ny são as forças longitudinal e lateral normalizadas pela capacidade máxima
static func combined_slip_scale(nx: float, ny: float, q: float = 2.0) -> float:
	var r: float = pow(pow(absf(nx), q) + pow(absf(ny), q), 1.0 / q)
	if r <= 1.0:
		return 1.0
	return 1.0 / r

## Momentos de inércia aproximados de uma caixa sólida (Ix, Iy, Iz)
## Ix = 1/12 m (h² + l²) (Roll)
## Iy = 1/12 m (w² + l²) (Yaw)
## Iz = 1/12 m (w² + h²) (Pitch)
static func calculate_box_inertia(mass: float, dimensions: Vector3, multiplier: Vector3 = Vector3.ONE) -> Vector3:
	var w: float = dimensions.x # Largura
	var h: float = dimensions.y # Altura
	var l: float = dimensions.z # Comprimento
	var ix: float = (1.0 / 12.0) * mass * (h * h + l * l) * multiplier.x
	var iy: float = (1.0 / 12.0) * mass * (w * w + l * l) * multiplier.y
	var iz: float = (1.0 / 12.0) * mass * (w * w + h * h) * multiplier.z
	return Vector3(ix, iy, iz)
