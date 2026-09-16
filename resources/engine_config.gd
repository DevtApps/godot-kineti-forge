class_name KFEngineConfig
extends Resource

@export_group("RPM Limits")
## Rotação de marcha lenta em RPM.
@export var idle_rpm: float = 850.0
## Rotação mínima de sustentação (abaixo disso o motor morre/stall) em RPM.
@export var stall_rpm: float = 400.0
## Rotação da faixa vermelha em RPM.
@export var redline_rpm: float = 6500.0
## Rotação de corte de injeção / limitador em RPM.
@export var limiter_rpm: float = 7000.0

@export_group("Inertia & Friction")
## Inércia do virabrequim + volante em kg·m² (tipicamente 0.15 a 0.35 para motores 4/6 cilindros).
@export var flywheel_inertia: float = 0.22
## Fricção interna constante (Coulomb) em N·m.
@export var friction_static: float = 12.0
## Fricção linear viscosa em N·m / (rad/s).
@export var friction_linear: float = 0.04
## Fricção quadrática (perdas aerodinâmicas/bombeamento) em N·m / (rad/s)².
@export var friction_quadratic: float = 0.00015
## Perda de bombeamento com borboleta fechada (Engine Braking) em N·m.
@export var engine_braking_torque: float = 35.0

@export_group("Torque Output")
## Torque máximo de pico do motor em N·m.
@export var max_torque: float = 380.0
## Curva normalizada de torque vs RPM (0.0 = idle_rpm, 1.0 = redline_rpm).
## Se não fornecida, uma curva padrão de combustão interna com pico em 4500 RPM é gerada.
@export var torque_curve: Curve

@export_group("Starter & Ignition")
## Torque aplicado pelo motor de arranque em N·m.
@export var starter_torque: float = 90.0
## Rotação máxima em que o arranque se desacopla em RPM.
@export var starter_max_rpm: float = 550.0
## Rotação mínima para ligar por push-start / tranco em RPM.
@export var push_start_min_rpm: float = 450.0

@export_group("Turbo / Forced Induction")
## Se o motor possui turbocompressor.
@export var turbo_enabled: bool = false
## Pressão máxima de boost em Bar.
@export var turbo_max_boost: float = 1.2
## Constante de tempo do turbo (turbo lag) em segundos.
@export var turbo_time_constant: float = 0.45
## Ganho de torque por bar de boost (ex: 0.7 = +70% de torque por bar).
@export var turbo_torque_gain: float = 0.65

@export_group("Anti-Lag & Exhaust")
@export var anti_lag_enabled: bool = false
@export var anti_lag_min_rpm: float = 2500.0
@export_range(0.0, 1.0) var anti_lag_target_boost_ratio: float = 0.65
@export var exhaust_heat_rate: float = 0.55
@export var exhaust_cooling_rate: float = 0.28
@export_range(0.0, 1.0) var backfire_heat_threshold: float = 0.72
@export_range(0.0, 1.0) var backfire_fuel_threshold: float = 0.15

func get_torque_at_rpm(rpm: float) -> float:
	if torque_curve != null:
		var norm_rpm: float = clampf(inverse_lerp(idle_rpm, redline_rpm, rpm), 0.0, 1.0)
		return torque_curve.sample_baked(norm_rpm) * max_torque
	
	# Curva padrão aproximada se não houver Curve definida
	var norm: float = clampf(inverse_lerp(idle_rpm, redline_rpm, rpm), 0.0, 1.0)
	# Parábola com pico aos ~60% da faixa útil (~4200 RPM)
	var factor: float = 0.45 + 0.55 * sin(norm * PI * 0.9)
	return factor * max_torque
