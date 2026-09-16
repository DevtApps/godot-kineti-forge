class_name KFTireConfig
extends Resource

@export_group("Dimensions & Mass")
## Raio do pneu em metros (ex: 0.32m = aro ~17" com perfil esportivo).
@export var radius: float = 0.33
## Largura da banda de rodagem em metros.
@export var width: float = 0.235
## Massa da roda/pneu em kg.
@export var mass: float = 18.0
## Inércia rotacional em kg·m² (I = 1/2 m r² ou ~1.2 para rodas comuns).
@export var rotational_inertia: float = 1.25
## Proteções para rotação livre durante saltos/perda de contato.
@export var max_wheel_rpm: float = 2800.0
@export var airborne_angular_drag: float = 1.2
## Margem usada pelo sphere shape cast da roda.
@export_range(0.0, 0.1) var shape_cast_margin: float = 0.015
## Fração máxima da aderência usada apenas para sincronizar uma roda livre
## com a velocidade do solo. Evita frenagem artificial quando a embreagem
## abre durante uma troca de marcha.
@export_range(0.01, 0.5) var free_rolling_force_ratio: float = 0.08

@export_group("Longitudinal Pacejka (B, C, E)")
## Rigidez longitudinal (Stiffness Bx).
@export var bx: float = 10.0
## Fator de forma longitudinal (Shape Cx).
@export var cx: float = 1.65
## Curvatura longitudinal (Curvature Ex).
@export var ex: float = 0.0

@export_group("Lateral Pacejka (B, C, E)")
## Rigidez lateral (Stiffness By).
@export var by: float = 8.5
## Fator de forma lateral (Shape Cy).
@export var cy: float = 1.35
## Curvatura lateral (Curvature Ey).
@export var ey: float = -1.0

@export_group("Load Sensitivity")
## Carga vertical de referência (Fz0 em Newtons, ~1/4 da massa do carro * g).
@export var reference_load: float = 3800.0
## Coeficiente de atrito base nominal (μ0 no pico).
@export var friction_coefficient: float = 1.05
## Expoente de sensibilidade à carga (0 < p <= 1, tipicamente 0.85 a 0.95).
@export var load_exponent: float = 0.90

@export_group("Relaxation Length")
## Relaxation length longitudinal em metros (distância para a força se desenvolver).
@export var longitudinal_relaxation: float = 0.28
## Relaxation length lateral em metros.
@export var lateral_relaxation: float = 0.35

@export_group("Combined Slip")
## Expoente da elipse de atrito / modelo TMeasy (q = 2.0 para elipse padrão).
@export var combined_slip_power: float = 2.0

@export_group("Rolling Resistance")
## Coeficiente de resistência ao rolamento base (Crr).
@export var rolling_resistance_coeff: float = 0.015
