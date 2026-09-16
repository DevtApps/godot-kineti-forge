class_name KFVehicleConfig
extends Resource

enum DrivetrainLayout {
	RWD, # Tração Traseira
	FWD, # Tração Dianteira
	AWD  # Tração Integral / 4x4
}

@export_group("Chassis & Mass")
## Massa total do veículo em kg (incluindo ocupantes/líquidos).
@export var total_mass: float = 1450.0
## Deslocamento do centro de massa em relação à origem do modelo (metros).
## Recomenda-se centro de massa rebaixado (ex: Y = -0.25 a -0.35) para estabilidade realista.
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.25, 0.05)
## Dimensões estimadas para cálculo de inércia da caixa (Largura X, Altura Y, Comprimento Z).
@export var dimensions: Vector3 = Vector3(1.85, 1.45, 4.40)
## Multiplicador de inércia para afinação de roll, pitch e yaw.
@export var inertia_multiplier: Vector3 = Vector3(1.0, 1.0, 1.0)
## Amortecimento agregado de roll produzido pelos amortecedores do chassis.
## Só é aplicado quando pelo menos duas rodas têm contato.
@export var grounded_roll_damping: float = 5200.0
@export var max_roll_damping_torque: float = 14000.0

@export_group("Drivetrain Layout")
## Disposição da tração.
@export var layout: DrivetrainLayout = DrivetrainLayout.RWD
## Distribuição de torque dianteira em caso de AWD (ex: 0.4 = 40% frente, 60% traseira).
@export var awd_front_split: float = 0.40
@export var driveshaft_stiffness: float = 1800.0
@export var driveshaft_damping: float = 45.0
@export var driveshaft_max_torque: float = 6000.0

@export_group("Steering & Geometry")
## Ângulo máximo de esterçamento das rodas dianteiras em graus.
@export var max_steer_angle: float = 34.0
## Fator de geometria Ackermann (0.0 = rodas paralelas, 1.0 = Ackermann 100% ideal).
@export var ackermann_factor: float = 0.85
@export var wheelbase: float = 3.414
@export var front_track: float = 1.733
## Curva de redução de esterçamento vs velocidade (0 = parado, 1 = velocidade máxima).
@export var steer_speed_reduction: Curve
## Taxa de resposta do volante em rad/s ou velocidade de transição.
@export var steer_response_rate: float = 12.0

@export_group("Brakes & Handbrake")
## Torque total máximo de frenagem em N·m.
@export var max_brake_torque: float = 3400.0
## Distribuição de freio dianteiro (Brake Bias, ex: 0.65 = 65% na frente).
@export var front_brake_bias: float = 0.65
## Torque do freio de mão nas rodas traseiras em N·m.
@export var handbrake_torque: float = 2400.0

@export_group("Aerodynamics")
## Coeficiente de arrasto aerodinâmico (Cd).
@export var drag_coefficient: float = 0.32
## Área frontal projetada em m².
@export var frontal_area: float = 2.15
## Coeficiente de downforce dianteiro (Cl frontal).
@export var front_downforce_coeff: float = 0.15
## Coeficiente de downforce traseiro (Cl traseiro).
@export var rear_downforce_coeff: float = 0.28
@export var front_aero_position: Vector3 = Vector3(0.0, 0.2, -1.5)
@export var rear_aero_position: Vector3 = Vector3(0.0, 0.3, 1.5)
@export var aero_linear_damping: Vector3 = Vector3(25.0, 8.0, 3.0)
@export var aero_angular_damping: Vector3 = Vector3(120.0, 80.0, 100.0)
## Densidade do ar em kg/m³.
@export var air_density: float = 1.225

@export_group("Assistances")
## Sistema de freio ABS ativado.
@export var abs_enabled: bool = true
## Slip alvo do ABS (ex: -0.15 = 15% de derrapagem permitida na frenagem).
@export var abs_target_slip: float = -0.16
## Sistema de controle de tração (TCS) ativado.
@export var tcs_enabled: bool = true
## Slip alvo do TCS (ex: 0.18 = 18% de slip máximo antes de atenuar torque).
@export var tcs_target_slip: float = 0.20

@export_group("Sub-Configurations")
@export var engine_config: KFEngineConfig
@export var gearbox_config: KFGearboxConfig
@export var front_diff_config: KFDifferentialConfig
@export var rear_diff_config: KFDifferentialConfig
@export var center_diff_config: KFDifferentialConfig
@export var front_suspension_config: KFSuspensionConfig
@export var rear_suspension_config: KFSuspensionConfig
@export var front_tire_config: KFTireConfig
@export var rear_tire_config: KFTireConfig

@export_group("Physics Raycast Mask")
## Camada de colisão do chão/pistas para os raycasts das rodas.
@export_flags_3d_physics var road_collision_mask: int = 1
