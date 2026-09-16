class_name KFVehicleState
extends RefCounted

## Velocidades do chassis
var speed_ms: float = 0.0
var speed_kmh: float = 0.0
var forward_speed: float = 0.0
var lateral_speed: float = 0.0
var vertical_speed: float = 0.0

## Acelerações e forças G
var linear_acceleration: Vector3 = Vector3.ZERO
var longitudinal_g: float = 0.0
var lateral_g: float = 0.0
var vertical_g: float = 0.0

## Rotações angulares (rad/s)
var yaw_rate: float = 0.0
var pitch_rate: float = 0.0
var roll_rate: float = 0.0

## Powertrain
var engine_running: bool = false
var engine_rpm: float = 0.0
var engine_torque: float = 0.0
var combustion_torque: float = 0.0
var friction_torque: float = 0.0
var turbo_boost_bar: float = 0.0

## Transmissão e Embreagem
var current_gear: int = 1 # -1 = Ré, 0 = Neutro, 1..N = Marchas à frente
var gear_name: String = "1"
var is_shifting: bool = false
var clutch_engagement: float = 1.0 # 0.0 = desengatada, 1.0 = acoplada
var clutch_slip: float = 0.0

## Diferencial / Eixos
var driveshaft_rpm: float = 0.0
var driveshaft_torque: float = 0.0

## Forças globais aplicadas
var total_wheel_force: Vector3 = Vector3.ZERO
var aero_drag_force: Vector3 = Vector3.ZERO
var aero_downforce: Vector3 = Vector3.ZERO

## Assistências globais
var abs_active: bool = false
var tcs_active: bool = false

func update_gear_name() -> void:
	if current_gear == -1:
		gear_name = "R"
	elif current_gear == 0:
		gear_name = "N"
	else:
		gear_name = str(current_gear)

