class_name KFWheelState
extends RefCounted

## Estado de contato com o solo
var in_contact: bool = false
var contact_point: Vector3 = Vector3.ZERO
var contact_normal: Vector3 = Vector3.UP
var contact_distance: float = 0.0
var collider: Object = null
var surface_friction_multiplier: float = 1.0
var surface_rolling_resistance: float = -1.0

## Suspensão
var suspension_length: float = 0.0
var suspension_extension_velocity: float = 0.0
var suspension_contact_factor: float = 0.0
var compression: float = 0.0
var compression_velocity: float = 0.0
var suspension_force: float = 0.0
var anti_roll_force: float = 0.0
var normal_load: float = 0.0 # Fz final = max(0, Fsusp + Farb)

## Base da roda no espaço global
var wheel_forward: Vector3 = Vector3.FORWARD
var wheel_right: Vector3 = Vector3.RIGHT
var wheel_up: Vector3 = Vector3.UP

## Geometria e orientação
var steer_angle: float = 0.0 # em radianos
var camber_angle: float = 0.0 # em radianos
var toe_angle: float = 0.0 # em radianos
var caster_angle: float = 0.0 # em radianos
var geometry_lateral_offset: float = 0.0
var geometry_longitudinal_offset: float = 0.0
var motion_ratio: float = 1.0
var spin_angle: float = 0.0 # rotação visual cumulativa em radianos

## Dinâmica de contato e velocidades
var point_velocity: Vector3 = Vector3.ZERO
var vx: float = 0.0 # Velocidade longitudinal do ponto de contato
var vy: float = 0.0 # Velocidade lateral do ponto de contato
var surface_speed: float = 0.0 # ω * R

## Dinâmica rotacional da roda
var omega: float = 0.0 # Velocidade angular da roda em rad/s
var rpm: float = 0.0
var drive_torque: float = 0.0
var brake_torque: float = 0.0
var service_brake_torque: float = 0.0
var handbrake_torque: float = 0.0
var rolling_torque: float = 0.0
var reaction_torque: float = 0.0
var net_torque: float = 0.0

## Slips e Relaxamento
var slip_ratio: float = 0.0 # κ instantâneo
var slip_angle: float = 0.0 # α instantâneo
var relaxed_slip_ratio: float = 0.0 # κ filtrado por relaxation length
var relaxed_slip_angle: float = 0.0 # α filtrado por relaxation length

## Forças de atrito do pneu
var constraint_fx: float = 0.0
var constraint_fy: float = 0.0
var effective_sprung_mass: float = 400.0
var effective_inertia: float = 1.35
var friction_capacity: float = 0.0 # Capacidade máxima de atrito (N)
var pure_longitudinal_force: float = 0.0 # Fx0
var pure_lateral_force: float = 0.0 # Fy0
var combined_scale: float = 1.0
var longitudinal_force: float = 0.0 # Fx final aplicado ao chassis
var lateral_force: float = 0.0 # Fy final aplicado ao chassis
var total_force_world: Vector3 = Vector3.ZERO

## Assistências ativas nesta roda
var abs_active: bool = false
var tcs_active: bool = false
