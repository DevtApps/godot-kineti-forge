class_name KFAntiRollBar
extends RefCounted

## Aplica a transferência de carga lateral entre as rodas esquerda e direita do mesmo eixo
## A barra resiste à diferença de curso sem criar carga vertical líquida no eixo.
static func apply_anti_roll(
	left_state: KFWheelState,
	right_state: KFWheelState,
	anti_roll_rate: float,
	anti_roll_damping: float = 0.0
) -> void:
	if not left_state.in_contact or not right_state.in_contact:
		left_state.anti_roll_force = 0.0
		right_state.anti_roll_force = 0.0
		left_state.normal_load = left_state.suspension_force if left_state.in_contact else 0.0
		right_state.normal_load = right_state.suspension_force if right_state.in_contact else 0.0
		return
	
	# Diferença de compressão: positiva quando a roda esquerda está mais comprimida
	var delta_compression: float = left_state.compression - right_state.compression
	var delta_velocity: float = left_state.compression_velocity - right_state.compression_velocity
	var farb: float = anti_roll_rate * delta_compression + anti_roll_damping * delta_velocity
	# Não permitir que a barra crie carga negativa em um dos pneus. Dentro
	# destes limites, a soma das cargas do eixo permanece exatamente constante.
	farb = clampf(farb, -left_state.suspension_force, right_state.suspension_force)
	
	# A roda mais comprimida recebe reação adicional no chassis; a roda
	# estendida recebe a reação oposta. Isso combate, em vez de amplificar, roll.
	left_state.anti_roll_force = farb
	right_state.anti_roll_force = -farb
	
	left_state.normal_load = maxf(0.0, left_state.suspension_force + farb) if left_state.in_contact else 0.0
	right_state.normal_load = maxf(0.0, right_state.suspension_force - farb) if right_state.in_contact else 0.0
