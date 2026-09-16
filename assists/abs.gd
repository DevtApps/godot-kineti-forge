class_name KFAssistABS
extends RefCounted

var enabled: bool = true
var target_slip: float = -0.16 # Slip ótimo de frenagem
var reapply_margin: float = 0.04
var release_rate: float = 14.0
var apply_rate: float = 10.0

var wheel_multipliers: Array[float] = [1.0, 1.0, 1.0, 1.0]

func solve(wheels: Array[KFWheel], dt: float) -> bool:
	if not enabled:
		for i in range(wheels.size()):
			wheels[i].state.abs_active = false
		return false
	
	var any_active: bool = false
	
	for i in range(wheels.size()):
		var wheel: KFWheel = wheels[i]
		if not wheel.state.in_contact or wheel.state.service_brake_torque <= 10.0:
			wheel_multipliers[i] = 1.0
			wheel.state.abs_active = false
			wheel.state.brake_torque = wheel.state.service_brake_torque + wheel.state.handbrake_torque
			continue
		
		var slip: float = wheel.state.relaxed_slip_ratio
		
		# Se a roda está travando (slip mais negativo que o alvo)
		if slip < target_slip:
			wheel_multipliers[i] = maxf(0.0, wheel_multipliers[i] - release_rate * dt)
			wheel.state.abs_active = true
			any_active = true
		elif slip > target_slip + reapply_margin:
			wheel_multipliers[i] = minf(1.0, wheel_multipliers[i] + apply_rate * dt)
			wheel.state.abs_active = false
		else:
			wheel.state.abs_active = wheel_multipliers[i] < 0.999
			any_active = any_active or wheel.state.abs_active
		
		# Modula o torque de frenagem
		wheel.state.brake_torque = (
			wheel.state.service_brake_torque * wheel_multipliers[i]
			+ wheel.state.handbrake_torque
		)
	
	return any_active
