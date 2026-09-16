class_name KFAssistTCS
extends RefCounted

var enabled: bool = true
var target_slip: float = 0.20 # Limite de slip em aceleração
var cut_rate: float = 12.0
var restore_rate: float = 8.0

var torque_multiplier: float = 1.0

func solve(wheels: Array[KFWheel], dt: float) -> bool:
	if not enabled:
		torque_multiplier = 1.0
		for wheel in wheels:
			wheel.state.tcs_active = false
		return false
	
	var max_slip: float = 0.0
	var any_slip_excess: bool = false
	
	for wheel in wheels:
		if not wheel.is_driven or not wheel.state.in_contact:
			wheel.state.tcs_active = false
			continue
		
		var slip: float = wheel.state.relaxed_slip_ratio
		if slip > target_slip:
			any_slip_excess = true
			max_slip = maxf(max_slip, slip)
			wheel.state.tcs_active = true
		else:
			wheel.state.tcs_active = false
	
	if any_slip_excess:
		var excess: float = (max_slip - target_slip) / target_slip
		torque_multiplier = maxf(0.15, torque_multiplier - cut_rate * excess * dt)
		return true
	else:
		torque_multiplier = minf(1.0, torque_multiplier + restore_rate * dt)
		return false
