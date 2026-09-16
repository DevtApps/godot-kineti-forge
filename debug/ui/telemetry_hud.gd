class_name KFTelemetryHUD
extends CanvasLayer

@export var vehicle: KFVehicleBody

@onready var speed_label: Label = $Panel/VBox/SpeedLabel
@onready var rpm_label: Label = $Panel/VBox/RPMLabel
@onready var gear_label: Label = $Panel/VBox/GearLabel
@onready var gforce_label: Label = $Panel/VBox/GForceLabel
@onready var wheels_container: HBoxContainer = $Panel/VBox/WheelsContainer

func _process(_delta: float) -> void:
	if vehicle == null or vehicle.solver == null:
		return
	
	var st := vehicle.solver.state
	
	speed_label.text = "Velocidade: %3.1f km/h" % st.speed_kmh
	rpm_label.text = "Motor: %4.0f RPM | Torque: %3.0f Nm" % [st.engine_rpm, st.engine_torque]
	gear_label.text = "Marcha: %s | Boost: %.2f bar" % [st.gear_name, st.turbo_boost_bar]
	gforce_label.text = "G-Lat: %+.2f G | G-Long: %+.2f G" % [st.lateral_g, st.longitudinal_g]
	
	# Atualiza telemetria de cada roda se os labels existirem
	var wheel_names := ["FL", "FR", "RL", "RR"]
	for i in range(min(vehicle.solver.wheels.size(), wheel_names.size())):
		var w: KFWheel = vehicle.solver.wheels[i]
		var label_node: Label = wheels_container.get_node_or_null("Wheel_" + wheel_names[i])
		if label_node != null:
			var wst: KFWheelState = w.state
			label_node.text = "[%s]\nFz: %4.0f N\nFx: %4.0f N\nFy: %4.0f N\nSlip κ: %+.1f%%\nSlip α: %+.1f°\nComp: %2.1f cm\nABS: %s\nTCS: %s" % [
				wheel_names[i],
				wst.normal_load,
				wst.longitudinal_force,
				wst.lateral_force,
				wst.relaxed_slip_ratio * 100.0,
				rad_to_deg(wst.relaxed_slip_angle),
				wst.compression * 100.0,
				"ON" if wst.abs_active else "OFF",
				"ON" if wst.tcs_active else "OFF"
			]

