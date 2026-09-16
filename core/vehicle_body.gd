class_name KFVehicleBody
extends RigidBody3D

signal engine_started
signal engine_stalled
signal gear_changed(old_gear: int, new_gear: int)
signal backfire(intensity: float)
signal wheel_contact_changed(wheel_index: int, in_contact: bool)
signal abs_state_changed(wheel_index: int, active: bool)
signal tcs_state_changed(active: bool)

@export var config: KFVehicleConfig

@export_group("Wheel Mounts (Node3D)")
@export var mount_fl: Node3D
@export var mount_fr: Node3D
@export var mount_rl: Node3D
@export var mount_rr: Node3D

@export_group("Visual Wheels (Node3D)")
@export var visual_fl: Node3D
@export var visual_fr: Node3D
@export var visual_rl: Node3D
@export var visual_rr: Node3D

@export_group("Visual Hubs (Node3D)")
@export var hub_fl: Node3D
@export var hub_fr: Node3D
@export var hub_rl: Node3D
@export var hub_rr: Node3D

var solver: KFVehicleSolver
var _previous_contacts: Array[bool] = [false, false, false, false]
var _previous_abs: Array[bool] = [false, false, false, false]
var _previous_tcs: bool = false

func _ready() -> void:
	if Engine.physics_ticks_per_second < 120:
		Engine.physics_ticks_per_second = 120
	
	if config == null:
		config = _create_default_config()
	
	# Configura automaticamente as rodas e cubos do modelo GLB se existirem
	_setup_glb_wheel_meshes()
	
	var mount_nodes: Dictionary = {}
	if mount_fl != null: mount_nodes["FL"] = mount_fl
	if mount_fr != null: mount_nodes["FR"] = mount_fr
	if mount_rl != null: mount_nodes["RL"] = mount_rl
	if mount_rr != null: mount_nodes["RR"] = mount_rr
	
	var visual_nodes: Dictionary = {}
	if visual_fl != null: visual_nodes["FL"] = visual_fl
	if visual_fr != null: visual_nodes["FR"] = visual_fr
	if visual_rl != null: visual_nodes["RL"] = visual_rl
	if visual_rr != null: visual_nodes["RR"] = visual_rr
	
	var hub_nodes: Dictionary = {}
	if hub_fl != null: hub_nodes["FL"] = hub_fl
	if hub_fr != null: hub_nodes["FR"] = hub_fr
	if hub_rl != null: hub_nodes["RL"] = hub_rl
	if hub_rr != null: hub_nodes["RR"] = hub_rr
	
	solver = KFVehicleSolver.new()
	solver.initialize(self, config, mount_nodes, visual_nodes, hub_nodes)
	solver.engine.engine_started.connect(func() -> void: engine_started.emit())
	solver.engine.engine_stalled.connect(func() -> void: engine_stalled.emit())
	solver.engine.backfire.connect(func(intensity: float) -> void: backfire.emit(intensity))
	solver.gearbox.gear_changed.connect(func(old_gear: int, new_gear: int) -> void: gear_changed.emit(old_gear, new_gear))

func _setup_glb_wheel_meshes() -> void:
	var visual_wheels_root: Node3D = get_node_or_null("VisualWheels")
	if visual_wheels_root == null:
		visual_wheels_root = Node3D.new()
		visual_wheels_root.name = "VisualWheels"
		add_child(visual_wheels_root)
	
	var visual_hubs_root: Node3D = get_node_or_null("VisualHubs")
	if visual_hubs_root == null:
		visual_hubs_root = Node3D.new()
		visual_hubs_root.name = "VisualHubs"
		add_child(visual_hubs_root)
	
	# Garante os nós contentores para cada roda (gira com spin)
	if visual_fl == null: visual_fl = _get_or_create_node(visual_wheels_root, "Visual_FL", Vector3(-0.865, 0.493, -1.699))
	if visual_fr == null: visual_fr = _get_or_create_node(visual_wheels_root, "Visual_FR", Vector3(0.868, 0.493, -1.699))
	if visual_rl == null: visual_rl = _get_or_create_node(visual_wheels_root, "Visual_RL", Vector3(-0.854, 0.482, 1.715))
	if visual_rr == null: visual_rr = _get_or_create_node(visual_wheels_root, "Visual_RR", Vector3(0.857, 0.482, 1.715))
	
	# Garante os nós contentores para cada cubo / manga de eixo (NÃO gira com spin)
	if hub_fl == null: hub_fl = _get_or_create_node(visual_hubs_root, "Hub_FL", Vector3(-0.865, 0.493, -1.699))
	if hub_fr == null: hub_fr = _get_or_create_node(visual_hubs_root, "Hub_FR", Vector3(0.868, 0.493, -1.699))
	if hub_rl == null: hub_rl = _get_or_create_node(visual_hubs_root, "Hub_RL", Vector3(-0.854, 0.482, 1.715))
	if hub_rr == null: hub_rr = _get_or_create_node(visual_hubs_root, "Hub_RR", Vector3(0.857, 0.482, 1.715))
	
	# Procura rodas somente dentro do modelo importado. Uma busca a partir do
	# corpo também encontra os contentores Hub_* criados acima e pode tentar
	# reparentar um contentor para ele mesmo quando um GLB novo é integrado.
	var model_root: Node = get_node_or_null("VisualModel")
	if model_root == null:
		return
	var all_children: Array[Node] = model_root.find_children("*", "Node3D", true, false)
	for child in all_children:
		var n: String = child.name.to_lower()
		var parent_node: Node = child.get_parent()
		
		# Não processa nós já organizados sob VisualWheels ou VisualHubs
		if parent_node in [visual_fl, visual_fr, visual_rl, visual_rr, hub_fl, hub_fr, hub_rl, hub_rr]:
			continue
		
		var target_parent: Node3D = null
		
		# Cubos e mangas de eixo (apenas esterçamento e camber)
		if "hub_lf" in n:
			target_parent = hub_fl
		elif "hub_rf" in n:
			target_parent = hub_fr
		elif "hub_lr" in n:
			target_parent = hub_rl
		elif "hub_rr" in n:
			target_parent = hub_rr
		# Rodas e pneus (esterçamento, camber E spin)
		elif "wheel_lf" in n:
			target_parent = visual_fl
		elif "wheel_rf" in n:
			target_parent = visual_fr
		elif "wheel_lr" in n:
			target_parent = visual_rl
		elif "wheel_rr" in n:
			target_parent = visual_rr
		
		if target_parent != null and child is Node3D and child != target_parent:
			(child as Node3D).reparent(target_parent)
			(child as Node3D).position = Vector3.ZERO
			(child as Node3D).rotation = Vector3.ZERO

func _get_or_create_node(parent: Node, node_name: String, default_pos: Vector3) -> Node3D:
	var existing: Node3D = parent.get_node_or_null(node_name)
	if existing != null:
		return existing
	var new_node := Node3D.new()
	new_node.name = node_name
	new_node.position = default_pos
	parent.add_child(new_node)
	return new_node

func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if solver != null:
		solver.physics_step(state)
		_emit_state_changes()

func _emit_state_changes() -> void:
	for i in range(solver.wheels.size()):
		var wheel_state: KFWheelState = solver.wheels[i].state
		if wheel_state.in_contact != _previous_contacts[i]:
			_previous_contacts[i] = wheel_state.in_contact
			wheel_contact_changed.emit(i, wheel_state.in_contact)
		if wheel_state.abs_active != _previous_abs[i]:
			_previous_abs[i] = wheel_state.abs_active
			abs_state_changed.emit(i, wheel_state.abs_active)
	if solver.state.tcs_active != _previous_tcs:
		_previous_tcs = solver.state.tcs_active
		tcs_state_changed.emit(_previous_tcs)

func _process(_delta: float) -> void:
	if solver != null:
		solver.update_visuals(global_transform)

## -------------------------------------------------------------
## API Pública de Controle e Leitura
## -------------------------------------------------------------

func start_engine() -> void:
	if solver != null and solver.engine != null:
		solver.engine.start()

func stop_engine() -> void:
	if solver != null and solver.engine != null:
		solver.engine.stop()

func shift_up() -> void:
	if solver != null and solver.gearbox != null:
		solver.gearbox.shift_up()

func shift_down() -> void:
	if solver != null and solver.gearbox != null:
		solver.gearbox.shift_down()

func set_gear(gear_index: int) -> void:
	if solver != null and solver.gearbox != null:
		solver.gearbox.shift_to(gear_index)

func set_throttle(value: float) -> void:
	if solver != null:
		solver.input.external_control = true
		solver.input.raw_throttle = clampf(value, 0.0, 1.0)

func set_brake(value: float) -> void:
	if solver != null:
		solver.input.external_control = true
		solver.input.raw_brake = clampf(value, 0.0, 1.0)

func set_steering(value: float) -> void:
	if solver != null:
		solver.input.external_control = true
		solver.input.raw_steer = clampf(value, -1.0, 1.0)

func set_handbrake(value: float) -> void:
	if solver != null:
		solver.input.external_control = true
		solver.input.raw_handbrake = clampf(value, 0.0, 1.0)

func set_clutch(value: float) -> void:
	if solver != null:
		solver.input.external_control = true
		solver.input.raw_clutch = clampf(value, 0.0, 1.0)

func use_player_input() -> void:
	if solver != null:
		solver.input.external_control = false

func get_speed_kmh() -> float:
	return solver.state.speed_kmh if solver != null else 0.0

func get_engine_rpm() -> float:
	return solver.state.engine_rpm if solver != null else 0.0

func get_gear_name() -> String:
	return solver.state.gear_name if solver != null else "N"

func get_wheel_state(index: int) -> KFWheelState:
	if solver != null and index >= 0 and index < solver.wheels.size():
		return solver.wheels[index].state
	return null

func get_slip_ratio(index: int) -> float:
	var wheel_state := get_wheel_state(index)
	return wheel_state.relaxed_slip_ratio if wheel_state != null else 0.0

func get_normal_load(index: int) -> float:
	var wheel_state := get_wheel_state(index)
	return wheel_state.normal_load if wheel_state != null else 0.0

func _create_default_config() -> KFVehicleConfig:
	var cfg := KFVehicleConfig.new()
	cfg.engine_config = KFEngineConfig.new()
	cfg.gearbox_config = KFGearboxConfig.new()
	cfg.front_diff_config = KFDifferentialConfig.new()
	cfg.rear_diff_config = KFDifferentialConfig.new()
	cfg.front_suspension_config = KFSuspensionConfig.new()
	cfg.rear_suspension_config = KFSuspensionConfig.new()
	cfg.front_tire_config = KFTireConfig.new()
	cfg.rear_tire_config = KFTireConfig.new()
	return cfg
