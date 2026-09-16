class_name KFDebugDraw
extends Node3D

@export var enabled: bool = true
@export var force_scale: float = 0.001 # 1000 N = 1.0m
@export var show_suspension: bool = true
@export var show_forces: bool = true

var vehicle: KFVehicleBody
var mesh_instance: MeshInstance3D
var imm_mesh: ImmediateMesh

func _ready() -> void:
	vehicle = get_parent() as KFVehicleBody
	
	imm_mesh = ImmediateMesh.new()
	mesh_instance = MeshInstance3D.new()
	mesh_instance.mesh = imm_mesh
	mesh_instance.top_level = true
	add_child(mesh_instance)
	
	var mat := StandardMaterial3D.new()
	mat.shading_mode = StandardMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	mat.no_depth_test = true
	mesh_instance.material_override = mat

func _process(_delta: float) -> void:
	if not enabled or vehicle == null or vehicle.solver == null:
		if imm_mesh.get_surface_count() > 0:
			imm_mesh.clear_surfaces()
		return
	
	imm_mesh.clear_surfaces()
	imm_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	
	for wheel in vehicle.solver.wheels:
		var st: KFWheelState = wheel.state
		var mount_pos: Vector3 = wheel.get_mount_global_position(vehicle.global_transform)
		
		# 1. Linha da suspensão (Verde)
		if show_suspension:
			imm_mesh.surface_set_color(Color.GREEN)
			imm_mesh.surface_add_vertex(mount_pos)
			var target_pos: Vector3 = mount_pos - vehicle.global_transform.basis.y.normalized() * (wheel.suspension_config.rest_length + wheel.suspension_config.travel)
			imm_mesh.surface_add_vertex(st.contact_point if st.in_contact else target_pos)
		
		if st.in_contact and show_forces:
			var cp: Vector3 = st.contact_point
			
			# 2. Carga Vertical Fz (Azul)
			imm_mesh.surface_set_color(Color.DODGER_BLUE)
			imm_mesh.surface_add_vertex(cp)
			imm_mesh.surface_add_vertex(cp + st.contact_normal * (st.normal_load * force_scale))
			
			# 3. Força Longitudinal Fx (Vermelho)
			imm_mesh.surface_set_color(Color.RED)
			imm_mesh.surface_add_vertex(cp)
			imm_mesh.surface_add_vertex(cp + st.wheel_forward * (st.longitudinal_force * force_scale))
			
			# 4. Força Lateral Fy (Amarelo)
			imm_mesh.surface_set_color(Color.YELLOW)
			imm_mesh.surface_add_vertex(cp)
			imm_mesh.surface_add_vertex(cp + st.wheel_right * (st.lateral_force * force_scale))
	
	imm_mesh.surface_end()

