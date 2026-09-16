class_name KFContactSolver
extends RefCounted

## Varre o volume do pneu ao longo da suspensão. Diferentemente do raycast,
## detecta quinas, degraus e obstáculos que atingem as laterais da roda.
static func cast_wheel_shape(
	space_state: PhysicsDirectSpaceState3D,
	vehicle_rid: RID,
	mount_global_pos: Vector3,
	suspension_down_dir: Vector3,
	rest_length: float,
	travel: float,
	wheel_shape: Shape3D,
	query_margin: float,
	collision_mask: int,
	out_state: KFWheelState
) -> bool:
	var max_center_travel: float = rest_length + travel
	var motion: Vector3 = suspension_down_dir.normalized() * max_center_travel
	var query := PhysicsShapeQueryParameters3D.new()
	query.shape = wheel_shape
	query.transform = Transform3D(Basis.IDENTITY, mount_global_pos)
	query.motion = motion
	query.margin = maxf(query_margin, 0.001)
	query.collision_mask = collision_mask
	query.exclude = [vehicle_rid]
	query.collide_with_areas = false
	query.collide_with_bodies = true

	# cast_motion ignora colisores que já sobrepõem a shape na origem. Isso
	# ocorre em aterrissagens fortes, quando o mount entra no piso entre ticks.
	# Recupera esse contato antes do sweep para a suspensão poder levantar o
	# chassis novamente.
	var initial_rest: Dictionary = space_state.get_rest_info(query)
	if not initial_rest.is_empty():
		_set_shape_contact_from_rest(
			out_state,
			initial_rest,
			_shape_radius(wheel_shape)
		)
		return true

	var fractions: PackedFloat32Array = space_state.cast_motion(query)
	if fractions.size() < 2 or (fractions[0] >= 1.0 and fractions[1] >= 1.0):
		_clear_contact(out_state, max_center_travel)
		return false

	var unsafe_fraction: float = clampf(fractions[1], 0.0, 1.0)
	var overlap_distance: float = minf(max_center_travel, unsafe_fraction * max_center_travel + maxf(query_margin, 0.002))
	var center_position: Vector3 = mount_global_pos + suspension_down_dir.normalized() * overlap_distance
	query.transform.origin = center_position
	query.motion = Vector3.ZERO
	var rest: Dictionary = space_state.get_rest_info(query)

	if rest.is_empty():
		# Jolt pode devolver o safe/unsafe fraction sem manter overlap no ponto
		# exato. Avança uma pequena margem adicional e consulta novamente.
		query.transform.origin += suspension_down_dir.normalized() * maxf(query_margin, 0.01)
		rest = space_state.get_rest_info(query)
	if rest.is_empty():
		var contacts: Array[Vector3] = space_state.collide_shape(query, 1)
		if contacts.size() < 2:
			_clear_contact(out_state, max_center_travel)
			return false
		var overlaps: Array[Dictionary] = space_state.intersect_shape(query, 1)
		out_state.in_contact = true
		out_state.contact_point = contacts[1]
		out_state.contact_normal = (contacts[0] - contacts[1]).normalized()
		out_state.contact_distance = unsafe_fraction * max_center_travel + _shape_radius(wheel_shape)
		out_state.collider = overlaps[0].collider if not overlaps.is_empty() else null
		_read_surface(out_state)
		return true

	_set_shape_contact_from_rest(
		out_state,
		rest,
		unsafe_fraction * max_center_travel + _shape_radius(wheel_shape)
	)
	return true

static func _set_shape_contact_from_rest(
	out_state: KFWheelState,
	rest: Dictionary,
	contact_distance: float
) -> void:
	out_state.in_contact = true
	out_state.contact_point = rest.point
	out_state.contact_normal = (rest.normal as Vector3).normalized()
	out_state.contact_distance = contact_distance
	var collider_id: int = int(rest.get("collider_id", 0))
	out_state.collider = instance_from_id(collider_id) if collider_id != 0 else null
	_read_surface(out_state)

static func _shape_radius(shape: Shape3D) -> float:
	return (shape as SphereShape3D).radius if shape is SphereShape3D else 0.0

static func _clear_contact(out_state: KFWheelState, distance: float) -> void:
	out_state.in_contact = false
	out_state.contact_point = Vector3.ZERO
	out_state.contact_normal = Vector3.UP
	out_state.contact_distance = distance
	out_state.collider = null
	out_state.compression = 0.0
	out_state.surface_friction_multiplier = 1.0
	out_state.surface_rolling_resistance = -1.0

static func _read_surface(out_state: KFWheelState) -> void:
	out_state.surface_friction_multiplier = 1.0
	out_state.surface_rolling_resistance = -1.0
	if out_state.collider != null and out_state.collider.has_meta("kf_road_surface"):
		var surface: Variant = out_state.collider.get_meta("kf_road_surface")
		if surface is KFRoadSurface:
			out_state.surface_friction_multiplier = surface.effective_friction()
			out_state.surface_rolling_resistance = surface.rolling_resistance
	elif out_state.collider != null and out_state.collider.has_meta("kf_friction_multiplier"):
		out_state.surface_friction_multiplier = maxf(float(out_state.collider.get_meta("kf_friction_multiplier")), 0.0)

## Executa a consulta de raio no espaço físico para detectar contato do pneu com o solo
static func cast_wheel_ray(
	space_state: PhysicsDirectSpaceState3D,
	vehicle_rid: RID,
	mount_global_pos: Vector3,
	suspension_down_dir: Vector3,
	rest_length: float,
	travel: float,
	tire_radius: float,
	collision_mask: int,
	out_state: KFWheelState
) -> bool:
	# Raio longo o suficiente para cobrir repouso + curso completo + raio do pneu + margem
	var max_cast_dist: float = rest_length + travel + tire_radius + 0.40
	var ray_start: Vector3 = mount_global_pos
	var ray_end: Vector3 = mount_global_pos + suspension_down_dir * max_cast_dist
	
	var query := PhysicsRayQueryParameters3D.create(ray_start, ray_end, collision_mask)
	query.exclude = [vehicle_rid]
	query.collide_with_areas = false
	query.collide_with_bodies = true
	
	var hit := space_state.intersect_ray(query)
	
	if hit.is_empty():
		_clear_contact(out_state, max_cast_dist)
		return false
	
	out_state.in_contact = true
	out_state.contact_point = hit.position
	out_state.contact_normal = (hit.normal as Vector3).normalized()
	out_state.collider = hit.collider
	_read_surface(out_state)
	out_state.contact_distance = ray_start.distance_to(hit.position)
	return true
