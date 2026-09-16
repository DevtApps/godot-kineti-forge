class_name KFRoadSurface
extends Resource

@export_range(0.0, 4.0) var friction_multiplier: float = 1.0
@export_range(0.0, 0.2) var rolling_resistance: float = 0.015
@export_range(0.0, 1.0) var wetness: float = 0.0

func effective_friction() -> float:
	return friction_multiplier * lerpf(1.0, 0.55, wetness)
