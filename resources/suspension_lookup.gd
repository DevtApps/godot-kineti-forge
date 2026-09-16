class_name KFSuspensionLookup
extends Resource

@export var camber_samples: PackedFloat32Array
@export var toe_samples: PackedFloat32Array
@export var caster_samples: PackedFloat32Array
@export var lateral_offset_samples: PackedFloat32Array
@export var longitudinal_offset_samples: PackedFloat32Array
@export var motion_ratio_samples: PackedFloat32Array

func sample(samples: PackedFloat32Array, normalized_travel: float, fallback: float = 0.0) -> float:
	if samples.is_empty():
		return fallback
	if samples.size() == 1:
		return samples[0]
	var f: float = clampf(normalized_travel, 0.0, 1.0) * float(samples.size() - 1)
	var i0: int = floori(f)
	var i1: int = mini(i0 + 1, samples.size() - 1)
	return lerpf(samples[i0], samples[i1], f - float(i0))

func sample_geometry(normalized_travel: float) -> Dictionary:
	return {
		"camber": sample(camber_samples, normalized_travel),
		"toe": sample(toe_samples, normalized_travel),
		"caster": sample(caster_samples, normalized_travel),
		"lateral_offset": sample(lateral_offset_samples, normalized_travel),
		"longitudinal_offset": sample(longitudinal_offset_samples, normalized_travel),
		"motion_ratio": maxf(sample(motion_ratio_samples, normalized_travel, 1.0), 0.01),
	}
