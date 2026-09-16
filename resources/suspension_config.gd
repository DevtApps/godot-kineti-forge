class_name KFSuspensionConfig
extends Resource

enum GeometryType {
	STRAIGHT,
	MACPHERSON_LOOKUP,
	DOUBLE_WISHBONE_LOOKUP,
	SOLID_AXLE
}

@export_group("Travel & Geometry")
## Comprimento de repouso (rest length) da suspensão em metros.
@export var rest_length: float = 0.45
## Curso máximo de compressão (bump) em metros.
@export var travel: float = 0.25
## Curso máximo de extensão abaixo do comprimento de repouso.
@export var droop_travel: float = 0.10
## Velocidade visual/física com que a roda alcança o droop máximo no ar.
@export var droop_extension_rate: float = 0.8
## Ângulo de camber estático em graus (negativo = topo inclinado para dentro).
@export var static_camber: float = -1.0
## Ângulo de toe estático em graus (positivo = toe-in / convergência).
@export var static_toe: float = 0.1
## Ângulo de caster em graus.
@export var caster_angle: float = 4.0
@export var geometry_type: GeometryType = GeometryType.STRAIGHT
@export var geometry_lookup: KFSuspensionLookup
@export_range(0.0, 1.0) var anti_dive_ratio: float = 0.0
@export_range(0.0, 1.0) var anti_squat_ratio: float = 0.0
@export var unsprung_mass: float = 0.0

@export_group("Spring & Damping")
## Constante de mola (Spring rate) em N/m.
@export var spring_rate: float = 38000.0
## Pré-carga da mola em N.
@export var spring_preload: float = 1200.0
## Amortecimento em compressão (Bump damping) em N·s/m.
@export var bump_damping: float = 3500.0
## Amortecimento em extensão (Rebound damping) em N·s/m (tipicamente ~1.5x a 2x o bump).
@export var rebound_damping: float = 5200.0
## Limite de velocidade usado para impedir picos ao perder/recuperar contato.
@export var max_compression_velocity: float = 5.0
@export var max_suspension_force: float = 30000.0
## Normal mínima aceitável em relação ao "up" do veículo. Evita que uma
## suspensão quase horizontal use o chão como contato lateral ao capotar.
@export_range(-1.0, 1.0) var contact_normal_min_dot: float = 0.45
## Acima deste alinhamento a suspensão recupera 100% da força.
@export_range(-1.0, 1.0) var contact_normal_full_force_dot: float = 0.78

@export_group("Bump Stop & Limits")
## Início da atuação do batente / bump stop (fração do curso, ex: 0.85 = últimos 15%).
@export var bump_stop_threshold: float = 0.82
## Rigidez do batente / bump stop em N/m. A força máxima é esta rigidez
## multiplicada pelo comprimento da região final definido pelo threshold.
@export var bump_stop_rate: float = 90000.0
## Expoente progressivo do batente (n = 2 a 3).
@export var bump_stop_exponent: float = 2.5

@export_group("Anti-Roll Bar")
## Rigidez da barra estabilizadora (Anti-roll bar stiffness) em N/m por diferença de compressão.
@export var anti_roll_rate: float = 12000.0
## Amortecimento da diferença de velocidade entre as duas suspensões.
@export var anti_roll_damping: float = 1600.0
