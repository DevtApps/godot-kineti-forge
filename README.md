# GodotKinetiForge

<p align="center">
  <a href="#english">English</a> |
  <a href="#portugues">Português</a>
</p>

<a id="english"></a>

## English

GodotKinetiForge is an independent Godot 4.7 reimplementation of the vehicle
physics concepts introduced by the original
[KinetiForge Vehicle System](https://github.com/myoozy/KinetiForge-Vehicle-System).

The original project was created by
[Zhengyi Miao (`myoozy`)](https://github.com/myoozy) for Unreal Engine 5 and is
distributed under the MIT License. Full credit for the original architecture,
research direction, and KinetiForge name belongs to its author and
contributors.

This repository is maintained by **DevtApps**. It is not an official port and
is not affiliated with or endorsed by the upstream project.

## About the reimplementation

The Unreal Engine C++ component architecture was redesigned around Godot's
`RigidBody3D`, `PhysicsDirectBodyState3D`, resources, and Jolt-compatible shape
queries. It is a native GDScript implementation rather than a drop-in source
translation.

The reimplementation currently includes:

- a modular one-dimensional engine, clutch, gearbox, driveshaft, differential,
  and axle simulation;
- sphere shape-cast wheel contacts;
- Pacejka-style longitudinal and lateral tire forces, combined slip, relaxation
  length, and load sensitivity;
- spring, asymmetric damper, progressive bump-stop, droop, and anti-roll
  simulation;
- automatic transmission logic, ABS, and traction control;
- aerodynamic drag, configurable downforce, telemetry, and debug drawing;
- Godot resources for reusable vehicle, engine, transmission, differential,
  suspension, tire, and road-surface configurations.

Some upstream systems are represented with Godot-specific approximations and
do not yet have feature parity with the Unreal Engine implementation. In
particular, detailed hardpoint/linkage suspension geometry, unsprung rigid-body
dynamics, electric motor layouts, and ESP are not complete equivalents.

Please use the upstream repository for the authoritative Unreal Engine version:
<https://github.com/myoozy/KinetiForge-Vehicle-System>.

## Installation

1. Copy the complete `addons/kinetiforge` directory into the target project.
2. Open **Project > Project Settings > Plugins**.
3. Enable **GodotKinetiForge**.
4. Use `KFVehicleBody` as the script/class of the vehicle `RigidBody3D` and
   assign a `KFVehicleConfig` resource.

The addon is self-contained. The vehicles and resources under the host
project's `scenes/`, `resources/`, and `assets/` directories are examples and
are not required by the plugin.

## Runtime input

Player-controlled vehicles look for these optional Input Map actions:

- `vehicle_accelerate`
- `vehicle_brake`
- `vehicle_steer_left`
- `vehicle_steer_right`
- `vehicle_handbrake`
- `vehicle_reset`
- `vehicle_shift_up`
- `vehicle_shift_down`

Projects can instead control a vehicle through `set_throttle()`, `set_brake()`,
`set_steering()`, `set_handbrake()`, `set_clutch()`, and `set_gear()`.

## Physics

Godot Jolt is recommended. The solver sets the vehicle body mass, inertia,
center of mass, and damping from `KFVehicleConfig`. Wheel contacts use sphere
shape casts against the configured road collision mask.

Use a physics tick rate of at least 120 Hz. `KFVehicleBody` raises the project
runtime rate to 120 Hz when necessary.

## Package contents

- `core/`: vehicle body, solver, state, input, and math helpers
- `drivetrain/`: engine, clutch, gearbox, differential, brakes, and axles
- `wheel/`: contact, slip, tire force, and wheel rotation solvers
- `suspension/`: spring/damper and anti-roll models
- `assists/`: automatic gearbox, ABS, and traction control
- `aero/`: aerodynamic forces and damping
- `resources/`: configuration resource classes
- `debug/`: optional telemetry HUD and debug visualization

---

<a id="portugues"></a>

## Português

O GodotKinetiForge é uma reimplementação independente para Godot 4.7 dos
conceitos de física veicular apresentados pelo projeto original
[KinetiForge Vehicle System](https://github.com/myoozy/KinetiForge-Vehicle-System).

O projeto original foi criado por
[Zhengyi Miao (`myoozy`)](https://github.com/myoozy) para Unreal Engine 5 e é
distribuído sob a Licença MIT. Todos os créditos pela arquitetura original,
direção da pesquisa e nome KinetiForge pertencem ao autor e aos colaboradores
do projeto original.

Este repositório é mantido pela **DevtApps**. Ele não é um port oficial e não
possui afiliação nem endosso do projeto original.

### Sobre a reimplementação

A arquitetura de componentes em C++ da Unreal Engine foi reprojetada em torno
de `RigidBody3D`, `PhysicsDirectBodyState3D`, recursos do Godot e consultas de
forma compatíveis com o Jolt. Esta é uma implementação nativa em GDScript, e
não uma tradução direta do código-fonte.

A reimplementação inclui atualmente:

- simulação modular unidimensional de motor, embreagem, câmbio, eixo de
  transmissão, diferencial e semieixos;
- contato das rodas por sphere shape cast;
- forças longitudinais e laterais de pneus no estilo Pacejka, atrito combinado,
  comprimento de relaxamento e sensibilidade à carga;
- simulação de mola, amortecedor assimétrico, batente progressivo, extensão e
  barra antirrolagem;
- transmissão automática, ABS e controle de tração;
- arrasto aerodinâmico, downforce configurável, telemetria e visualização de
  depuração;
- recursos do Godot reutilizáveis para configurar veículo, motor, transmissão,
  diferencial, suspensão, pneus e superfícies.

Alguns sistemas do projeto original usam aproximações específicas para Godot e
ainda não possuem paridade completa com a implementação da Unreal Engine. Em
especial, a geometria detalhada de hardpoints e braços da suspensão, dinâmica
de massas não suspensas, conjuntos de motores elétricos e ESP ainda não são
equivalentes completos.

Consulte o repositório original para obter a versão oficial para Unreal Engine:
<https://github.com/myoozy/KinetiForge-Vehicle-System>.

### Instalação

1. Copie o diretório completo `addons/kinetiforge` para o projeto de destino.
2. Abra **Projeto > Configurações do Projeto > Plugins**.
3. Ative **GodotKinetiForge**.
4. Use `KFVehicleBody` como script/classe do `RigidBody3D` do veículo e atribua
   um recurso `KFVehicleConfig`.

O addon é independente. Os veículos e recursos localizados nos diretórios
`scenes/`, `resources/` e `assets/` do projeto hospedeiro são exemplos e não
são necessários para o funcionamento do plugin.

### Controles em tempo de execução

Veículos controlados pelo jogador procuram estas ações opcionais no Input Map:

- `vehicle_accelerate`
- `vehicle_brake`
- `vehicle_steer_left`
- `vehicle_steer_right`
- `vehicle_handbrake`
- `vehicle_reset`
- `vehicle_shift_up`
- `vehicle_shift_down`

O projeto também pode controlar o veículo por meio de `set_throttle()`,
`set_brake()`, `set_steering()`, `set_handbrake()`, `set_clutch()` e
`set_gear()`.

### Física

O Godot Jolt é recomendado. O solver configura massa, inércia, centro de massa
e amortecimento do corpo do veículo a partir de `KFVehicleConfig`. O contato das
rodas usa sphere shape casts contra a máscara de colisão configurada para a
pista.

Use uma frequência de física de pelo menos 120 Hz. `KFVehicleBody` eleva a taxa
de física do projeto para 120 Hz quando necessário.

### Conteúdo do pacote

- `core/`: corpo do veículo, solver, estado, entrada e utilitários matemáticos
- `drivetrain/`: motor, embreagem, câmbio, diferencial, freios e semieixos
- `wheel/`: contato, deslizamento, força dos pneus e rotação das rodas
- `suspension/`: modelos de mola, amortecedor e barra antirrolagem
- `assists/`: câmbio automático, ABS e controle de tração
- `aero/`: forças aerodinâmicas e amortecimento
- `resources/`: classes de recursos de configuração
- `debug/`: HUD de telemetria e visualização de depuração opcionais

<p align="right"><a href="#godotkinetiforge">Voltar ao topo</a></p>
