# Plano de Implementação para Agente Autônomo (Antigravity)

> [!IMPORTANT]
> **Restrições Operacionais Críticas para a Instância Implementadora do Antigravity:**
> 1. **Cena Base para o Parser de Ambientes (`scene_house`):**
>    A cena de referência para o parser CoppeliaSim $\rightarrow$ Unity da **Etapa 1** é a [`scene_house/scene_house.ttt`](file:///home/mateus/ia368/projeto_final/scene_house/scene_house.ttt) (espelhada no Windows em `C:\Users\mateu\unicamp\IA368\projeto_final\scene_house\scene_house.ttt`). Ela representa o ambiente doméstico completo (paredes, pisos, cômodos, mobília) e atualmente **ainda não contém o robô**. O parser ZMQ (`zmq_scene_streamer.lua` e `CoppeliaSceneBuilder.cs`) deve ser testado contra essa cena, extraindo robustamente malhas estáticas simples e compostas (*compound shapes*). Ele deve ser flexível para operar com ou sem robô: se `/myRobot` estiver ausente, ele constrói toda a casa; quando `/myRobot` (Pioneer P3-DX) for inserido na cena posteriormente, o streaming de poses dinâmicas na porta `23006` passa a sincronizar o robô automaticamente.
> 2. **Plataforma Robótica Alvo Padronizada (Pioneer P3-DX):**
>    O robô a ser integrado na cena e controlado pelas etapas de ROS 2 é o **Pioneer P3-DX (`/myRobot`)** com cinemática diferencial ($r = 0.0975\text{ m}, L = 0.331\text{ m}$, motores `/leftMotor` e `/rightMotor`), LiDAR 2D (publicando `/myRobot/scan`), Vision Sensor frontal (`/camera/image_raw`) e odometria Ground Truth (`/myRobot/odom` e TF `odom -> base_link`).
> 3. **Alinhamento Espacial Determinístico (Zero Calibração):**
>    Tanto o Unity quanto o CoppeliaSim/ROS 2 compartilham a origem $(0,0,0)$. O alinhamento espacial entre o mundo Unity e o frame `/map` do ROS 2 é puramente geométrico e constante:
>    $$x_{\text{ros}} = z_{\text{unity}}, \quad y_{\text{ros}} = -x_{\text{unity}}, \quad z_{\text{ros}} = y_{\text{unity}}$$
>    Não há necessidade de marcadores fiduciais ArUco nem de rotinas manuais de alinhamento.
> 4. **Sem Acesso ao CoppeliaSim pelo Agente:** Você está executando dentro do WSL2 e **NÃO consegue** acessar o CoppeliaSim no Windows nem iniciar simulações remotamente (o CoppeliaSim estará fechado durante a sua implementação). Todos os nós ROS 2 devem tratar falhas de conexão com o CoppeliaSim de forma graciosa (retry loop ou `mock_mode:=true`) e **jamais** depender do CoppeliaSim aberto para testes automatizados.
> 5. **Sem Acesso ao Unity Editor pelo Agente:** Você **NÃO tem acesso** ao Unity Editor nem ao compilador C# do projeto Unity no Windows. Os scripts C# devem ser escritos em `/home/mateus/ia368/projeto_final/unity_scripts/` acompanhados de um guia detalhado (`unity_scripts/README_UNITY_SETUP.md`) explicando ao usuário como configurar a cena, os GameObjects, as Layers e os componentes no Unity.
> 6. **Arquitetura de Diretórios Híbrida (WSL2 + Windows):**
>    * **No WSL2 (`ext4`):** `/home/mateus/ia368/projeto_final/` hospeda o workspace ROS 2 (`ros2_ws/`), os testes automatizados (`tests/`), a cena [`scene_house/`](file:///home/mateus/ia368/projeto_final/scene_house/) e a cópia mestre dos scripts (`unity_scripts/`, `coppelia_scripts/`, `scripts/`).
>    * **No Windows (`NTFS`):** `/mnt/c/Users/mateu/unicamp/IA368/projeto_final/` (`C:\Users\mateu\unicamp\IA368\projeto_final`, acessível via symlink `/home/mateus/ia368/projeto_final/windows_workspace`) é a pasta nativa do projeto Unity e da cena no CoppeliaSim.
>    * **Regra de Sincronização:** Sempre que criar ou atualizar arquivos em `unity_scripts/`, `coppelia_scripts/`, `scene_house/` ou `scripts/*.ps1`, execute `bash /home/mateus/ia368/projeto_final/scripts/sync_to_windows.sh` para espelhá-los automaticamente no Windows.
> 7. **Estratégia de Verificação Offline no WSL2:** Toda verificação autônoma realizada por você deve rodar **100% offline dentro do WSL2**, utilizando `colcon build`, testes em Python (`pytest`), *mocks* locais de ZMQ e *Mock Action Servers* do ROS 2.

---

## 0. Contexto do Ambiente e Ativos de Referência Existentes

Antes de criar qualquer código do zero, utilize os recursos já instalados e validados nesta máquina:

* **Sistema Operacional:** Host Windows + Guest WSL2 (Ubuntu 24.04 Linux).
* **Distribuição ROS 2:** **ROS 2 Jazzy** (`/opt/ros/jazzy/setup.bash`).
* **Bibliotecas Python Instaladas:** `coppeliasim-zmqremoteapi-client`, `numpy`.
* **Cena Base do Ambiente Doméstico:** [`scene_house/scene_house.ttt`](file:///home/mateus/ia368/projeto_final/scene_house/scene_house.ttt) (cenário com cômodos, paredes e mobília para a Etapa 1).
* **Repositório de Referência da Disciplina (`/home/mateus/ia368/IA368ii/IA368_ws`):**
  Contém código funcional já testado com o Pioneer P3-DX (`/myRobot`) no CoppeliaSim que **deve ser usado como base direta**:
  * Launch Nav2 Jazzy: [nav2_jazzy.launch.py](file:///home/mateus/ia368/IA368ii/IA368_ws/src/ia368_pkg/launch/nav2_jazzy.launch.py)
  * Launch SLAM Toolbox: [slam_toolbox.launch.py](file:///home/mateus/ia368/IA368ii/IA368_ws/src/ia368_pkg/launch/slam_toolbox.launch.py)
  * Ponte LiDAR (`/myRobot/scan`): [lidar.py](file:///home/mateus/ia368/IA368ii/IA368_ws/src/ia368_pkg/slam_toolbox/lidar.py)
  * Ponte TF (`odom -> base_link`): [tf_node.py](file:///home/mateus/ia368/IA368ii/IA368_ws/src/ia368_pkg/slam_toolbox/tf_node.py)
  * Ponte Odometria Ground Truth (`/myRobot/odom`): [ground_truth_node.py](file:///home/mateus/ia368/IA368ii/IA368_ws/src/ia368_pkg/nav2/ground_truth_node.py)
  * Ponte Velocidade P3-DX (`/cmd_vel` $\rightarrow$ `/leftMotor`, `/rightMotor`): [velocity_node.py](file:///home/mateus/ia368/IA368ii/IA368_ws/src/ia368_pkg/position_control/velocity_node.py)
  * Ponte Câmera Frontal (`sim.getVisionSensorImg` + `sim.transformImage`): [kinect.py](file:///home/mateus/ia368/IA368ii/IA368_ws/src/ia368_pkg/rtabmap/kinect.py)
  * Parâmetros validados do Nav2 e SLAM: [nav2_params_jazzy.yaml](file:///home/mateus/ia368/IA368ii/IA368_ws/config/nav2_params_jazzy.yaml) e [mapper_params_online_async.yaml](file:///home/mateus/ia368/IA368ii/IA368_ws/config/mapper_params_online_async.yaml)

---

## 1. Estrutura de Diretórios Alvo (Híbrida WSL2 + Windows)

```text
# 1. Diretório Base no WSL2 (ext4 - Rápido para ROS 2 / colcon / pytest)
/home/mateus/ia368/projeto_final/
├── PROPOSTA_PROJETO.md
├── PLANO_IMPLEMENTACAO.md
├── windows_workspace -> /mnt/c/Users/mateu/unicamp/IA368/projeto_final
├── scene_house/
│   └── scene_house.ttt                 # Cena base da casa (paredes, pisos, mobília) para o parser da Etapa 1
├── coppelia_scripts/
│   ├── zmq_scene_streamer.lua          # Customization Script Lua (simZMQ ports 23005/23006)
│   ├── README_COPPELIA_SETUP.md        # Passo a passo para o usuário colar o script Lua no CoppeliaSim
│   └── test_zmq_client.py              # Script Python para testar o protocolo ZMQ (contra Mock ou CoppeliaSim)
├── scripts/
│   ├── sync_to_windows.sh              # Sincroniza unity_scripts, coppelia_scripts, scene_house e .ps1 para o Windows
│   ├── setup_adb_reverse.ps1           # Script PowerShell (Windows) para adb reverse e portproxy
│   └── get_windows_host_ip.sh          # Helper bash para detectar ou receber o IP do host Windows
├── tests/                              # Testes automatizados offline (executados pelo Antigravity no WSL2)
│   ├── test_coordinate_math.py         # Valida conversões de eixos ROS 2 <-> Unity
│   ├── test_minimap_uv_math.py         # Valida conversões de UV -> métrica ROS, amostragem de waypoints e máscara keepout
│   ├── test_zmq_protocol_mock.py       # Simula o servidor ZMQ (23005/23006) localmente e valida o schema JSON
│   ├── test_mission_supervisor.py      # Testa NavigateToPose, NavigateThroughPoses, Pause, Resume, Cancel e Keepout
│   └── test_twist_mux_offline.py       # Testa a prioridade do twist_mux (/cmd_vel_teleop vs /cmd_vel_nav)
├── ros2_ws/
│   └── src/
│       ├── ROS-TCP-Endpoint/           # Repositório oficial Unity-Technologies/ROS-TCP-Endpoint (branch main-ros2)
│       └── vr_robot_pkg/               # Pacote ROS 2 principal do projeto final
│           ├── config/
│           │   ├── mapper_params_online_async.yaml
│           │   ├── nav2_params_jazzy.yaml  # Configurado com Costmap Filter (Keepout Zone)
│           │   └── twist_mux.yaml
│           ├── launch/
│           │   └── vr_robot_bringup.launch.py
│           ├── vr_robot_pkg/
│           │   ├── __init__.py
│           │   ├── coppelia_bridge_node.py    # Ponte P3-DX (LiDAR, Odom, TF, Vel, Camera) com suporte a mock_mode
│           │   └── mission_supervisor_node.py # Supervisor: NavigateToPose, NavigateThroughPoses, Keepout Mask, Pause/Resume
│           ├── package.xml
│           └── setup.py
└── unity_scripts/                      # Cópia mestre dos scripts C# (sincronizada via sync_to_windows.sh)
    ├── README_UNITY_SETUP.md           # Guia detalhado de montagem da cena no Unity para o usuário
    ├── Layer1_SimulatedWorld_ZMQ/
    │   ├── CoppeliaSceneBuilder.cs     # NetMQ REQ (23005) -> new Mesh(), MeshCollider
    │   ├── CoppeliaPoseSubscriber.cs   # NetMQ SUB (23006) -> Atualiza Transforms dos objetos móveis
    │   └── CoordinateConverter.cs      # Conversão matemática CoppeliaSim (RH Z-up) <-> Unity (LH Y-up)
    └── Layer2_ARInterface_ROS2/
        ├── VRModeAndMissionMenu.cs     # Menu Espacial: Abre Mesa Tática, Seleciona Ferramentas, Teleop, Pause/Resume/Cancel
        ├── HolographicTacticalTable.cs # Mesa Tática Holográfica: minimapa manipulável (XRGrabInteractable), renderização de /map
        ├── PathPainter.cs              # Desenho de trajetória contínua no minimapa -> Action NavigateThroughPoses
        ├── NoGoZonePainter.cs          # Pincel de No-Go Zones no minimapa -> Publica nav_msgs/OccupancyGrid para Keepout Filter
        └── FirstPersonTeleopController.cs # Recebe /camera/image_raw (LoadRawTextureData) + envia /cmd_vel_teleop

# 2. Diretório Espelhado no Windows (NTFS - Nativo para Unity Editor / Gradle APK / CoppeliaSim)
/mnt/c/Users/mateu/unicamp/IA368/projeto_final/  (C:\Users\mateu\unicamp\IA368\projeto_final)
├── scene_house/
│   └── scene_house.ttt                 # Cena pronta para abrir no CoppeliaSim no Windows
├── Assets/
│   └── Scripts/
│       ├── Layer1_SimulatedWorld_ZMQ/  # Sincronizado automaticamente por scripts/sync_to_windows.sh
│       └── Layer2_ARInterface_ROS2/    # Sincronizado automaticamente por scripts/sync_to_windows.sh
├── coppelia_scripts/                   # Sincronizado automaticamente por scripts/sync_to_windows.sh
└── scripts/                            # Contém setup_adb_reverse.ps1 pronto para rodar no PowerShell
```

---

## 2. Modelagem Matemática de Transformações e Cinemática do Pioneer P3-DX

### 2.1. Cinemática Diferencial Inversa do Pioneer P3-DX
Dada a velocidade linear $v_x$ e angular $\omega_z$ vindas de `/cmd_vel` (`geometry_msgs/Twist`), com raio de roda $r = 0.0975\text{ m}$ e *wheelbase* $L = 0.331\text{ m}$:
$$\omega_{\text{right}} = \frac{v_x + \frac{L}{2}\omega_z}{r}, \quad \omega_{\text{left}} = \frac{v_x - \frac{L}{2}\omega_z}{r}$$
Essas velocidades angulares em rad/s são aplicadas diretamente às juntas `/rightMotor` e `/leftMotor` no CoppeliaSim.

### 2.2. Conversão Determinística de Eixos: ROS 2 $\leftrightarrow$ Unity
Com origens coincidentes em $(0,0,0)$:
* **Posição:** $x_u = -y_r, \quad y_u = z_r, \quad z_u = x_r \quad \Longleftrightarrow \quad x_r = z_u, \quad y_r = -x_u, \quad z_r = y_u$
* **Quaternions:** $qx_u = qy_r, \quad qy_u = -qz_r, \quad qz_u = -qx_r, \quad qw_u = qw_r$
* **Winding Order dos Triângulos:** Inverter os dois últimos índices $(i_0, i_1, i_2) \rightarrow (i_0, i_2, i_1)$.

### 2.3. Matemática da Mesa Tática Holográfica (Interação UV $\rightarrow$ Métrica ROS)
O *Raycast* do controle interage com o colisor plano da Mesa Tática Holográfica, obtendo as coordenadas normalizadas de textura $(u, v) \in [0.0, 1.0] \times [0.0, 1.0]$:
1. **Coordenadas de Grade (Grid Células):**
   $$\text{col} = \lfloor u \cdot \text{width} \rfloor, \quad \text{row} = \lfloor v \cdot \text{height} \rfloor$$
2. **Coordenadas Métricas no Frame `/map` do ROS 2:**
   $$x_{\text{map}} = \text{origin.position.x} + (\text{col} + 0.5) \cdot \text{resolution}$$
   $$y_{\text{map}} = \text{origin.position.y} + (\text{row} + 0.5) \cdot \text{resolution}$$
3. **Mapeamento de Pincel de No-Go Zones (Keepout Filter):**
   Para cada ponto pintado no raio de pincel $R_{\text{brush}}$ ao redor de $(u, v)$:
   $$\text{index} = \text{col} + \text{row} \cdot \text{width}, \quad \text{mask\_data}[\text{index}] = 100$$
   Publicado em `/keepout_filter_mask` como `nav_msgs/OccupancyGrid`.

---

## 3. Detalhamento Passo a Passo das Etapas

---

### Etapa 1: Ponte ZMQ e Parser da Cena Doméstica (`scene_house` $\rightarrow$ Unity)

#### Objetivo
Criar o script Lua (`simZMQ`) no CoppeliaSim com socket `REP` (`23005`) e `PUB` (`23006`), e os scripts C# (`NetMQ`) no Unity para instanciar a geometria completa da cena [`scene_house.ttt`](file:///home/mateus/ia368/projeto_final/scene_house/scene_house.ttt) (paredes, pisos, portas, mobília) proceduralmente em tempo de execução via `new Mesh()`, adicionando colisores e mantendo suporte pronto para streaming dinâmico de poses de objetos móveis e do Pioneer P3-DX.

#### Arquivos a Criar
1. `coppelia_scripts/zmq_scene_streamer.lua`
2. `coppelia_scripts/README_COPPELIA_SETUP.md`
3. `coppelia_scripts/test_zmq_client.py`
4. `tests/test_zmq_protocol_mock.py`
5. `unity_scripts/Layer1_SimulatedWorld_ZMQ/CoordinateConverter.cs`
6. `unity_scripts/Layer1_SimulatedWorld_ZMQ/CoppeliaSceneBuilder.cs`
7. `unity_scripts/Layer1_SimulatedWorld_ZMQ/CoppeliaPoseSubscriber.cs`

#### Especificação do Parser
* O script Lua varre recursivamente a cena com `sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type)` da cena [`scene_house.ttt`](file:///home/mateus/ia368/projeto_final/scene_house/scene_house.ttt).
* Trata shapes simples e compostos (*compound shapes* com múltiplos agrupamentos geométricos), extraindo vértices, índices e cores (`sim.getShapeViz` e `sim.getShapeMesh`).
* Se houver objetos dinâmicos (ou quando `/myRobot` for adicionado à casa), registra seus handles para publicação no socket `PUB` (`23006`). Se não houver robô ainda, o parser gera com sucesso 100% da geometria estática da casa.
* No Unity, `CoppeliaSceneBuilder.cs` cria a hierarquia de `GameObjects`, atribui materiais com as cores recebidas e anexa `MeshCollider` para os pisos e paredes.

#### Critérios de Verificação Autônoma da Etapa 1 (Offline no WSL2)
* Executar `pytest tests/test_zmq_protocol_mock.py` no WSL2 e validar com sucesso o contrato ZMQ `23005`/`23006`.
* Executar `bash scripts/sync_to_windows.sh` para espelhar os scripts e a cena no Windows.

---

### Etapa 2: Scripts de Topologia de Rede Híbrida (Windows $\leftrightarrow$ WSL2 $\leftrightarrow$ Quest 3 via ADB)

#### Objetivo
Fornecer scripts para automatizar `adb reverse` e a parametrização dinâmica de IP do host Windows nos nós do WSL2.

#### Arquivos a Criar
1. `scripts/setup_adb_reverse.ps1`
2. `scripts/get_windows_host_ip.sh`

#### Critérios de Verificação Autônoma da Etapa 2 (Offline no WSL2)
* Validar sintaxe com `bash -n scripts/get_windows_host_ip.sh`, testar retorno de IP e rodar `bash scripts/sync_to_windows.sh`.

---

### Etapa 3: Base Robótica do Pioneer P3-DX no WSL2 (Ponte ROS2-ZMQ, `slam_toolbox`, `Nav2` e Keepout Filters)

#### Objetivo
Criar e compilar o workspace `ros2_ws` com `vr_robot_pkg` e `ROS-TCP-Endpoint`, integrando a cinemática diferencial do Pioneer P3-DX, `slam_toolbox` em tempo real e `Nav2` com suporte ao plugin de **Costmap Filters (Keepout Zones)**.

#### Arquivos a Criar
1. `ros2_ws/src/vr_robot_pkg/package.xml` e `setup.py`
2. `ros2_ws/src/vr_robot_pkg/config/mapper_params_online_async.yaml`
3. `ros2_ws/src/vr_robot_pkg/config/nav2_params_jazzy.yaml` (incluindo filtro `keepout_filter` em `global_costmap` e `local_costmap`)
4. `ros2_ws/src/vr_robot_pkg/vr_robot_pkg/coppelia_bridge_node.py` (cinemática P3-DX, LiDAR, Odom Ground Truth, Câmera e suporte a `mock_mode:=true`)
5. `ros2_ws/src/vr_robot_pkg/launch/vr_robot_bringup.launch.py`

#### Critérios de Verificação Autônoma da Etapa 3 (Offline no WSL2)
* `colcon build` em `ros2_ws` sem erros.
* Testar `coppelia_bridge_node` com `mock_mode:=true` garantindo publicação sintética de `/myRobot/scan`, `/myRobot/odom` e `/tf`.

---

### Etapa 4: Locomoção VR e Menu Espacial de Controle

#### Objetivo
Configurar a locomoção do avatar e implementar o **Menu Espacial VR** (`VRModeAndMissionMenu.cs`), a partir do qual o usuário pode abrir/fechar a Mesa Tática Holográfica, selecionar ferramentas de navegação e controlar a missão.

#### Arquivos a Criar
1. `unity_scripts/Layer2_ARInterface_ROS2/VRModeAndMissionMenu.cs`
2. `tests/test_coordinate_math.py`

#### Especificação de Implementação
* Canvas flutuante no espaço VR acessível por botão no controle (botão de menu ou botão primário).
* Botões da interface:
  * **Mesa Tática Holográfica:** Abre/fecha o holograma da mesa tática à frente do operador.
  * **Ferramentas Táticas:** Point-and-Click, Path Painting, Pincel No-Go Zone.
  * **Modo Teleoperação em 1ª Pessoa:** Alterna a visão para a câmera frontal do robô.
  * **Controle de Missão:** Pausar, Retomar e Cancelar.

#### Critérios de Verificação Autônoma da Etapa 4 (Offline no WSL2)
* Executar `pytest tests/test_coordinate_math.py` validando as fórmulas de conversão geométrica de eixos e quaternions.
* Rodar `bash scripts/sync_to_windows.sh`.

---

### Etapa 5: Mesa Tática Holográfica Sob Demanda e Navegação Multimodal

#### Objetivo
Implementar a **Mesa Tática Holográfica** no Unity (invocada pelo menu, manipulável com `XRGrabInteractable`, com point-and-click, path painting e pincel de No-Go Zones) e o `mission_supervisor_node.py` no ROS 2 gerenciando as ações `NavigateToPose`, `NavigateThroughPoses` e a máscara do Keepout Filter para o Pioneer P3-DX.

#### Arquivos a Criar
1. `ros2_ws/src/vr_robot_pkg/vr_robot_pkg/mission_supervisor_node.py`
2. `unity_scripts/Layer2_ARInterface_ROS2/HolographicTacticalTable.cs`
3. `unity_scripts/Layer2_ARInterface_ROS2/PathPainter.cs`
4. `unity_scripts/Layer2_ARInterface_ROS2/NoGoZonePainter.cs`
5. `tests/test_minimap_uv_math.py`
6. `tests/test_mission_supervisor.py`

#### Especificação de Implementação
1. **`HolographicTacticalTable.cs`:**
   * Invocada pelo Menu VR. Possui `XRGrabInteractable` para agarrar, mover, rotacionar e dar zoom em escala.
   * Assina `/map` (`nav_msgs/OccupancyGrid`) e atualiza a textura holográfica com mini-avatar do Pioneer P3-DX.
   * Converte coordenadas de colisão UV para metros do `/map`.
2. **`PathPainter.cs`:**
   * Coleta pontos métricos do `/map` enquanto o usuário arrasta o laser na mesa e dispara para `nav2_msgs/action/NavigateThroughPoses`.
3. **`NoGoZonePainter.cs`:**
   * Desenha sobre a textura da máscara e publica `nav_msgs/OccupancyGrid` em `/keepout_filter_mask`.
4. **`mission_supervisor_node.py`:**
   * Action Client para `/navigate_to_pose` e `/navigate_through_poses` com controle de `PAUSE`, `RESUME` e `CANCEL`.

#### Critérios de Verificação Autônoma da Etapa 5 (Offline no WSL2)
* Executar `pytest tests/test_minimap_uv_math.py` validando as conversões de coordenadas UV, waypoints e grade de keepout.
* Executar `pytest tests/test_mission_supervisor.py` com *Mock Action Servers* para `NavigateToPose` e `NavigateThroughPoses`.
* Rodar `bash scripts/sync_to_windows.sh`.

---

### Etapa 6: Teleoperação em 1ª Pessoa Sob Demanda, `twist_mux` e Guias de Montagem

#### Objetivo
Configurar o `twist_mux` priorizando `/cmd_vel_teleop` sobre `/cmd_vel_nav`, criar o controlador de teleoperação em 1ª pessoa em C# (`FirstPersonTeleopController.cs`) com a câmera frontal do Pioneer P3-DX, sincronizar tudo com a pasta do Windows (`scripts/sync_to_windows.sh`) e documentar o passo a passo completo para o usuário montar a cena no Unity e no CoppeliaSim.

#### Arquivos a Criar
1. `ros2_ws/src/vr_robot_pkg/config/twist_mux.yaml`
2. `unity_scripts/Layer2_ARInterface_ROS2/FirstPersonTeleopController.cs`
3. `unity_scripts/README_UNITY_SETUP.md`
4. `tests/test_twist_mux_offline.py`

#### Critérios de Verificação Autônoma da Etapa 6 (Offline no WSL2)
* Executar `pytest tests/test_twist_mux_offline.py` e rodar a suíte completa de testes:
  ```bash
  pytest tests/
  colcon build --packages-select vr_robot_pkg
  bash scripts/sync_to_windows.sh
  ```
* Verificar que todos os arquivos C#, Lua e PowerShell estão espelhados em `/mnt/c/Users/mateu/unicamp/IA368/projeto_final/`.
