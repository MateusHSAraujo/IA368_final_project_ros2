# Proposta de Projeto: Interface Humano-Máquina em Realidade Mista e Gêmeo Digital para Controle, Navegação e Teleoperação de Robô Móvel

**Disciplina:** Design e Desenvolvimento de Robôs Autônomos Inteligentes (IA368)

---

## 1. Resumo da Proposta

O projeto propõe o desenvolvimento de uma **Interface Humano-Máquina (HMI) imersiva e futurística em Realidade Virtual/Mista (Meta Quest 3, via Unity)** para o planejamento tático, monitoramento e teleoperação em primeira pessoa de um **robô móvel diferencial Pioneer P3-DX** simulado no CoppeliaSim EDU, integrado à pilha **ROS 2 (Jazzy)** e **Nav2**.

O foco central da pesquisa e do desenvolvimento é o **Design de Interação Espacial**. O sistema opera sobre uma arquitetura de Gêmeo Digital dividida em duas camadas desacopladas:

1. **Camada de "Mundo Físico Simulado" (via ZeroMQ):** Reconstrói proceduralmente qualquer cena aberta no CoppeliaSim (malhas, cores e texturas) e atualiza as poses dos objetos móveis em tempo real. Esta camada simula o ambiente que seria visto pelas câmeras de *Passthrough* em uma aplicação real.
2. **Camada de "HMI e Controle em Realidade Mista" (via ROS 2):** Comunica-se exclusivamente com os tópicos e ações do ROS 2. Em alinhamento com a abordagem validada nos laboratórios da disciplina, adota-se a sincronização determinística de coordenadas via Ground Truth: o mundo virtual no Unity e o ambiente no ROS 2 compartilham a mesma origem $(0,0,0)$, permitindo alinhamento espacial perfeito e imediato sem a necessidade de calibrações manuais ou marcadores fiduciais adicionais.

A interface do operador é centralizada em um **Menu Espacial no VR**, a partir do qual o usuário pode invocar e alternar as seguintes funcionalidades sob demanda:
* **Mesa Tática Holográfica Sob Demanda (*Holographic Tactical Table*):** Ao ser selecionada no Menu, projeta à frente do usuário um minimapa 3D flutuante e manipulável espacialmente com as mãos/controles (*XRGrabInteractable* para mover, girar e aproximar), renderizado a partir da malha do SLAM (`slam_toolbox`). Nela, o usuário dispõe de:
  * **Point-and-Click Holográfico:** Envio de destinos pontuais (`/goal_pose`) clicando no holograma.
  * **Path Painting (Desenho Livre de Trajetória):** O usuário "pinta" um trajeto contínuo com o laser do controle sobre o minimapa, transmitido à Action `NavigateThroughPoses` do Nav2 como uma cadeia ordenada de waypoints.
  * **Pintura de No-Go Zones (Zonas Proibidas):** O usuário desenha áreas restritas com um pincel virtual sobre o minimapa; essas áreas são serializadas como uma máscara de ocupação e injetadas diretamente no **Nav2 Costmap Filter (Keepout Zone)**, fazendo com que o robô desvie ou replaneje imediatamente o trajeto.
* **Teleoperação de Resgate em 1ª Pessoa:** Painel flutuante com vídeo sob demanda da câmera frontal do Pioneer P3-DX e controle manual prioritário via joystick com multiplexação de comandos (`twist_mux`).
* **Controle do Ciclo de Vida da Missão:** Comandos de **Pausar**, **Retomar** e **Cancelar** rotas ativas do robô gerenciados por um Nó Supervisor dedicado.

---

## 2. Plataforma Robótica Alvo: Pioneer P3-DX

Para manter total compatibilidade com o ecossistema e cinemática já validados na disciplina, o robô utilizado na cena do CoppeliaSim será a plataforma diferencial **Pioneer P3-DX (`/myRobot`)**:
* **Cinemática Diferencial:**
  * Raio da roda: $r = 0.0975\text{ m}$ (diâmetro $0.195\text{ m}$).
  * Distância entre rodas (*wheel base*): $L = 0.331\text{ m}$.
  * Atuadores no CoppeliaSim: juntas rotacionais `/leftMotor` e `/rightMotor`.
* **Sensores Embarcados:**
  * **LiDAR 2D:** Sensor planar publicando no tópico `/myRobot/scan`.
  * **Câmera Frontal (Vision Sensor):** Câmera acoplada na frente do robô transmitindo imagens RGB (`sensor_msgs/Image`) sob demanda via `/camera/image_raw`.
  * **Odometria / Ground Truth:** Nó de odometria publicando em `/myRobot/odom` e fornecendo a transformação TF `odom -> base_link` sincronizada com a origem do simulador.
* **Cena de Referência no CoppeliaSim (`scene_house`):** A cena base adotada para a reconstrução do gêmeo digital é a [`scene_house/scene_house.ttt`](file:///home/mateus/ia368/projeto_final/scene_house/scene_house.ttt) (espelhada no Windows em `C:\Users\mateu\unicamp\IA368\projeto_final\scene_house\scene_house.ttt`). Trata-se de um ambiente residencial completo com múltiplos cômodos, paredes, portas, pisos e mobílias. O parser procedural em Unity via ZMQ utiliza essa cena como referência estática inicial para a geração procedural de malhas no Unity, e suporta a posterior inserção do robô diferencial Pioneer P3-DX (`/myRobot`) para streaming dinâmico de poses e teleoperação.

---

## 3. Arquitetura do Sistema e Topologia de Rede

O sistema integra o **Host Windows** (CoppeliaSim EDU + ADB), o **Guest WSL2** (ROS 2 Jazzy, Nav2, SLAM Toolbox) e o **Meta Quest 3** (Unity APK) via cabo USB-C com redirecionamento de portas (`adb reverse`):

```
+---------------------------------------------------------------------------------------------------------+
|                                              HOST: WINDOWS                                              |
|                                                                                                         |
|  +--------------------------------------------+       +----------------------------------------------+  |
|  |       CoppeliaSim EDU (Pioneer P3-DX)      |       |             ADB Server (adb.exe)             |  |
|  |                                            |       |       Port Forwarding via Cabo USB-C:        |  |
|  | - Cena Customizada com /myRobot            |       |       - tcp:23005 (ZMQ Meshes REP)           |  |
|  | - LiDAR 2D + Câmera Frontal                |       |       - tcp:23006 (ZMQ Poses PUB)            |  |
|  | - Lua simZMQ Server:                       |       |       - tcp:10000 (ROS-TCP-Endpoint)         |  |
|  |   * Porta 23005 (REP: Malhas 3D da Cena)   |<----->|       +----------------------+---------------+  |
|  |   * Porta 23006 (PUB: Poses Móveis)        |       +----------------------+-----------------------+  |
|  | - ZMQ Remote API (Porta 23000)             |                              |                          |
|  +---------------------+----------------------+                              | Cabo USB-C               |
|                        | ZMQ (Localhost)                                     v                          |
|  +---------------------+-----------------------------------------------------+-----------------------+  |
|  |                                  GUEST: WSL2 (Linux / ROS 2 Jazzy)        |                       |  |
|  |                                                                           |                       |  |
|  |  +----------------------------+             +-----------------------------v--------------------+  |  |
|  |  |   Nó Ponte Coppelia-ROS 2  |             |                 ROS-TCP-Endpoint                 |  |  |
|  |  |   (Python + ZMQ Client)    |             |                  (Porta 10000)                   |  |  |
|  |  |   - /myRobot/scan, odom, tf|             +--------+--------------------+--------------------+  |  |
|  |  |   - /camera/image_raw      |                      |                    ^                       |  |
|  |  |   - /cmd_vel               |                      v                    |                       |  |
|  |  +-------------+--------------+             +--------+--------------------+--------------------+  |  |
|  |                |                            |      Nó Supervisor de Missão & HMI (Python)      |  |  |
|  |                v                            |      - Action Client: NavigateToPose             |  |  |
|  |  +-------------+--------------+             |      - Action Client: NavigateThroughPoses       |  |  |
|  |  |      slam_toolbox          |             |      - Publica: /keepout_filter_mask             |  |  |
|  |  |   (SLAM Online Dinâmico)   |             |      - Comandos: PAUSE, RESUME, CANCEL           |  |  |
|  |  +-------------+--------------+             +--------+--------------------+--------------------+  |  |
|  |                | /map                                |                    |                       |  |
|  |                v                                     |                    |                       |  |
|  |  +-------------+--------------+                      |                    |                       |  |
|  |  |            Nav2            |<---------------------+                    |                       |  |
|  |  |  - Costmap Keepout Filter  |                                           |                       |  |
|  |  |  - Global & Local Planner  |                                           |                       |  |
|  |  +-------------+--------------+                                           |                       |  |
|  |                | /cmd_vel_nav                                             | /cmd_vel_teleop       |  |
|  |                +------------------------------+---------------------------+                       |  |
|  |                                               v                                                   |  |
|  |                                     +---------+----------+                                        |  |
|  |                                     |     twist_mux      |--> /cmd_vel                            |  |
|  |                                     +--------------------+                                        |  |
|  +---------------------------------------------------------------------------------------------------+  |
+---------------------------------------------------------------------------------------------------------+
                                                                               ^
                                                                               | Cabo USB-C (127.0.0.1)
                                                                               v
+---------------------------------------------------------------------------------------------------------+
|                                   CLIENTE: META QUEST 3 (Unity APK)                                     |
|                                                                                                         |
|  +---------------------------------------+  +--------------------------------------------------------+  |
|  |  Camada 1: Mundo Físico Simulado      |  |  Camada 2: HMI em Realidade Mista (ROS 2)              |  |
|  |  - NetMQ REQ (23005): Constrói Cena   |  |  1. Menu Espacial VR:                                  |  |
|  |    e Pioneer P3-DX em tempo real      |  |     * Botão para abrir/fechar Mesa Tática Holográfica  |  |
|  |  - NetMQ SUB (23006): Atualiza Poses  |  |     * Seleção de Ferramenta (Point, Paint, No-Go)      |  |
|  |    de Entidades Móveis                |  |     * Modo Teleoperação em 1ª Pessoa                   |  |
|  |  - Alinhamento Determinístico         |  |     * Controle de Missão (Pausar / Retomar / Cancelar) |  |
|  |    Instantâneo (Origem 0,0,0)         |  |  2. Mesa Tática Holográfica (Sob Demanda):             |  |
|  +---------------------------------------+  |     * Point-and-Click (Raycast UV -> /goal_pose)       |  |
|                                             |     * Path Painting (Risco Livre -> Waypoints)         |  |
|                                             |     * No-Go Zone Brush (Pintura de Máscara de Custo)   |  |
|                                             |     * Manipulação Espacial (Pegar, Girar, Escalar)     |  |
|                                             |  3. Monitor Virtual de 1ª Pessoa + Direção Joystick     |  |
|                                             +--------------------------------------------------------+  |
+---------------------------------------------------------------------------------------------------------+
```

---

## 4. Principais Inovações e Justificativas de Engenharia

1. **Alinhamento Espacial Determinístico (Zero Calibração):**
   Como a odometria e o SLAM utilizam a referência do mundo do simulador e o Unity importa a cena na sua própria origem $(0,0,0)$, a sincronização espacial é instantânea via conversão geométrica direta de eixos ($x_{\text{ros}} = z_{\text{unity}}, y_{\text{ros}} = -x_{\text{unity}}, z_{\text{ros}} = y_{\text{unity}}$). Isso elimina pontos de falha e etapas preliminares de calibração manual.
2. **Mesa Tática Holográfica Sob Demanda:**
   A Mesa Tática não ocupa a visão do usuário de forma contínua; ela é invocada pelo **Menu Espacial**, permitindo ao operador inspecionar a planta da casa em 3D, girar, ampliar com as mãos e planejar rotas táticas sem interferir no chão físico da cena.
3. **Desacoplamento por Coordenadas UV de Textura:**
   A interação de apontamento, desenho de rota e pintura de zonas ocorre sobre a superfície do holograma. O *Raycast* obtém coordenadas normalizadas $(u, v) \in [0, 1] \times [0, 1]$, que são convertidas diretamente para coordenadas métricas do `/map` a partir da resolução e origem da grade, sem qualquer acoplamento com o chão da simulação.
4. **Navegação Multimodal (Point-and-Click + Path Painting):**
   O operador pode enviar metas pontuais (`NavigateToPose`) ou traçar caminhos à mão livre na Mesa Tática, executados via Action `NavigateThroughPoses` do Nav2.
5. **Controle Dinâmico de Barreiras Virtuais via Nav2 Costmap Filters:**
   Permite delimitar zonas restritas ("No-Go Zones") com um pincel virtual no minimapa. A pintura gera uma máscara de ocupação enviada diretamente ao plugin **Keepout Filter** do Nav2, provocando o desvio automático da rota em tempo real.
6. **Topologia Cabeada USB de Alta Performance via ADB:**
   Uso de `adb reverse` para conexões em `localhost` nas portas `23005`, `23006` (ZMQ) e `10000` (ROS), eliminando oscilações de rede sem fio e garantindo alta taxa de quadros (FPS) no Meta Quest 3.

---

## 5. Stack Tecnológico

### Simulação e Gêmeo Digital (Windows)
* **CoppeliaSim EDU:** Plataforma Pioneer P3-DX (`/myRobot`), atuadores `/leftMotor` e `/rightMotor`, LiDAR 2D e Vision Sensor frontal em cena customizada.
* **Plugin `simZMQ` (Lua):** Servidor ZMQ customizado com socket `REP` (porta `23005` para malhas da cena) e socket `PUB` (porta `23006` para streaming de poses móveis).

### Robótica, Autonomia e Middleware (WSL2 / Linux)
* **ROS 2 (Jazzy):** Middleware de robótica principal.
* **`slam_toolbox`:** Construção dinâmica do mapa de ocupação 2D (`/map`) em tempo real.
* **`Nav2` (Navigation2):** Pilha completa com suporte a `NavigateToPose`, `NavigateThroughPoses` e `nav2_costmap_2d::CostmapFilter` (Keepout Zones).
* **`twist_mux`:** Multiplexador de velocidade priorizando a teleoperação manual (`/cmd_vel_teleop`) sobre o planejador autônomo (`/cmd_vel_nav`).
* **`ROS-TCP-Endpoint`:** Ponte de comunicação bidirecional TCP/IP com o Unity na porta `10000`.

### Realidade Virtual e HMI (Meta Quest 3 / Unity)
* **Unity Engine (C#):** Motor gráfico para renderização imersiva no headset.
* **XR Interaction Toolkit:** Locomoção contínua (*Continuous Move*), emissão de raios (*Raycast*) e manipulação em duas mãos da mesa tática (*XRGrabInteractable*).
* **NetMQ (C#):** Cliente ZeroMQ para consumo da geometria da cena e poses em tempo real.
* **ROS-TCP-Connector (C#):** Cliente oficial Unity Robotics Hub para publicação e assinatura de tópicos e ações do ROS 2.

---

## 6. Etapas de Desenvolvimento

### Etapa 1: Ponte ZMQ e Reconstrução Dinâmica do Pioneer P3-DX (CoppeliaSim $\rightarrow$ Unity)
* Criar o script Lua (`simZMQ`) no CoppeliaSim com socket `REP` (`23005`) e `PUB` (`23006`).
* Criar os scripts C# no Unity para instanciar a geometria da cena e do Pioneer P3-DX em tempo de execução via `new Mesh()`, atualizando as poses dinamicamente na Main Thread com conversão determinística de eixos.

### Etapa 2: Scripts de Topologia de Rede Híbrida (Windows $\leftrightarrow$ WSL2 $\leftrightarrow$ Quest 3 via ADB)
* Script PowerShell `setup_adb_reverse.ps1` mapeando as portas `23005`, `23006` e `10000` via USB-C.
* Utilitário de detecção dinâmica de IP do host Windows para configuração dos nós no WSL2.

### Etapa 3: Base Robótica do Pioneer P3-DX no WSL2 (Ponte ROS2-ZMQ, `slam_toolbox`, `Nav2` e Keepout Filters)
* Unificar os nós de LiDAR, odometria Ground Truth, TF, cinemática diferencial ($r=0.0975, L=0.331$) e câmera frontal em `coppelia_bridge_node.py` (com suporte a `mock_mode:=true` para testes 100% offline).
* Configurar o `slam_toolbox` em modo *online async* e o `Nav2` com suporte ativado a **Costmap Keepout Filters** (`/keepout_filter_mask`).

### Etapa 4: Locomoção VR e Menu Espacial de Controle
* Configurar o *Continuous Move Provider* no `XR Origin` para deslocamento pelo cenário.
* Desenvolver o **Menu Espacial no VR** acessível pelo controle, contendo botões para:
  * Abrir/Fechar a **Mesa Tática Holográfica**.
  * Selecionar Ferramenta: **Point-and-Click**, **Path Painting**, **Pintura de No-Go Zones**.
  * Alternar para o **Modo Teleoperação em 1ª Pessoa**.
  * Controle de Missão: **Pausar**, **Retomar** e **Cancelar**.

### Etapa 5: Mesa Tática Holográfica Sob Demanda e Navegação Multimodal
* **Mesa Tática Holográfica (`HolographicTacticalTable.cs`):** Objeto flutuante invocado pelo menu, manipulável com as mãos/controles (`XRGrabInteractable`), renderizando a malha do `/map` e o mini-avatar do Pioneer P3-DX.
* **Point-and-Click:** Clique com laser na mesa converte UV para $(X, Y)$ do mapa e dispara a meta.
* **Path Painting (`PathPainter.cs`):** Desenho contínuo de caminho no minimapa amostrado em waypoints para a Action `NavigateThroughPoses`.
* **Pintura de No-Go Zones (`NoGoZonePainter.cs`):** Pincel virtual no minimapa gerando máscara binária enviada para o Nav2 Costmap Filter (`/keepout_filter_mask`).
* **Nó Supervisor de Missão:** Gerencia as ações `NavigateToPose` e `NavigateThroughPoses`, com comandos de `PAUSE`, `RESUME` e `CANCEL`.

### Etapa 6: Teleoperação em 1ª Pessoa Sob Demanda, `twist_mux` e Guias de Montagem
* Configurar `twist_mux` priorizando `/cmd_vel_teleop` sobre `/cmd_vel_nav`.
* Criar controlador no Unity com monitor flutuante da câmera frontal do Pioneer P3-DX (`/camera/image_raw` sob demanda) e controle analógico do robô via joystick.
* Fornecer roteiros detalhados de montagem no Unity e no CoppeliaSim, sincronizados automaticamente para o diretório Windows via `sync_to_windows.sh`.
