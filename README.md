# IA368 - Projeto Final: Interface Humano-Máquina em Realidade Mista e Gêmeo Digital para Robô Móvel

**Disciplina:** Design e Desenvolvimento de Robôs Autônomos Inteligentes (IA368) — Faculdade de Engenharia Elétrica e de Computação (FEEC / Unicamp)  
**Autor:** Mateus Henrique Silva Araújo  

---

## 📌 Visão Geral

Desenvolvimento de uma **Interface Humano-Máquina (HMI) imersiva em Realidade Virtual/Mista (Meta Quest 3, via Unity)** para o planejamento tático, monitoramento e teleoperação em primeira pessoa de um **robô móvel diferencial Pioneer P3-DX** simulado no **CoppeliaSim EDU**, integrado à pilha **ROS 2 (Jazzy)** e **Nav2**.

O sistema opera sobre uma arquitetura de Gêmeo Digital dividida em duas camadas desacopladas:
1. **Camada de Mundo Físico Simulado (ZeroMQ):** Reconstrói proceduralmente a cena doméstica ([`scene_house.ttt`](scene_house/scene_house.ttt)) no Unity e sincroniza poses dos corpos dinâmicos em tempo real.
2. **Camada de HMI e Controle em Realidade Mista (ROS 2):** Menu Espacial VR sob demanda com **Mesa Tática Holográfica** (minimapa com *Point-and-Click*, *Path Painting* para `NavigateThroughPoses` e *Pintura de No-Go Zones* para Costmap Keepout Filter) e **Teleoperação em 1ª Pessoa** (`/camera/image_raw` + joystick com prioridade via `twist_mux`).

---

## 📁 Estrutura do Repositório

```text
├── PROPOSTA_PROJETO.md          # Proposta técnica completa aprovada
├── PLANO_IMPLEMENTACAO.md       # Plano de implementação detalhado passo a passo
├── scene_house/
│   └── scene_house.ttt          # Cena base do ambiente residencial no CoppeliaSim
├── coppelia_scripts/            # Scripts Lua para streaming de cena e poses no CoppeliaSim
├── unity_scripts/               # Scripts C# para Unity (Mundo Simulado e HMI ROS 2)
│   ├── Layer1_SimulatedWorld_ZMQ/
│   └── Layer2_ARInterface_ROS2/
├── ros2_ws/                     # Workspace ROS 2 Jazzy (vr_robot_pkg e ROS-TCP-Endpoint)
├── scripts/                     # Scripts de automação (sincronização Windows/WSL2, adb reverse)
└── tests/                       # Testes automatizados offline em Python (pytest)
```

Para detalhes da arquitetura, topologia de rede e planejamento de etapas, consulte:
* [Proposta de Projeto](PROPOSTA_PROJETO.md)
* [Plano de Implementação](PLANO_IMPLEMENTACAO.md)
