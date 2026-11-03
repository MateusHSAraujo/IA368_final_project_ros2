#!/usr/bin/env bash
set -euo pipefail

WSL_ROOT="/home/mateus/ia368/projeto_final"
WIN_ROOT="/mnt/c/Users/mateu/unicamp/IA368/projeto_final"

mkdir -p "${WIN_ROOT}/Assets/Scripts/Layer1_SimulatedWorld_ZMQ" \
         "${WIN_ROOT}/Assets/Scripts/Layer2_ARInterface_ROS2" \
         "${WIN_ROOT}/coppelia_scripts" \
         "${WIN_ROOT}/scene_house" \
         "${WIN_ROOT}/scripts"

if [ -d "${WSL_ROOT}/unity_scripts" ]; then
    cp -r "${WSL_ROOT}/unity_scripts/"* "${WIN_ROOT}/Assets/Scripts/" 2>/dev/null || true
fi

if [ -d "${WSL_ROOT}/coppelia_scripts" ]; then
    cp -r "${WSL_ROOT}/coppelia_scripts/"* "${WIN_ROOT}/coppelia_scripts/" 2>/dev/null || true
fi

if [ -f "${WSL_ROOT}/PROPOSTA_PROJETO.md" ]; then
    cp -u "${WSL_ROOT}/PROPOSTA_PROJETO.md" "${WIN_ROOT}/" 2>/dev/null || true
    cp -u "${WSL_ROOT}/PROPOSTA_PROJETO.md" "${WIN_ROOT}/My project/" 2>/dev/null || true
fi

if [ -f "${WSL_ROOT}/PLANO_IMPLEMENTACAO.md" ]; then
    cp -u "${WSL_ROOT}/PLANO_IMPLEMENTACAO.md" "${WIN_ROOT}/" 2>/dev/null || true
    cp -u "${WSL_ROOT}/PLANO_IMPLEMENTACAO.md" "${WIN_ROOT}/My project/" 2>/dev/null || true
fi

if [ -f "${WSL_ROOT}/scene_house/scene_house.ttt" ]; then
    cp -u "${WSL_ROOT}/scene_house/scene_house.ttt" "${WIN_ROOT}/scene_house/" 2>/dev/null || true
    mkdir -p "${WIN_ROOT}/My project/scene_house"
    cp -u "${WSL_ROOT}/scene_house/scene_house.ttt" "${WIN_ROOT}/My project/scene_house/" 2>/dev/null || true
fi

if [ -d "${WSL_ROOT}/scripts" ]; then
    cp -r "${WSL_ROOT}/scripts/"*.ps1 "${WIN_ROOT}/scripts/" 2>/dev/null || true
fi

echo "Sincronizado com sucesso para ${WIN_ROOT} (C:\\Users\\mateu\\unicamp\\IA368\\projeto_final)"
