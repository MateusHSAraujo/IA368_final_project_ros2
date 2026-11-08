#!/usr/bin/env bash
# Copies the Windows-side helper scripts (scripts/*.ps1) from this WSL repository to the
# Windows master copy. Nothing else is copied: the Unity project, the CoppeliaSim scripts,
# the scenes and the planning documents are owned by the Windows repository
# (IA368_final_project) and must never be overwritten from here.
set -euo pipefail

WSL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WIN_ROOT="/mnt/c/Users/mateu/unicamp/IA368/projeto_final"

shopt -s nullglob
ps1_files=("${WSL_ROOT}/scripts/"*.ps1)

if [ ${#ps1_files[@]} -eq 0 ]; then
    echo "No scripts/*.ps1 to sync."
    exit 0
fi

mkdir -p "${WIN_ROOT}/scripts"
cp -u "${ps1_files[@]}" "${WIN_ROOT}/scripts/"
echo "Synced ${#ps1_files[@]} PowerShell script(s) to ${WIN_ROOT}/scripts"
