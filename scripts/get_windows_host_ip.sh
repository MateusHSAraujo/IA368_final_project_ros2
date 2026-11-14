#!/usr/bin/env bash
# Prints the IPv4 address of the Windows host as seen from WSL2, where CoppeliaSim
# (ZMQ Remote API, port 23000) runs.
#
# Resolution order:
#   1. $COPPELIA_HOST, if set (manual override).
#   2. 127.0.0.1 when WSL runs in mirrored networking mode (Windows and WSL share localhost).
#   3. The default gateway of the WSL network (NAT mode): this is the Windows host.
#
# Usage:
#   get_windows_host_ip.sh                 # print the IP
#   get_windows_host_ip.sh --check [PORT]  # print the IP and test TCP reachability (default 23000)
#   get_windows_host_ip.sh --help
#
# Exit codes: 0 = OK, 1 = host IP not found, 2 = host found but PORT unreachable, 3 = bad usage.
# The Python equivalent used by the ROS nodes is vr_robot_pkg/network.py.
set -euo pipefail

usage() {
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
}

is_ipv4() {
    [[ "$1" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]
}

detect_host_ip() {
    if [[ -n "${COPPELIA_HOST:-}" ]]; then
        echo "${COPPELIA_HOST}"
        return 0
    fi
    if command -v wslinfo >/dev/null 2>&1; then
        local mode
        mode="$(wslinfo --networking-mode 2>/dev/null || true)"
        if [[ "${mode}" == "mirrored" ]]; then
            echo "127.0.0.1"
            return 0
        fi
    fi
    local gateway
    gateway="$(ip route show default 2>/dev/null | awk '/^default/ {print $3; exit}')"
    if [[ -n "${gateway}" ]] && is_ipv4 "${gateway}"; then
        echo "${gateway}"
        return 0
    fi
    return 1
}

check=false
port=23000
case "${1:-}" in
    "") ;;
    --check)
        check=true
        if [[ -n "${2:-}" ]]; then
            if [[ ! "$2" =~ ^[0-9]+$ ]]; then
                echo "Invalid port: $2" >&2
                exit 3
            fi
            port="$2"
        fi
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Unknown option: $1" >&2
        usage >&2
        exit 3
        ;;
esac

if ! host_ip="$(detect_host_ip)"; then
    echo "Could not determine the Windows host IP (set COPPELIA_HOST)." >&2
    exit 1
fi
echo "${host_ip}"

if [[ "${check}" == true ]]; then
    if timeout 2 bash -c "</dev/tcp/${host_ip}/${port}" 2>/dev/null; then
        echo "TCP ${host_ip}:${port} reachable" >&2
    else
        echo "TCP ${host_ip}:${port} NOT reachable (is CoppeliaSim running? Windows firewall?)" >&2
        exit 2
    fi
fi
