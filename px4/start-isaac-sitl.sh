#!/usr/bin/env bash
set -euo pipefail
px4_console_args=()
if [[ "${1:-}" == '--daemon' ]]; then
    px4_console_args=(-d)
elif [[ $# -ne 0 ]]; then
    echo 'Usage: start-isaac-sitl.sh [--daemon]' >&2
    exit 2
fi
# Keep Windows/Conda Python settings out of the Linux startup helpers.
unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
px4_repo=/home/finel/PX4-Autopilot
px4_binary="$px4_repo/build/px4_sitl_default/bin/px4"
bridge_project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
runtime_dir=/home/finel/.local/share/px4-isaac
if pgrep -x px4 >/dev/null; then
    echo 'PX4 is already running. Stop the existing SIH/PX4 instance before starting Isaac SITL.' >&2
    exit 1
fi
if [[ ! -x "$px4_binary" ]]; then
    echo "Build missing: $px4_binary" >&2
    exit 1
fi
mkdir -p "$runtime_dir/rootfs"
python3 - "$px4_repo" "$runtime_dir" "$bridge_project" <<'PY'
import pathlib, sys
repo, runtime, project = map(pathlib.Path, sys.argv[1:])
text = (repo / 'ROMFS/px4fmu_common/init.d-posix/rcS').read_text()
needle = '. px4-rc.simulator'
if text.count(needle) != 1:
    raise SystemExit('PX4 startup include changed; review before using the external simulator.')
text = text.replace(needle, '. ' + str(project / 'px4/simulator-udp.sh'))
(runtime / 'rcS-isaac').write_text(text)
PY
if [[ ! -e "$runtime_dir/rootfs/etc" ]]; then
    ln -s "$px4_repo/build/px4_sitl_default/etc" "$runtime_dir/rootfs/etc"
fi
windows_host=$(ip -4 route show default | awk 'NR==1 {print $3}')
export ROS_DOMAIN_ID=0
export PX4_PARAM_UXRCE_DDS_AG_IP
PX4_PARAM_UXRCE_DDS_AG_IP=$(python3 - "$windows_host" <<'PY'
import socket, struct, sys
print(struct.unpack('!i', socket.inet_aton(sys.argv[1]))[0])
PY
)
export PX4_SYS_AUTOSTART=10016
export PX4_SIM_MODEL=none_iris
export PX4_SIMULATOR=isaac
export PX4_PARAM_COM_RC_IN_MODE=4
export PX4_PARAM_COM_RCL_EXCEPT=4
export PX4_PARAM_MPC_THR_HOVER=0.5
export PX4_PARAM_THR_MDL_FAC=1.0
# Match the square NVIDIA quadcopter and motor order FR, RL, FL, RR (FRD).
export PX4_PARAM_CA_ROTOR0_PX=0.10253 PX4_PARAM_CA_ROTOR0_PY=0.10253 PX4_PARAM_CA_ROTOR0_KM=0.05
export PX4_PARAM_CA_ROTOR1_PX=-0.10253 PX4_PARAM_CA_ROTOR1_PY=-0.10253 PX4_PARAM_CA_ROTOR1_KM=0.05
export PX4_PARAM_CA_ROTOR2_PX=0.10253 PX4_PARAM_CA_ROTOR2_PY=-0.10253 PX4_PARAM_CA_ROTOR2_KM=-0.05
export PX4_PARAM_CA_ROTOR3_PX=-0.10253 PX4_PARAM_CA_ROTOR3_PY=0.10253 PX4_PARAM_CA_ROTOR3_KM=-0.05
cd "$runtime_dir/rootfs"
echo "Isaac PX4 SITL: waiting for Isaac sensor data on WSL UDP 14560; DDS agent $windows_host:8888."
exec "$px4_binary" "${px4_console_args[@]}" -s "$runtime_dir/rcS-isaac"
