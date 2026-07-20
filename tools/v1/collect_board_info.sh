#!/usr/bin/env bash
# VisionArm BallTrack - V1 read-only board environment collector
# This script does not install packages, change system configuration, load/unload
# drivers, open camera devices, transmit on UART/CAN, or run an RKNN model.

set -u
set -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." >/dev/null 2>&1 && pwd -P)"
LOG_DIR="/mnt/nfs/visionarm_logs/v1_env"
START_STAMP="$(date '+%Y%m%d_%H%M%S' 2>/dev/null || printf 'unknown_time')"
SESSION_LOG="${LOG_DIR}/collect_board_info_${START_STAMP}.log"

mkdir -p -- "${LOG_DIR}" || {
    printf 'ERROR: cannot create log directory: %s\n' "${LOG_DIR}" >&2
    exit 1
}

# Mirror all console output to a session log. Individual evidence files are also
# written below. The active session log is excluded from manifest.sha256 because
# it remains open until script exit.
exec > >(tee -a "${SESSION_LOG}") 2>&1

now_iso() {
    date --iso-8601=seconds 2>/dev/null || date '+%Y-%m-%dT%H:%M:%S%z' 2>/dev/null || date
}

banner() {
    printf '\n================================================================================\n'
    printf '%s\n' "$1"
    printf '================================================================================\n'
}

note() {
    printf '[INFO] %s\n' "$*"
}

warn() {
    printf '[WARN] %s\n' "$*"
}

run_cmd() {
    local description="$1"
    shift

    printf '\n--- %s ---\n' "${description}"
    printf '$'
    printf ' %q' "$@"
    printf '\n'

    "$@"
    local rc=$?
    if (( rc == 0 )); then
        printf '[EXIT] 0\n'
    else
        printf '[EXIT] %d (continued; no system changes were attempted)\n' "${rc}"
    fi
    return 0
}

run_if_command() {
    local command_name="$1"
    local description="$2"
    shift 2

    if command -v "${command_name}" >/dev/null 2>&1; then
        run_cmd "${description}" "$@"
    else
        printf '\n--- %s ---\n' "${description}"
        printf 'MISSING: command not found: %s\n' "${command_name}"
    fi
}

read_text_file() {
    local path="$1"
    local description="$2"

    printf '\n--- %s ---\n' "${description}"
    printf 'PATH: %s\n' "${path}"
    if [[ -r "${path}" ]]; then
        cat -- "${path}"
        printf '\n'
    elif [[ -e "${path}" ]]; then
        printf 'BLOCKED: exists but is not readable by user %s\n' "$(id -un 2>/dev/null || printf UNKNOWN)"
    else
        printf 'MISSING: path does not exist\n'
    fi
}

read_dt_string() {
    local path="$1"
    local description="$2"

    printf '\n--- %s ---\n' "${description}"
    printf 'PATH: %s\n' "${path}"
    if [[ -r "${path}" ]]; then
        tr '\000' '\n' < "${path}"
    elif [[ -e "${path}" ]]; then
        printf 'BLOCKED: exists but is not readable\n'
    else
        printf 'MISSING\n'
    fi
}

write_block() {
    local outfile="$1"
    local title="$2"
    local collector="$3"

    {
        banner "${title}"
        "${collector}"
    } 2>&1 | tee "${outfile}"
    local rc=${PIPESTATUS[0]}
    if (( rc != 0 )); then
        warn "collector ${collector} returned ${rc}; script continues"
    fi
}

collect_session_metadata() {
    printf 'project=VisionArm BallTrack\n'
    printf 'stage=V1 RK3588 environment baseline\n'
    printf 'collector_version=1.0\n'
    printf 'start_time=%s\n' "${START_TIME_ISO}"
    printf 'execution_user=%s\n' "$(id -un 2>/dev/null || printf UNKNOWN)"
    printf 'uid_gid=%s\n' "$(id 2>/dev/null || printf UNKNOWN)"
    printf 'hostname=%s\n' "$(hostname 2>/dev/null || printf UNKNOWN)"
    printf 'pwd=%s\n' "$(pwd -P 2>/dev/null || pwd)"
    printf 'script_path=%s\n' "${BASH_SOURCE[0]}"
    printf 'script_dir=%s\n' "${SCRIPT_DIR}"
    printf 'repo_root=%s\n' "${REPO_ROOT}"
    printf 'log_dir=%s\n' "${LOG_DIR}"
    printf 'sudo_used=NO\n'
    printf 'system_modification=NO (repository log files only)\n'
}

collect_board_identity() {
    read_dt_string /proc/device-tree/model 'Device Tree model'
    read_dt_string /proc/device-tree/compatible 'Device Tree compatible list'
    read_dt_string /proc/device-tree/serial-number 'Device Tree serial-number'
    read_text_file /proc/sys/kernel/hostname 'Kernel hostname'
    read_text_file /proc/cmdline 'Kernel command line'
    run_if_command uname 'uname -a' uname -a

    if command -v dmesg >/dev/null 2>&1; then
        printf '\n--- Board/SoC evidence filtered from dmesg ---\n'
        dmesg 2>&1 | grep -Eai 'machine model|rockchip|rk3588|rk3588s|soc[ :]|u-boot|uboot|bootloader' | head -n 240
        local rc=${PIPESTATUS[0]}
        if (( rc != 0 )); then
            printf 'WARN: dmesg unavailable, restricted, or no matching lines; see dmesg.log\n'
        fi
    else
        printf '\nMISSING: dmesg command not found\n'
    fi

    printf '\n--- Candidate board identity files (bounded) ---\n'
    for path in \
        /sys/firmware/devicetree/base/model \
        /sys/firmware/devicetree/base/compatible \
        /sys/firmware/devicetree/base/serial-number \
        /sys/devices/soc0/machine \
        /sys/devices/soc0/family \
        /sys/devices/soc0/soc_id \
        /sys/devices/soc0/revision; do
        if [[ -r "${path}" ]]; then
            printf '%s: ' "${path}"
            tr '\000' ' ' < "${path}"
            printf '\n'
        elif [[ -e "${path}" ]]; then
            printf '%s: BLOCKED (not readable)\n' "${path}"
        else
            printf '%s: MISSING\n' "${path}"
        fi
    done
}

collect_device_tree() {
    read_dt_string /proc/device-tree/model 'model'
    read_dt_string /proc/device-tree/compatible 'compatible'
    read_dt_string /proc/device-tree/chosen/bootargs 'chosen/bootargs'
    read_dt_string /proc/device-tree/serial-number 'serial-number'

    printf '\n--- Device Tree top-level entries (maximum 240) ---\n'
    if [[ -d /proc/device-tree ]]; then
        find /proc/device-tree -mindepth 1 -maxdepth 2 -printf '%y %p\n' 2>&1 | sort | head -n 240
    else
        printf 'MISSING: /proc/device-tree\n'
    fi

    printf '\n--- Relevant Device Tree node names (bounded depth and result count) ---\n'
    if [[ -d /proc/device-tree ]]; then
        find /proc/device-tree -mindepth 1 -maxdepth 6 -type d -print 2>/dev/null \
            | grep -Eai '/(npu|rknpu|gpu|vop|isp|csi|mipi|camera|video|uart|serial|can|ethernet|gmac|pcie|usb)(@|/|$)' \
            | sort | head -n 300
        local rc=${PIPESTATUS[1]}
        if (( rc != 0 )); then
            printf 'INFO: no matching Device Tree node names found, or access was restricted\n'
        fi
    else
        printf 'MISSING: /proc/device-tree\n'
    fi
}

collect_uname() {
    run_if_command uname 'uname -a' uname -a
    run_if_command uname 'uname -srvm' uname -srvm
    run_if_command uname 'uname -m' uname -m
    read_text_file /proc/version '/proc/version'
    read_text_file /proc/cmdline '/proc/cmdline'
}

collect_os_release() {
    read_text_file /etc/os-release '/etc/os-release'
    read_text_file /usr/lib/os-release '/usr/lib/os-release (fallback/vendor copy)'
    run_if_command lsb_release 'lsb_release -a' lsb_release -a
    run_if_command systemctl 'systemd version' systemctl --version

    printf '\n--- PID 1 identity ---\n'
    if [[ -r /proc/1/comm ]]; then
        cat /proc/1/comm
    else
        printf 'BLOCKED or MISSING: /proc/1/comm\n'
    fi
    if [[ -r /proc/1/cmdline ]]; then
        tr '\000' ' ' < /proc/1/cmdline
        printf '\n'
    else
        printf 'BLOCKED or MISSING: /proc/1/cmdline\n'
    fi
}

collect_cpuinfo() {
    run_if_command lscpu 'lscpu' lscpu
    read_text_file /proc/cpuinfo '/proc/cpuinfo'
    run_if_command getconf 'getconf LONG_BIT' getconf LONG_BIT
    run_if_command getconf 'getconf GNU_LIBC_VERSION' getconf GNU_LIBC_VERSION

    printf '\n--- Package-manager architecture evidence ---\n'
    if command -v dpkg >/dev/null 2>&1; then
        run_cmd 'dpkg --print-architecture' dpkg --print-architecture
        run_cmd 'dpkg --print-foreign-architectures' dpkg --print-foreign-architectures
    elif command -v rpm >/dev/null 2>&1; then
        run_cmd 'rpm architecture query' rpm --eval '%{_arch}'
    elif command -v apk >/dev/null 2>&1; then
        run_cmd 'apk architecture query' apk --print-arch
    else
        printf 'MISSING: no dpkg/rpm/apk architecture query available\n'
    fi

    printf '\n--- libc / loader evidence ---\n'
    if command -v ldd >/dev/null 2>&1; then
        ldd --version 2>&1 | head -n 12
    else
        printf 'MISSING: ldd\n'
    fi
    if command -v file >/dev/null 2>&1; then
        file /bin/sh 2>&1
        [[ -e /bin/bash ]] && file /bin/bash 2>&1
    else
        printf 'MISSING: file command\n'
    fi
    for path in /lib/ld-linux-aarch64.so.1 /lib64/ld-linux-aarch64.so.1 /lib/aarch64-linux-gnu/ld-linux-aarch64.so.1; do
        if [[ -e "${path}" ]]; then
            ls -l -- "${path}"
            readlink -f -- "${path}" 2>/dev/null || true
        fi
    done
}

collect_memory_storage() {
    run_if_command free 'free -h' free -h
    read_text_file /proc/meminfo '/proc/meminfo'
    run_if_command lsblk 'lsblk inventory (bytes)' lsblk -b -o NAME,KNAME,PATH,TYPE,SIZE,RO,RM,FSTYPE,FSVER,LABEL,UUID,PARTUUID,MOUNTPOINTS,MODEL,SERIAL,TRAN
    run_if_command df 'df -hT' df -hT
    run_if_command findmnt 'findmnt filesystem tree' findmnt -o TARGET,SOURCE,FSTYPE,OPTIONS,SIZE,USED,AVAIL

    printf '\n--- MMC/eMMC sysfs identity (bounded) ---\n'
    local found=0
    local devbase path
    for devbase in /sys/block/mmcblk*; do
        [[ -e "${devbase}" ]] || continue
        found=1
        printf '\n[%s]\n' "${devbase}"
        for path in \
            "${devbase}/device/type" \
            "${devbase}/device/name" \
            "${devbase}/device/manfid" \
            "${devbase}/device/oemid" \
            "${devbase}/device/cid" \
            "${devbase}/device/csd" \
            "${devbase}/device/date" \
            "${devbase}/device/rev" \
            "${devbase}/size" \
            "${devbase}/ro"; do
            if [[ -r "${path}" ]]; then
                printf '%s=' "${path##*/}"
                cat -- "${path}"
            elif [[ -e "${path}" ]]; then
                printf '%s=BLOCKED\n' "${path##*/}"
            else
                printf '%s=MISSING\n' "${path##*/}"
            fi
        done
    done
    (( found == 1 )) || printf 'MISSING: no /sys/block/mmcblk* entries\n'
}

collect_partition_map() {
    run_if_command lsblk 'lsblk partition map' lsblk -o NAME,MAJ:MIN,SIZE,TYPE,FSTYPE,FSVER,LABEL,UUID,PARTUUID,PARTLABEL,MOUNTPOINTS
    read_text_file /proc/partitions '/proc/partitions'
    run_if_command findmnt 'findmnt -A' findmnt -A
    run_if_command mount 'mount output' mount
}

collect_network() {
    if command -v ip >/dev/null 2>&1; then
        run_cmd 'ip -details link show' ip -details link show
        run_cmd 'ip address show' ip address show
        run_cmd 'ip route show table all' ip route show table all
        run_cmd 'ip -6 route show table all' ip -6 route show table all
    else
        printf 'MISSING: ip command\n'
        run_if_command ifconfig 'ifconfig -a fallback' ifconfig -a
        run_if_command route 'route -n fallback' route -n
    fi
    read_text_file /etc/resolv.conf '/etc/resolv.conf'
    read_text_file /proc/net/dev '/proc/net/dev'
}

collect_device_nodes() {
    printf '%s\n' 'NOTE: device nodes are only listed; none are opened.'

    printf '\n--- Video nodes ---\n'
    compgen -G '/dev/video*' >/dev/null 2>&1 && ls -l /dev/video* 2>&1 || printf 'MISSING: /dev/video*\n'
    if [[ -d /sys/class/video4linux ]]; then
        find /sys/class/video4linux -mindepth 1 -maxdepth 1 -printf '%f -> %l\n' 2>&1 | sort | head -n 120
    else
        printf 'MISSING: /sys/class/video4linux\n'
    fi

    printf '\n--- UART/serial candidate nodes ---\n'
    local pattern matched=0
    for pattern in /dev/ttyS* /dev/ttyFIQ* /dev/ttyUSB* /dev/ttyACM* /dev/ttyAMA*; do
        if compgen -G "${pattern}" >/dev/null 2>&1; then
            matched=1
            # shellcheck disable=SC2086
            ls -l ${pattern} 2>&1
        fi
    done
    (( matched == 1 )) || printf 'MISSING: no common UART/serial device nodes found\n'

    printf '\n--- /proc/tty/driver/serial (read-only) ---\n'
    if [[ -r /proc/tty/driver/serial ]]; then
        cat /proc/tty/driver/serial
    elif [[ -e /proc/tty/driver/serial ]]; then
        printf 'BLOCKED: permission denied for current user\n'
    else
        printf 'MISSING\n'
    fi

    printf '\n--- CAN network interfaces ---\n'
    if command -v ip >/dev/null 2>&1; then
        ip -details link show type can 2>&1 || printf 'INFO: no CAN interface reported, or command unsupported\n'
    else
        printf 'MISSING: ip command\n'
    fi

    printf '\n--- NPU/media candidate nodes ---\n'
    matched=0
    for pattern in /dev/rknpu* /dev/npu* /dev/mpp_service /dev/dri/*; do
        if compgen -G "${pattern}" >/dev/null 2>&1; then
            matched=1
            # shellcheck disable=SC2086
            ls -l ${pattern} 2>&1
        fi
    done
    (( matched == 1 )) || printf 'MISSING: no common NPU/media nodes found\n'

    printf '\n--- TTY sysfs names (maximum 240) ---\n'
    if [[ -d /sys/class/tty ]]; then
        find /sys/class/tty -mindepth 1 -maxdepth 1 -printf '%f -> %l\n' 2>&1 | sort | head -n 240
    else
        printf 'MISSING: /sys/class/tty\n'
    fi
}

collect_thermal_power() {
    printf '%s\n' 'NOTE: only current values are read; governors and frequencies are not changed.'

    printf '\n--- Thermal zones (maximum 64) ---\n'
    local zone count=0 path
    for zone in /sys/class/thermal/thermal_zone*; do
        [[ -d "${zone}" ]] || continue
        count=$((count + 1))
        (( count > 64 )) && break
        printf '\n[%s]\n' "${zone}"
        for path in "${zone}/type" "${zone}/temp" "${zone}/policy"; do
            if [[ -r "${path}" ]]; then
                printf '%s=' "${path##*/}"
                cat -- "${path}"
            elif [[ -e "${path}" ]]; then
                printf '%s=BLOCKED\n' "${path##*/}"
            else
                printf '%s=MISSING\n' "${path##*/}"
            fi
        done
    done
    (( count > 0 )) || printf 'MISSING: no thermal_zone entries\n'

    printf '\n--- CPU frequency policies (maximum 32) ---\n'
    count=0
    local policy
    for policy in /sys/devices/system/cpu/cpufreq/policy*; do
        [[ -d "${policy}" ]] || continue
        count=$((count + 1))
        (( count > 32 )) && break
        printf '\n[%s]\n' "${policy}"
        for path in \
            "${policy}/affected_cpus" \
            "${policy}/scaling_driver" \
            "${policy}/scaling_governor" \
            "${policy}/scaling_cur_freq" \
            "${policy}/cpuinfo_min_freq" \
            "${policy}/cpuinfo_max_freq"; do
            if [[ -r "${path}" ]]; then
                printf '%s=' "${path##*/}"
                cat -- "${path}"
            elif [[ -e "${path}" ]]; then
                printf '%s=BLOCKED\n' "${path##*/}"
            else
                printf '%s=MISSING\n' "${path##*/}"
            fi
        done
    done
    (( count > 0 )) || printf 'MISSING: no cpufreq policy entries\n'
}

print_tool() {
    local tool="$1"
    shift
    printf '\n--- %s ---\n' "${tool}"
    if command -v "${tool}" >/dev/null 2>&1; then
        printf 'path=%s\n' "$(command -v "${tool}")"
        "$@" 2>&1 | head -n 30
        local rc=${PIPESTATUS[0]}
        printf 'exit=%d\n' "${rc}"
    else
        printf 'MISSING\n'
    fi
}

collect_tool_versions() {
    print_tool gcc gcc --version
    if command -v gcc >/dev/null 2>&1; then
        run_cmd 'gcc target triple' gcc -dumpmachine
        run_cmd 'gcc version token' gcc -dumpfullversion -dumpversion
    fi

    print_tool g++ g++ --version
    if command -v g++ >/dev/null 2>&1; then
        run_cmd 'g++ target triple' g++ -dumpmachine
        run_cmd 'g++ version token' g++ -dumpfullversion -dumpversion
    fi

    print_tool cc cc --version
    print_tool c++ c++ --version
    print_tool make make --version
    print_tool cmake cmake --version
    print_tool pkg-config pkg-config --version
    print_tool ld ld --version
    print_tool as as --version
    print_tool ar ar --version
    print_tool readelf readelf --version
    print_tool objdump objdump --version
    print_tool file file --version
    print_tool git git --version
}

collect_dmesg() {
    if command -v dmesg >/dev/null 2>&1; then
        dmesg 2>&1
        local rc=$?
        if (( rc != 0 )); then
            printf '\nBLOCKED: dmesg failed for current user. Do not use sudo in this step; preserve this evidence.\n'
        fi
    else
        printf 'MISSING: dmesg command not found\n'
    fi
}

collect_journal_boot() {
    if command -v journalctl >/dev/null 2>&1; then
        journalctl -b --no-pager 2>&1
        local rc=$?
        if (( rc != 0 )); then
            printf '\nBLOCKED: journalctl -b failed for current user, or systemd journal is unavailable.\n'
        fi
    else
        printf 'NOT_APPLICABLE: journalctl command not found\n'
    fi
}

collect_boot_full_placeholder() {
    printf 'MISSING: complete external serial boot log was not captured by Step A.\n'
    printf 'This file is a placeholder only and MUST NOT be described as a complete boot log.\n'
    printf 'A complete log, when collected in Step C, should begin before power-on and may include U-Boot, kernel, init/systemd, and login prompt.\n'
}

collect_rknpu_driver() {
    printf '%s\n' 'NOTE: no module is loaded, unloaded, or modified by this collector.'

    run_if_command lsmod 'Loaded modules filtered for RKNPU/NPU' bash -c "lsmod 2>&1 | grep -Eai '(^|[[:space:]])(rknpu|npu)([[:space:]]|$)' || true"

    printf '\n--- Matching /sys/module entries ---\n'
    if [[ -d /sys/module ]]; then
        find /sys/module -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null \
            | grep -Eai 'rknpu|(^|_)npu($|_)' | sort | head -n 120 || true
    else
        printf 'MISSING: /sys/module\n'
    fi

    if command -v modinfo >/dev/null 2>&1; then
        run_cmd 'modinfo rknpu (metadata only)' modinfo rknpu
    else
        printf '\nMISSING: modinfo command\n'
    fi

    printf '\n--- Kernel configuration evidence for RKNPU/NPU ---\n'
    if [[ -r /proc/config.gz ]] && command -v zcat >/dev/null 2>&1; then
        zcat /proc/config.gz 2>/dev/null | grep -Eai 'CONFIG_.*(RKNPU|NPU)' || printf 'INFO: no matching CONFIG entries in /proc/config.gz\n'
    elif [[ -r "/boot/config-$(uname -r 2>/dev/null)" ]]; then
        grep -Eai 'CONFIG_.*(RKNPU|NPU)' "/boot/config-$(uname -r 2>/dev/null)" || printf 'INFO: no matching CONFIG entries in /boot config\n'
    else
        printf 'UNKNOWN: readable kernel configuration not available\n'
    fi

    printf '\n--- Matching platform-driver entries ---\n'
    if [[ -d /sys/bus/platform/drivers ]]; then
        find /sys/bus/platform/drivers -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null \
            | grep -Eai 'rknpu|(^|[-_])npu($|[-_])' | sort | head -n 120 || true
    else
        printf 'MISSING: /sys/bus/platform/drivers\n'
    fi

    printf '\n--- NPU/RKNPU kernel log lines (maximum 400) ---\n'
    if command -v dmesg >/dev/null 2>&1; then
        dmesg 2>&1 | grep -Eai 'rknpu|\bnpu\b|iommu.*npu|npu.*iommu' | head -n 400
        local rc=${PIPESTATUS[0]}
        if (( rc != 0 )); then
            printf 'WARN: dmesg unavailable/restricted, or no matching NPU lines found\n'
        fi
    else
        printf 'MISSING: dmesg command\n'
    fi

    printf '\n--- NPU candidate device nodes ---\n'
    local pattern matched=0
    for pattern in /dev/rknpu* /dev/npu* /dev/mpp_service; do
        if compgen -G "${pattern}" >/dev/null 2>&1; then
            matched=1
            # shellcheck disable=SC2086
            ls -l ${pattern} 2>&1
        fi
    done
    (( matched == 1 )) || printf 'MISSING: no common RKNPU/NPU device nodes\n'
}

inspect_rknn_path() {
    local path="$1"
    printf '\n[FOUND] %s\n' "${path}"
    ls -l -- "${path}" 2>&1 || true
    if command -v file >/dev/null 2>&1; then
        file -- "${path}" 2>&1 || true
    else
        printf 'MISSING: file command\n'
    fi
    if command -v sha256sum >/dev/null 2>&1 && [[ -f "${path}" ]]; then
        sha256sum -- "${path}" 2>&1 || true
    else
        printf 'MISSING or NOT_APPLICABLE: sha256sum unavailable or path is not a regular file\n'
    fi
    if command -v readelf >/dev/null 2>&1 && [[ -f "${path}" ]]; then
        readelf -h -- "${path}" 2>&1 | sed -n '1,40p' || true
    else
        printf 'MISSING or NOT_APPLICABLE: readelf unavailable or path is not a regular file\n'
    fi
}

collect_rknn_runtime() {
    printf '%s\n' 'NOTE: this collector does not execute rknn_server and does not load librknnrt.so.'

    printf '\n--- Dynamic linker cache entries ---\n'
    if command -v ldconfig >/dev/null 2>&1; then
        ldconfig -p 2>&1 | grep -Eai 'rknn|rknpu' || printf 'INFO: no RKNN/RKNPU entries in ldconfig cache\n'
    else
        printf 'MISSING: ldconfig command\n'
    fi

    printf '\n--- Bounded Runtime/server search roots ---\n'
    local roots=()
    local root
    for root in /usr /lib /lib64 /opt; do
        if [[ -d "${root}" ]]; then
            roots+=("${root}")
            printf 'SCAN_ROOT: %s\n' "${root}"
        else
            printf 'MISSING_ROOT: %s\n' "${root}"
        fi
    done

    local results_file
    results_file="$(mktemp "${LOG_DIR}/.rknn_paths.XXXXXX")" || {
        printf 'BLOCKED: cannot create temporary path list in %s\n' "${LOG_DIR}"
        return 0
    }

    if (( ${#roots[@]} > 0 )); then
        find "${roots[@]}" -xdev -type f \
            \( -name 'librknnrt.so' -o -name 'librknnrt.so.*' -o -name 'librknn_api.so' -o -name 'librknn_api.so.*' -o -name 'rknn_server' \) \
            -print 2>/dev/null | sort -u | head -n 240 > "${results_file}"
    fi

    if [[ -s "${results_file}" ]]; then
        while IFS= read -r path; do
            inspect_rknn_path "${path}"
        done < "${results_file}"
    else
        printf 'MISSING: no matching RKNN Runtime/server files found in bounded roots\n'
    fi
    rm -f -- "${results_file}"

    printf '\n--- rknn_server process/status evidence ---\n'
    if command -v ps >/dev/null 2>&1; then
        ps -ef 2>&1 | grep -E '[r]knn_server' || printf 'INFO: rknn_server process not observed\n'
    else
        printf 'MISSING: ps command\n'
    fi

    if command -v systemctl >/dev/null 2>&1; then
        systemctl status rknn_server --no-pager 2>&1 || printf 'INFO: rknn_server systemd unit absent, inactive, or not queryable\n'
    else
        printf 'NOT_APPLICABLE: systemctl command not found\n'
    fi
}

finalize_session_metadata() {
    {
        printf 'end_time=%s\n' "${END_TIME_ISO}"
        printf 'session_log=%s\n' "${SESSION_LOG}"
        printf 'manifest_note=active collect_board_info_*.log is excluded from manifest because it remains open until exit\n'
    } >> "${LOG_DIR}/session_metadata.txt"
}

generate_manifest() {
    local manifest="${LOG_DIR}/manifest.sha256"
    local temp="${LOG_DIR}/.manifest.sha256.tmp"

    if ! command -v sha256sum >/dev/null 2>&1; then
        printf 'MISSING: sha256sum command not found; manifest could not be generated\n' > "${manifest}"
        return 0
    fi

    : > "${temp}"
    while IFS= read -r -d '' path; do
        sha256sum -- "${path}" >> "${temp}" 2>&1 || true
    done < <(
        find "${LOG_DIR}" -maxdepth 1 -type f \
            ! -name 'manifest.sha256' \
            ! -name '.manifest.sha256.tmp' \
            ! -name '.rknn_paths.*' \
            ! -name 'collect_board_info_*.log' \
            -print0 2>/dev/null | sort -z
    )
    mv -f -- "${temp}" "${manifest}"
}

START_TIME_ISO="$(now_iso)"

banner 'VisionArm BallTrack - V1 Step A: read-only environment inventory'
note "Repository root: ${REPO_ROOT}"
note "Log directory: ${LOG_DIR}"
note 'No sudo is used. No package installation, reboot, library replacement, camera access, UART/CAN traffic, module change, or RKNN model execution is performed.'

write_block "${LOG_DIR}/session_metadata.txt" 'Session metadata' collect_session_metadata
write_block "${LOG_DIR}/board_identity.txt" 'Board identity evidence' collect_board_identity
write_block "${LOG_DIR}/device_tree.txt" 'Device Tree evidence' collect_device_tree
write_block "${LOG_DIR}/uname.txt" 'Kernel and uname evidence' collect_uname
write_block "${LOG_DIR}/os_release.txt" 'RootFS and init-system evidence' collect_os_release
write_block "${LOG_DIR}/cpuinfo.txt" 'CPU, userspace bitness, and libc evidence' collect_cpuinfo
write_block "${LOG_DIR}/memory_storage.txt" 'Memory and storage evidence' collect_memory_storage
write_block "${LOG_DIR}/partition_map.txt" 'Partition and mount evidence' collect_partition_map
write_block "${LOG_DIR}/network.txt" 'Network evidence' collect_network
write_block "${LOG_DIR}/device_nodes.txt" 'Device-node summary' collect_device_nodes
write_block "${LOG_DIR}/thermal_power.txt" 'Thermal and current power-policy evidence' collect_thermal_power
write_block "${LOG_DIR}/tool_versions.txt" 'Compiler and build-tool inventory' collect_tool_versions
write_block "${LOG_DIR}/dmesg.log" 'Raw dmesg output or permission evidence' collect_dmesg
write_block "${LOG_DIR}/journal_boot.log" 'Current boot journal or availability evidence' collect_journal_boot
write_block "${LOG_DIR}/boot_full.log" 'Complete serial boot log placeholder' collect_boot_full_placeholder
write_block "${LOG_DIR}/rknpu_driver.txt" 'RKNPU kernel-driver inventory' collect_rknpu_driver
write_block "${LOG_DIR}/rknn_runtime.txt" 'RKNN Runtime and rknn_server inventory' collect_rknn_runtime

END_TIME_ISO="$(now_iso)"
finalize_session_metadata
generate_manifest

banner 'Collection complete'
printf 'start_time=%s\n' "${START_TIME_ISO}"
printf 'end_time=%s\n' "${END_TIME_ISO}"
printf 'session_log=%s\n' "${SESSION_LOG}"
printf 'manifest=%s\n' "${LOG_DIR}/manifest.sha256"
printf 'Next action: review and paste the requested summaries; do not install or upgrade anything yet.\n'

exit 0
