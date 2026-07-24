colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# MVS ships an older libusb without libusb_set_option. If its directories are
# ahead of the system libraries, PCL IO fails before test_node reaches main().
# Filter MVS only for this child process so sourcing this script does not alter
# the caller's camera SDK environment.
(
    filtered_ld_library_path=""
    IFS=':' read -ra library_paths <<< "${LD_LIBRARY_PATH:-}"
    for library_path in "${library_paths[@]}"; do
        case "${library_path}" in
            /opt/MVS/lib/64|/opt/MVS/lib/32)
                continue
                ;;
        esac
        if [[ -n "${library_path}" ]]; then
            if [[ -n "${filtered_ld_library_path}" ]]; then
                filtered_ld_library_path+=":"
            fi
            filtered_ld_library_path+="${library_path}"
        fi
    done
    export LD_LIBRARY_PATH="${filtered_ld_library_path}"
    ros2 run relocation test_node "$@"
)
