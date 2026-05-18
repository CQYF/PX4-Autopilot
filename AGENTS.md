# PX4 Autopilot Developer Guide

## Build Commands

```bash
# Build SITL (Software In The Loop) - default target
make px4_sitl_default

# Build for specific board (e.g., fmu-v5)
make px4_fmu-v5_default

# Build for Matek H743
make matek_h743_dummy

# Build with sanitizers
PX4_ASAN=1 make px4_sitl_default    # AddressSanitizer
PX4_MSAN=1 make px4_sitl_default    # MemorySanitizer
PX4_UBSAN=1 make px4_sitl_default   # UndefinedBehaviorSanitizer
```

## Testing

```bash
# Unit tests (runs in SITL test config)
make tests

# Run specific test (via TESTFILTER)
TESTFILTER=test_name make tests

# Full integration tests (requires ROS, Gazebo)
make tests_integration

# SITL tests with Gazebo
make sitl_gazebo-classic
```

## Code Quality

```bash
# Format code
make format

# Check formatting
make check_format

# Clang-tidy (requires clang)
make clang-tidy-quiet
```

## Key Directories

- `src/modules/` - Flight applications (navigator, estimator, controllers)
- `src/drivers/` - Hardware drivers
- `src/lib/` - Shared libraries
- `boards/` - Board-specific configurations
- `Tools/simulation/` - Simulation backends (Gazebo, jMAVSim, FlightGear)

## Important Notes

- **Submodules**: Repository uses git submodules. Initialize with `git submodule update --init --recursive`
- **Ninja**: Build uses Ninja by default if available; falls back to Makefiles
- **Build artifacts**: Output in `build/<config>/` (e.g., `build/px4_sitl_default/`)
- **GTest**: Unit tests use Google Test framework; tests live alongside source in `test/` subdirectories within modules
- **Code style**: Enforced via astyle; CI runs clang-tidy checks
- **Configuration**: Uses Kconfig for board configuration (see `boards/*/*.px4board` files)
- **Board-specific builds**: If you create new modules, you MUST enable them in `boards/<board>/<board>.px4board` via Kconfig, otherwise they will NOT be compiled. Example: `make matek_h743_dummy` requires enabling modules in `boards/matek_h743/dummy.px4board`.

## New Module / Feature Implementation Checklist

When adding a new module or feature that requires a new uORB message, navigation state, or parameter:

1. **uORB Message**: Define in `msg/*.msg`, auto-generates headers
2. **Navigation State**: Add to `msg/VehicleStatus.msg` + `commander/px4_custom_mode.h`
3. **New Parameters**: Define param macros in the module's header/params file
4. **Board Configuration**: If creating a new module, enable in `boards/<target>/<target>.px4board` (CONFIG_MODULES_*)
5. **Build**: `make matek_h743_dummy` (or your target)
6. **Verify**: Check `build/<target>/.../CMakeCache.txt` or `make verbose <target>` for module inclusion

## CI Targets

- `check` - Full CI check (SITL + NuttX builds + tests + format)
- `quick_check` - Single NuttX + SITL + tests + format