# softflight-c — Software-Rendered Flight Simulator in C

A software renderer flight simulator in C with a 6‑DOF rigid‑body flight dynamics model, ISA atmosphere,
and a scanline triangle rasterizer (perspective‑correct with Z‑buffer). Uses SDL2 **only** for window,
keyboard input, and timing. No GPU calls are used for drawing; all pixels are written by the CPU.

## Features
- 6‑DOF rigid body dynamics (quaternion state, RK4 integrator).
- ISA atmosphere (troposphere) for density/pressure vs altitude; simple wind model.
- Aerodynamics: lift/drag/moment from wing/tail surfaces, control inputs (elevator/aileron/rudder), ground effect approximation.
- Propulsion: simple prop thrust model vs throttle and airspeed.
- Software renderer:
  - 32‑bit ARGB framebuffer.
  - Z‑buffered, perspective‑correct triangle rasterization.
  - Backface culling, near/far plane clipping, viewport transform.
  - Gouraud shading; flat & wireframe debug modes.
  - Simple texture sampler (procedural checker/terrain color map).
- Camera: chase/ cockpit modes, basic HUD (airspeed, altitude, attitude lines).
- Terrain: procedural heightmap tiles around the aircraft with frustum culling.
- Fixed‑timestep simulation loop (120 Hz) with render interpolation; pause/slow‑mo for debugging.
- Deterministic replay from input log.
- Configurable via `softflight.ini`.
- Unit tests for core math, raster, and FDM components (CTests).

## Non‑Goals (to keep scope reasonable)
- ATC, FMC, radios, nav beacons (stubs only).
- Accurate aircraft‑specific tables and detailed engine model.
- Complex weather or ocean rendering.
- Advanced LOD terrain/meshing beyond a basic quad‑grid tiling.

## Build
Dependencies: C99 compiler, CMake (>=3.16), SDL2 dev headers.
- macOS (Homebrew): `brew install sdl2 cmake`
- Ubuntu/Debian: `sudo apt-get install libsdl2-dev cmake build-essential`
- Windows (MSYS2): `pacman -S mingw-w64-x86_64-SDL2 cmake`

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
./softflight
```

### Sanitizers (Debug)
```bash
cmake -DCMAKE_BUILD_TYPE=Debug -DSANITIZE=ON ..
```

## Controls
- **W/S**: pitch down/up (elevator)
- **A/D**: roll left/right (aileron)
- **Q/E**: yaw left/right (rudder)
- **R/F**: throttle up/down
- **C**: cycle camera (chase/cockpit/free)
- **P**: pause physics
- **1/2/3**: shading mode (flat/gouraud/wire)
- **ESC**: quit

## Files of interest
- `src/math.h`: vector/matrix/quaternion, AABB, utilities (header‑only).
- `src/fdm.c`: flight dynamics model: forces/moments, integrator, constraints.
- `src/raster.c`: triangle rasterization with perspective‑correct barycentric interpolation.
- `src/renderer.c`: camera, transforms, draw passes.
- `src/terrain.c`: procedural height + tiling, frustum culling.
- `src/main.c`: loop, input, configuration, glue code.

## Production notes
- The renderer is CPU‑bound by design. For large resolutions, consider enabling `-O3 -march=native`.
- Inner loops are branch‑reduced and use `restrict` pointers. For further gains, add SIMD paths (SSE2/NEON).
- The FDM is numerically stable at dt ≤ 1/240 s with RK4. We clamp large frame delays and sub‑step if needed.
- Use `--replay=...` to run a deterministic input log for reproducibility.

## License
MIT
