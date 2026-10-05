# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project purpose

This is an educational ball balancing platform (BBP) for engineering students. It teaches **computer vision** and **control**, not programming. Students get the physical platform and this software. Every design decision should keep student-facing code minimal and high-level, while the framework does the plumbing: motor drivers, camera I/O, buffering and timing.

Remote: https://github.com/vanvuurenwerner2305-blip/BallBalancingPlatform.git

## Core concept: projects

Students create **projects**. A project is a folder holding `project.json` (the Setup values), `tracking.cpp` and `control.cpp`. By default it lives under `~/Documents/BBP Projects/`.

1. **Setup** is a settings tab, not a script. Settings have two scopes, set by their group in `app/bbp_app/settings_schema.py`. **setup** settings describe the experiment and apply on the real platform too: target, artificial effects, camera fps and exposure, and control limits. They appear in the Setup tab. **model** settings exist only in the simulator: ball radius, mass and wall thickness (presets fill these in), contact, servos, plate, camera noise and lens, and gravity. They appear in the *Simulated hardware* dialog. `framework/src/settings.cpp` parses both, so keep the two files in sync.
2. **Tracking** implements `void track(const Image&, Detection&)`. It receives the camera image and reports the ball centre in pixels.
3. **Control** implements `void control(const Ball&, const Target&, Platform&)`. It reads the ball state, already converted to mm on the plate with history and filtered velocity, and sets `platform.angleX`/`angleY` in degrees. It never touches motors directly.

`framework/include/bbp/bbp.hpp` is the entire student-facing API, so it is the product. Keep it small, template-free and commented for non-programmers. The app's "library" panel (`app/bbp_app/widgets/api_reference.py`) documents the same API and must match it.

The student API is layered. It ranges from one-line helpers such as `pid()`, `.derivative()` and `.integral()` down to raw ball history for students who want to write everything themselves. Students work against a small, documented library of readable variables and a list of commands. Keep that API small, stable, free of templates and well documented, because it is the product students see.

## Decisions already made

- **Language:** the student scripts and the framework are C++. The UI is Python + PySide6, with VTK for 3D and pyqtgraph for plots.
- **Building student code:** in the simulator, student code is compiled locally with MSVC. A portable bundled compiler is planned later.
- **3D view:** it shows the real CAD meshes, not simplified shapes.
- **Hardware:** the target is a DFRobot **FireBeetle 2 ESP32-S3**.
- **Same code on both targets:** the identical student and framework code must run on the FireBeetle and in the simulator. Don't add student-facing behaviour that exists in only one target.
- **Timing in the simulator:** student CPU time is mimicked with a virtual clock (wall time × a calibrated factor). This is "option A", which the framework implements.
- **IP split:** the framework and simulator implementation is closed IP. The physics and modelling must be fully transparent and are documented in `docs/physics/`. When the simulator's physics changes, update the paper in the same change.

## Commands

The build uses MSVC with the CMake bundled in Visual Studio 2022. `cmake` is not on PATH, so use the full path or a Developer shell:

```sh
CM="/c/Program Files/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
"$CM" -S . -B build -G "Visual Studio 17 2022" -A x64
"$CM" --build build --config Release
./build/Release/bbp_tests.exe                 # all tests
./build/Release/bbp_tests.exe Ball_           # tests whose name contains "Ball_"
./build/Release/bbp_tests.exe --csv build/csv # also write figure data
./build/Release/bbp_sim_demo.exe build/demo   # closed loop through the simulated camera
sh tools/check_esp32_core.sh                  # cross-compile core/ for the ESP32-S3 (float, -Wdouble-promotion -Werror)
python tools/plot/make_figures.py build/csv build/demo docs/physics/figures
sh tools/build_docs.sh                        # physics paper + control/vision guides -> app/assets/docs/*.pdf
"/c/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" -c "exec(open('tools/cad/extract_geometry.py').read())"  # regenerate cad_geometry.hpp
```

`siunitx` is broken in this MiKTeX install, so the paper writes units with its own `\un{}` macro.

**The app** needs the `bbp` conda environment (conda-forge Qt/VTK). Don't run it with Anaconda base Python: its 2020 MSVC runtime makes PySide6 6.11 fail to import.

```sh
conda env create -f app/environment.yml            # once
app/run_app.bat                                     # or: <env>/python.exe app/run_app.py
powershell -ExecutionPolicy Bypass -File tools/install_shortcuts.ps1  # desktop + Start-menu shortcuts
<env>/python.exe tools/make_icon.py                 # regenerate app/assets/bbp.ico/.png
"/c/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" -c "exec(open('tools/cad/export_meshes.py').read())"  # regenerate app/assets/meshes
```

The app compiles projects against the libraries in `build/Release`, so build the C++ (Release) before using it. `bbp_app/__init__.py` strips other conda installs from `PATH`. Without that, base Anaconda's DLLs crash VTK on the first render with `0xc06d007f`.

## Architecture

- **`core/include/bbp/core/`**: header-only, templated on the scalar type. It runs as `float` on the ESP32-S3 and as `double` in the simulator. It has no heap, no exceptions and no OS calls.
  - `kinematics.hpp`: the 3-RRS mechanism. The inverse kinematics is closed form, with zero torsion and the parasitic translation computed analytically. The forward kinematics is a Newton solve with a warm start. It also provides the velocity Jacobian.
  - `camera_geometry.hpp`: the pinhole model with Brown–Conrady distortion, plus refraction through the plate. The firmware back-projects a pixel to a plate position with the same code the renderer uses.
  - `cad_geometry.hpp`: **generated** by `tools/cad/extract_geometry.py` from the STEP file. Don't edit it by hand.
- **`sim/`**: the simulator (closed IP). It uses SI units throughout, and every parameter in `params.hpp` is tagged [CAD], [datasheet], [derived] or [assumed].
  - `world.cpp` integrates the servo, mechanism and ball state with a fixed-step RK4 (0.25 ms). The ball's reaction on the plate is applied one step late.
  - `platform_dynamics.cpp` holds the reduced-coordinate dynamics in the crank angles, derived by virtual work. Its `J̇θ̇` term is a central difference along `θ̇`.
  - `ball.cpp` is a hybrid contact model with five modes: Rolling, Stuck, Slipping, Flight and Lost. Each step integrates with the mode fixed, then `postStep` projects the state back onto the constraints and handles mode transitions. Slip is removed with a momentum-consistent impulse, not by overwriting ω.
  - `renderer.cpp` ray-casts from the camera through the plate and samples the scene at sub-frame times from `SimWorld`'s state history. This produces the rolling shutter and motion blur.
  - All noise comes from the counter-based RNG in `rng.hpp`, never `<random>`, so runs are reproducible across compilers and threads.
- **`framework/`**: the student library and its runtime.
  - `include/bbp/bbp.hpp` is the public header, and `student_api.cpp` implements it (Image, PID, `log`).
  - `runtime.cpp` is the framework loop. It handles the FPS cap and sensor delay, then calls `track()`, converts pixels to mm through `backprojectToPlate` (using the *commanded* pose, as the firmware would), keeps the history and velocity filter, and calls `control()`. It then applies the angle limits and IK, and delays the servo command by the modelled CPU time (PC time × factor) plus the motor delay.
  - `runner_main.cpp` provides `main()` for the sim runner.
- **Runner process**: on Run, the app compiles the student's two `.cpp` files with MSVC (`app/bbp_app/toolchain.py`) and links them with `bbp_runner.lib`, `bbp_framework.lib` and `bbp_sim.lib` into `<project>/build/runner_*.exe`.
  - The app launches the runner as a child process, so a student crash cannot take the UI down.
  - The runner's real stdout carries a binary protocol: STATE, FRAME, LOG and TEXT messages, with layouts in `runner_main.cpp` and the matching `S_*` indices in `app/bbp_app/runner.py`. Commands go the other way on stdin.
  - Student `printf` is redirected to stderr, which the app shows in its console.
- **`app/bbp_app/`**: the PySide6 UI.
  - The 3D view (`widgets/view3d.py`, VTK) animates the CAD meshes. The plate follows the simulated pose. Each crank rotates about its CAD axis by θ − θ_CAD, and each rod rotates in its leg plane from its CAD pin and joint to the simulated ones.
  - The camera view shows the frames actually handed to `track()`, after the sensor delay and FPS cap, with the student's detection drawn on top.
- **`tests/`**: a self-contained test harness. Most tests compare against closed-form physics, such as 5/7 g sin β and the 2/7 Ω turntable orbit, and the paper's verification table quotes their output.
- **`docs/`**: three LaTeX documents: `physics/` (the model specification), `control/` and `vision/` (student guides to the two libraries). All share `docs/physics/references.bib`. The built PDFs are committed in `app/assets/docs/`, because the app's Documentation button opens them. Rebuild them with `tools/build_docs.sh` after editing. The guides quote the student API and the design numbers (G ≈ 122 mm/s² per degree, the starter-code gains), so update them when `bbp.hpp`, the templates or the defaults change.

## Mechanical reference

`CAD/FullASSY.step` (~23 MB, Autodesk export) is the source of truth for geometry. Don't edit it by hand, and grep it rather than reading it whole.

The mechanism is a symmetric 3-RRS:
- Legs at −150°, −30° and 90°.
- Crank: 50 mm.
- Rod: 118.6 mm.
- Ball joints at radius 94 mm.
- Plate: Ø200 × 5 mm PMMA.
- An OV2710 camera module 92 mm below the plate, looking up through it.

The CAD pose has the cranks at −45°. The simulator's neutral pose has them horizontal (θ=0, 1500 µs).
