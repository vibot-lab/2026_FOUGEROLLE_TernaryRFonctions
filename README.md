# Constructing Associative Ternary R-functions

Reference implementation for the paper **“Constructing Associative Ternary
R-functions: A Geometric Perspective”**. The code implements and visualizes
ternary and N-ary R-function formulations for implicit modeling, with a focus on
spherical-simplex-based ternary composition, scalar-field regularity, gradient
stability, and CAD-style triple-junction reconstruction.

<p align="center">
  <img src="figs/ternary_operator_field.png" alt="Visual analysis of the proposed ternary R-function" width="850">
</p>

## Citation

If you find this paper or code useful for your research, please cite our work:

```bibtex
@article{ternary_r_functions_2026,
  title   = {Constructing Associative Ternary R-functions: A Geometric Perspective},
  author  = {Yohan Fougerolle, Joaquin Rodriguez, Aurore Gabon, Johan Gielis},
  journal = {Computer-aided design (CAD)},
  year    = {2026}
}
```

## Table of contents
- [Constructing Associative Ternary R-functions](#constructing-associative-ternary-r-functions)
  - [Citation](#citation)
  - [Table of contents](#table-of-contents)
  - [Overview](#overview)
  - [Repository structure](#repository-structure)
  - [Requirements](#requirements)
  - [Install on Windows with Visual Studio and `vcpkg`](#install-on-windows-with-visual-studio-and-vcpkg)
    - [Installing vcpkg](#installing-vcpkg)
    - [Installing dependencies with vcpkg](#installing-dependencies-with-vcpkg)
    - [Configuring the project in Visual Studio](#configuring-the-project-in-visual-studio)
  - [Install on Linux / Ubuntu / Debian](#install-on-linux--ubuntu--debian)
    - [Install system dependencies](#install-system-dependencies)
    - [Compile](#compile)
    - [Run](#run)
  - [Application modes](#application-modes)
  - [Configuration file](#configuration-file)
    - [`common`](#common)
    - [`field_render`](#field_render)
    - [`cad_junction`](#cad_junction)
    - [Benchmark sections](#benchmark-sections)
  - [Interactive controls](#interactive-controls)
  - [Method](#method)
  - [Mesh reconstruction pipeline](#mesh-reconstruction-pipeline)
  - [Results reported in the paper](#results-reported-in-the-paper)
    - [Gradient behavior](#gradient-behavior)
    - [Newton-Raphson convergence](#newton-raphson-convergence)
    - [CAD reconstruction](#cad-reconstruction)
  - [License](#license)

## Overview

Classical R-functions provide continuous implicit representations of Boolean
operations between primitives, but repeated binary compositions introduce
order-dependent scalar fields. Even when the zero level set is correct, the
field itself may become asymmetric, poorly conditioned, or numerically
inconvenient for optimization and reconstruction.

<p align="center">
  <img src="figs/binary_tree_imbalance.png" alt="Imbalance from successive binary intersections" width="650">
</p>

This repository explores a geometric alternative based on **ternary
R-functions**. The proposed operator reinterprets logical composition through
the geometry of a spherical simplex on $S^{2}$. The implementation provides:

- classical binary R-functions, including $R_{p}$ and $R_{0,m}$;
- N-ary reference formulations, including Rvachev- and Zenkin-style operators;
- the proposed spherical ternary operator and its normalized variant;
- OpenGL/FreeGLUT visualization of scalar fields and meshes;
- Marching Cubes extraction for implicit CAD scenes;
- mesh cleanup, valence optimization, smoothing, sharp-feature refinement, and
  projection back to the implicit zero set;
- a single executable controlled by application modes and a YAML configuration
  file.

## Repository structure

```text
.
├── AppConfig.cpp/.h         # YAML configuration parser and application-mode definitions
├── AppController.cpp/.h     # Application state, scene construction, benchmarks, OpenGL callbacks
├── CAD_Scene.h              # Composite implicit scene and selected Boolean/R-function logic
├── GLScene.cpp/.h           # OpenGL/FreeGLUT rendering context and interaction state
├── ImplicitObjects.cpp/.h   # Implicit primitives: spheres, planes, cylinders
├── MarchingCubes.cpp/.h     # Isosurface extraction, projection, sharp-edge refinement
├── Mesh.cpp/.h              # Triangular mesh structure, I/O, normals, drawing utilities
├── NeighborMesh.cpp/.h      # Mesh adjacency maps and topology-aware cleanup operations
├── Rfunctions.cpp/.h        # Classical, N-ary, and spherical ternary R-function implementations
├── main.cpp                 # Single executable entry point and mode dispatch
├── config.yaml              # Runtime configuration for scenes, benchmarks, CAD pipeline, outputs
├── launch.vs.json           # Visual Studio launch configurations for each application mode
├── CMakeLists.txt           # CMake build configuration
├── CMakePresets.json        # Visual Studio/vcpkg-oriented CMake preset
└── LICENSE                  # Project license
```

When compiled, the software can be executed using the following structure:

```bash
./TernaryRFunctions <APPLICATION_MODE> <CONFIG_FILE>
```

For example:

```bash
./TernaryRFunctions FIELD_RENDER ../config.yaml
./TernaryRFunctions CAD_JUNCTION_RENDER ../config.yaml
./TernaryRFunctions CAD_JUNCTION_BENCHMARK ../config.yaml
```

## Requirements

The project is written in **C++17** and uses CMake. The current build requires:

- CMake >= 3.16;
- a C++17 compiler, for example `g++`, `clang++`, or MSVC;
- OpenGL;
- GLU;
- FreeGLUT / GLUT;
- Eigen3;
- yaml-cpp.

## Install on Windows with Visual Studio and `vcpkg`

On Windows 10 or Windows 11, the recommended workflow is to use `vcpkg` to
install the dependencies, and Visual Studio for compilation and testing.

**Important:** keep the repository path short. On Windows, long paths can
trigger confusing CMake or compiler errors. A path such as
`C:\TernaryRFunctions` is preferable to a deeply nested directory.

### Installing vcpkg
First, install [`git` for Windows](https://git-scm.com/install/windows), and
[Visual Studio Community for
Windows](https://visualstudio.microsoft.com/vs/community/) with the C++
development tools. Then open a PowerShell terminal as administrator and run:

```powershell
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd C:\vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg integrate install
```

### Installing dependencies with vcpkg
Now, install the required dependencies, again using PowerShell. If you followed
the previous steps, `vcpkg` is installed in the root C drive. Thus, you should
run the following commands:

```powershell
C:\vcpkg\vcpkg install eigen3:x64-windows freeglut:x64-windows yaml-cpp:x64-windows
```

### Configuring the project in Visual Studio

As mentioned before, the repository should contain a short name, and the path
should be as short as possible. For the next steps, we will consider this git
repository has been downloaded in the folder `C:\TernaryRFunctions`.

Open Visual Studio, select the option `Open a folder`, and choose the repository
where the code is. Visual Studio will automatically detect that it is a CMake
project, but no extra configuration for CMake is required since we have
installed all the required libraries system wide using `vcpkg`.

To be able to correctly compile the project, we need to configure the
compilation tools in the `CMakePresets.json` file.

* For the `CMAKE_TOOLCHAIN_FILE`, we are going to choose vcpkg. Therefore we
  have to change the root path where vcpkg is installed (by default, it is
  `C:\vcpkg` as explained in this README file).

* Depending on the version of Visual Studio, we need to change the version of
  the `generator` variable. For instance, it can be `"generator": "Visual Studio
  18 2026"` or `"generator": "Visual Studio 17 2022"`. This information can be
  obtained in the `Help --> About Microsoft Visual Studio` menu.

Then use **Project → Delete Cache and Reconfigure**, followed by **Build → Build
All**.

The `launch.vs.json` file defines one launch configuration per application mode.
Each configuration runs the same executable and passes two arguments: the mode
name and `${workspaceRoot}\config.yaml`. By including this file in the
repository, Visual Studio recognizes the different tasks and creates the test
options as required.

## Install on Linux / Ubuntu / Debian

### Install system dependencies
Open a terminal and install the required development kits and libraries via apt:
```bash
sudo apt update
sudo apt install -y --no-install-recommends \
  build-essential \
  cmake \
  libeigen3-dev \
  libyaml-cpp-dev \
  freeglut3-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  mesa-common-dev
```

### Compile

From the repository root:

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j$(nproc)
```

### Run

From the build directory:

```bash
./TernaryRFunctions FIELD_RENDER ../config.yaml
./TernaryRFunctions CAD_JUNCTION_RENDER ../config.yaml
./TernaryRFunctions CAD_JUNCTION_BENCHMARK ../config.yaml
```

If the executable is launched from the repository root instead, use:

```bash
./build/TernaryRFunctions FIELD_RENDER config.yaml
```

## Application modes

The project uses a single executable with several application modes. The mode is
passed as the first command-line argument. The YAML configuration file is passed
as the second argument, and it defines the different modes input parameters.

| Mode                                 | Purpose                                                                                                                                                                                                           |
| ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `FIELD_RENDER`                       | Interactive visualization of scalar fields over a 2D domain or as a 3D height field. It can render configured spheres, a regular polygon, or a daisy-like arrangement of spheres.                                 |
| `CAD_JUNCTION_RENDER`                | Builds an implicit triple-cylinder junction, extracts the zero level set with Marching Cubes, applies the configured mesh-processing pipeline, optionally saves the mesh, and optionally opens the OpenGL viewer. |
| `CAD_JUNCTION_BENCHMARK`             | Benchmarks CAD junction reconstruction across logic types and grid resolutions.                                                                                                                                   |
| `GRADIENT_BENCHMARK`                 | Runs the gradient benchmark on the standard multi-sphere scene and writes a CSV report.                                                                                                                           |
| `GRADIENT_UNBALANCED_TREE_BENCHMARK` | Runs a gradient benchmark designed for unbalanced/deeper composition trees, typically using a daisy-like arrangement.                                                                                             |
| `NEWTON_RAPHSON_BENCHMARK`           | Tests Newton-Raphson convergence on a sampled domain and writes convergence statistics.                                                                                                                           |
| `RFUNCTION_PERFORMANCE_BENCHMARK`    | Compares evaluation cost of several R-function formulations for a given input dimension and number of points.                                                                                                     |


The available logic families are:

```cpp
LogicType::MIN_NAIVE    // Classical min/max logic
LogicType::BINARY_RP    // Symmetrized binary Rp composition
LogicType::TERNARY_KRF  // Proposed spherical ternary operator, normalized in CAD_Scene::Evaluate()
```

## Configuration file

The YAML file stores the parameters used by each mode. All the parameters are
organized as groups, and the most useful parameters are summarized below.

### `common`

```yaml
common:
  window:
    width: 1280
    height: 720

  rfunctions:
    rf_number: 8
    rvachev_m: 1
    default_logic: TERNARY_KRF

  spherical_evaluator:
    dimension: 3
```

- `window.width`, `window.height`: initial OpenGL window size.
- `rfunctions.rf_number`: number of R-function entries allocated for
  scalar-field rendering. The current interactive field renderer uses indices
  `0` to `7`.
- `rfunctions.rvachev_m`: regularization parameter used by the $R_{0,m}$ and
  Rvachev-style formulations.
- `rfunctions.default_logic`: default logic family where applicable. Valid
  values are `MIN_NAIVE`, `BINARY_RP`, and `TERNARY_KRF`.
- `spherical_evaluator.dimension`: dimension used to initialize the spherical
  evaluator. For the ternary operator used in the paper, this is normally `3`.

### `field_render`

```yaml
field_render:
  scene_mode: ClassicalExample
  sampling: 200
  is_intersection: false
  render_as_3d: false
  domain:
    U: [1.0, 0.0, 0.0]
    V: [0.0, 1.0, 0.0]
    LC: [-2.0, -2.0, 0.0]
    scale: 4.0
  scene:
    type: spheres
  gui:
    enabled: true
```

- `scene_mode`: internal rendering mode. Use `ClassicalExample` for the
  three-sphere/polygon-style example or `NaryExample` for N-ary/daisy-style
  visualization.
- `sampling`: grid resolution for field evaluation. Larger values give smoother
  visualizations but increase computation and OpenGL buffer size.
- `is_intersection`: selects the initial Boolean operation. `true` means
  intersection; `false` means union. It can also be toggled interactively with
  the keyboard key `I`.
- `render_as_3d`: if `false`, the scalar field is shown as a flat color map; if
  `true`, values are rendered as a height field. It can also be toggled with the
  key `P`.
- `domain.U`, `domain.V`: basis vectors of the sampled plane.
- `domain.LC`: lower-corner/origin offset of the sampled domain.
- `domain.scale`: size of the sampled square/parallelogram in the `U` and `V`
  directions.
- `scene.type`: selects the primitive setup. Valid values are `polygon`,
  `spheres`, and `daisy`.
- `scene.polygon.n_sides`, `scene.polygon.radius`: used when `type: polygon`.
- `scene.spheres`: list of sphere centers and radii used when `type: spheres`.
- `scene.daisy.primitive_radius`, `center_radius`, `primitive_count`: sphere
  radius, placement radius, and number of primitives used when `type: daisy`.
- `gui.enabled`: must currently be `true` for `FIELD_RENDER`, because the field
  renderer builds OpenGL display lists.

### `cad_junction`

```yaml
cad_junction:
  junction_type: X_JUNCTION
  cylinder_radius: 0.5
  logical_operation: union
  logic: TERNARY_KRF
```

- `junction_type`: selects the CAD junction geometry. Valid values are
  `X_JUNCTION` and `Y_JUNCTION`.
- `cylinder_radius`: radius of the cylinders used to construct the
  triple-junction scene.
- `logical_operation`: `union` or `intersection`.
- `logic`: R-function logic used to compose the primitives. Valid values are
  `MIN_NAIVE`, `BINARY_RP`, and `TERNARY_KRF`.

The Marching Cubes block controls the sampled 3D volume:

```yaml
marching_cubes:
  min_bound: [-1.5, -1.5, -1.5]
  max_bound: [1.5, 1.5, 1.5]
  grid_resolution: 80
```

- `min_bound`, `max_bound`: lower and upper corners of the sampled volume.
- `grid_resolution`: number of grid cells per axis. Higher values improve
  geometric resolution but increase runtime and memory use.

The threshold block controls numerical tolerances derived from the Marching
Cubes grid spacing:

```yaml
thresholds:
  init_epsilon_factor_dx: 0.01
  area_threshold_factor_init_squared: 5.0
  final_epsilon_factor_init: 2.0
  final_area_threshold_factor: 1.0
  sharp_angle_deg: 20.0
```

- `init_epsilon_factor_dx`: defines `init_epsilon = dx *
  init_epsilon_factor_dx`, where `dx` is the grid spacing. It affects initial
  vertex welding and topology cleanup.
- `area_threshold_factor_init_squared`: defines the initial small-triangle area
  threshold from `init_epsilon^2`.
- `final_epsilon_factor_init`: defines the final welding tolerance as a multiple
  of `init_epsilon`.
- `final_area_threshold_factor`: defines the final small-triangle collapse
  threshold from `final_epsilon^2`.
- `sharp_angle_deg`: angle threshold used to detect and refine sharp features
  and to guide feature-preserving smoothing.

The pipeline block enables or disables each CAD reconstruction step:

```yaml
pipeline:
  run_marching_cubes: true
  weld_initial_vertices: true
  run_topology_cleanup: true
  run_valence_optimization: true
  valence_max_passes: 5
  run_parasite_collapse: true
  run_triangle_improvement: true
  triangle_improvement_iterations: 50
  project_points_to_surface: true
  refine_sharp_edges: true
  run_small_triangle_collapse: true
  run_unified_cleanup: true
  run_feature_preserving_smooth: true
  feature_smooth_iterations: 10
  run_ultimate_mesh_sanitizer: true
  sanitizer_min_area_factor: 2.0
  sanitizer_min_aspect_ratio: 0.06
```

- `run_marching_cubes`: extracts the initial triangle mesh from the implicit
  field.
- `weld_initial_vertices`: merges near-duplicate vertices after extraction.
- `run_topology_cleanup`: removes invalid or degenerate topological structures.
- `run_valence_optimization`, `valence_max_passes`: performs local edge flips to
  improve valence regularity.
- `run_parasite_collapse`: collapses small parasitic structures.
- `run_triangle_improvement`, `triangle_improvement_iterations`: improves
  triangle quality by geometric relaxation.
- `project_points_to_surface`: projects mesh vertices back to the implicit zero
  level set.
- `refine_sharp_edges`: inserts additional geometry around sharp features
  detected by angle threshold.
- `run_small_triangle_collapse`: removes very small triangles using the final
  area threshold.
- `run_unified_cleanup`: performs a final welding/cleanup pass.
- `run_feature_preserving_smooth`, `feature_smooth_iterations`: smooths the mesh
  while preserving sharp features.
- `run_ultimate_mesh_sanitizer`: removes remaining low-quality triangles.
- `sanitizer_min_area_factor`: scales the minimum area used by the final
  sanitizer.
- `sanitizer_min_aspect_ratio`: controls removal of elongated/sliver triangles.
  Higher values are more aggressive.

Rendering and output options:

```yaml
render:
  mesh_buffer_sampling: 100
  gui_enabled: true

output:
  save_mesh: false
  mesh_file: "output.iv"
```

- `render.gui_enabled`: if `true`, opens the OpenGL viewer after building the
  mesh. If `false`, the CAD pipeline can run headlessly.
- `render.mesh_buffer_sampling`: used when buffering the result for
  visualization.
- `output.save_mesh`: writes the generated mesh to disk when enabled.
- `output.mesh_file`: path of the exported mesh file.

### Benchmark sections

`cad_junction_benchmark` uses a similar CAD setup, but adds:

- `logics`: list of logic families to compare.
- `resolutions`: Marching Cubes grid resolutions to test.
- `output.csv_file`: CSV output path.
- `gui.enabled_after_benchmark`: optionally opens the viewer after the
  benchmark.

`gradient_benchmark` and `gradient_unbalanced_tree_benchmark` control:

- `is_intersection`: operation tested.
- `points_per_circle`: number of sampled points used by the benchmark routine.
- `field_value_start`, `field_value_end`, `number_of_runs`: sampled field-offset
  range.
- `numerical.finite_difference_h`: finite-difference step for gradient
  estimation.
- `numerical.boundary_tolerance`: tolerance around the implicit boundary.
- `output.csv_file`: CSV output path.

`newton_raphson_benchmark` controls:

- `grid_resolution`: sampled grid resolution for convergence testing.
- `domain.min_bound`, `domain.max_bound`: sampled 3D domain.
- `numerical.epsilon`: convergence tolerance.
- `numerical.max_iterations`: maximum Newton iterations.
- `numerical.finite_difference_h`: finite-difference step.
- `numerical.suture_threshold`: threshold around suture regions.
- `methods`: methods to include in the benchmark.
- `output.csv_file`: CSV output path.

`rfunction_performance_benchmark` controls:

- `dimension`: number of primitive field values per sampled point. Useful values
  for recursive ternary tests are `3`, `9`, and `27`.
- `point_number`: number of random evaluations.
- `is_intersection`: operation tested.
- `methods`: methods to include in the performance comparison.
- `output.print_to_console`: prints results in the terminal.
- `output.csv_file`: optional CSV output path. Use `null` to disable file
  output.

## Interactive controls

| Key / input | Action |
|---:|---|
| Left mouse drag | Rotate scene |
| `+` | Zoom in |
| `-` | Zoom out |
| `R` | Reset rotation and view translation |
| `W` | Wireframe rendering |
| `F` | Filled surface rendering |
| `Esc` | Close the application |

Additional controls in `FIELD_RENDER`:

| Key | Action |
|---:|---|
| `0` | Min/Max |
| `1` | Chained $R_p$ |
| `2` | Chained $R_{0,m}$ |
| `3` | Zenkin N-ary |
| `4` | Rvachev N-ary |
| `5` | Spherical directional |
| `6` | Spherical normalized |
| `I` | Toggle intersection / union |
| `P` | Toggle flat / 3D height-field rendering |

## Method

The method addresses a structural limitation of binary R-function trees.
Pairwise compositions are not scalar-field associative: different binary
aggregation orders may represent the same Boolean set but produce different
field values and gradients away from the zero level set. This becomes
problematic for implicit modeling pipelines relying on stable gradients, for
example Newton projection, mesh relaxation, or optimization.

The proposed formulation builds a ternary operator from spherical-simplex
geometry. Given the values of three implicit primitives, the vector of signs and
magnitudes is mapped to a direction on the sphere. The operator evaluates an
opening parameter associated with a spherical simplex and compares it to a
reference threshold. The normalized version multiplies the directional output by
the input norm, restoring a distance-like behavior and a quasi-unit gradient
near the zero level set.

In implementation terms, the central class is `RCore::SphericalEvaluator`, which
exposes:

```cpp
double Compute(bool isIntersection, const Eigen::VectorXd& X);
double ComputeNormalized(bool isIntersection, const Eigen::VectorXd& X);
```

The normalized spherical operator is used in `CAD_Scene::Evaluate()` when
`LogicType::TERNARY_KRF` is selected.

## Mesh reconstruction pipeline

The CAD junction mode performs the following steps, depending on the enabled
`cad_junction.pipeline` flags:

1. Assemble an implicit triple-junction scene from cylinders.
2. Evaluate the selected R-function composition over a 3D grid.
3. Extract the zero level set using Marching Cubes.
4. Weld near-duplicate vertices and remove degenerate structures.
5. Optimize mesh valence by local edge flips.
6. Collapse parasitic structures and very small triangles.
7. Improve triangle quality and optionally smooth the mesh.
8. Project vertices back to the zero level set.
9. Refine sharp features and sanitize high-aspect-ratio elements.
10. Render and/or save the optimized mesh.

<p align="center">
  <img src="figs/cad_junction_meshes.png" alt="Initial and optimized CAD junction meshes" width="850">
</p>

## Results reported in the paper

The paper evaluates the proposed formulation through gradient analysis, runtime
comparison, Newton-Raphson convergence, and CAD reconstruction.

### Gradient behavior

The normalized spherical ternary operator gives a more regular field profile
than the non-normalized directional version. It avoids far-field flattening and
provides a more predictable gradient magnitude around the zero level set. The
experiments compare the proposed method with binary $R_p$, $R_{0,m}$, Rvachev
N-ary, and Zenkin N-ary formulations.

<p align="center">
  <img src="figs/gradient_convergence_analysis.png" alt="Gradient and convergence analysis" width="850">
</p>

### Newton-Raphson convergence

On a $100^{3}$ grid for a three-primitive composite scene, the normalized
spherical version reaches **100% convergence**, with an average of **4.64
iterations** and an average residual of **6.51e-08**. In the same benchmark, the
unnormalized directional version suffers from far-field gradient vanishing and
fails on a large fraction of the sampled points.

### CAD reconstruction

For triple-junction reconstruction, the ternary operator maintains low geometric
error across tested resolutions. The reported mean geometric error is
approximately **1.87e-09**, while preserving sharper junction behavior than
simpler max/min logic. The paper reports an expected runtime overhead of about
**2.75×** compared with max logic, with runtime comparable to the symmetrized
$R_p$ variant.

<p align="center">
  <img src="figs/performance_error_analysis.png" alt="Performance and error analysis" width="850">
</p>

The additional mesh resolution produced by the ternary operator reflects its
ability to preserve a sharper and more distinct potential field around singular
junctions, which triggers adaptive refinement and improves topological fidelity.

## License

This project is distributed under the **MIT License**. See [`LICENSE`](LICENSE)
for details.