# `robotics_cpp`
**A from-scratch C++23 robot kinematics library — FK/IK built on hand-rolled SE(3) math, cross-validated against ROS.**
[![CI](https://github.com/kshitij284/robotics_cpp/actions/workflows/ci.yml/badge.svg)](https://github.com/kshitij284/robotics_cpp/actions/workflows/ci.yml)
![C++](https://img.shields.io/badge/C%2B%2B-23-blue)
---

## The Library
This stack is built from the ground up to own the underlying mathematics rather than wrap an existing robotics framework. Every geometric primitive, Lie group transformation, analytical Jacobian, and kinematic solver is implemented from first principles in modern C++23 to prioritize transparency, zero-cost abstractions, and deterministic real-time safety.

---

## The Stack
* **`vec3` / `mat3` / `quaternion`** — `constexpr`-enabled 3D linear algebra primitives and unit quaternion operations without external math overhead.
* **`SE(3)`** — Special Euclidean group rigid-body transformations supporting composition, inversion, and RPY/axis-angle conversions.
* **`joint`** — Value-semantic joint representation (`Revolute`, `Prismatic`, `Fixed`) dispatched via `std::variant`.
* **`chain` / FK** — Serial kinematic chain evaluation computing local frames and end-effector poses in $O(N)$.
* **`Jacobian`** — Exact geometric $6 \times N$ spatial Jacobian construction for arbitrary serial chains.
* **`IK` (Damped Least Squares)** — Full 6-DOF pose inverse kinematics solver using Levenberg-Marquardt damping with Eigen's `LDLT` decomposition.
* **`URDF` Parser** — `tinyxml2`-backed parser enforcing serial chain topology (rejecting cycles, branches, and disconnected graphs at parse time).

---

## Minimal Example

```cpp
#include <iostream>
#include <vector>
#include "kinematics/iksolver.hpp"
#include "urdf/urdf_parser.hpp"

int main() {
  // 1. Parse and validate a robot description
  auto parsed = robotics::urdf::parse_urdf_joints("arm.urdf");
  if (!parsed) return 1;

  auto chain_exp = robotics::urdf::build_chain(*parsed);
  if (!chain_exp) return 1;
  const auto &chain = *chain_exp;

  // 2. Compute Forward Kinematics
  std::vector<double> q_seed = {0.1, -0.4, 0.5};
  auto current_pose = chain.forward_kinematics(q_seed);

  // 3. Solve 6-DOF Inverse Kinematics for a shifted target pose
  auto target_pose = kinematics::math::SE3::from_translation({0.05, 0.0, 0.0}) * current_pose;

  kinematics::math::IKSolver solver(chain);
  if (auto q_sol = solver.solve(target_pose, q_seed)) {
    std::cout << "Converged! Joint 0: " << (*q_sol)[0] << "\n";
  }
}
```

---

## Correctness & Verification

* **Numerical Cross-Validation:** Analytical spatial Jacobians are verified column-by-column against finite-difference approximations (`kEpsilonSmall`) across revolute, prismatic, and mixed kinematic chains.
* **ROS Ground-Truth Equivalence:** Forward kinematics transforms and RPY Euler compositions are cross-validated against `robot_state_publisher` (`tf2`) and SciPy spatial rotation references to machine precision ($10^{-9}$).
* **Adversarial & Negative Testing:** The URDF parser is stress-tested against malformed XML, missing attributes, shuffled joint declarations, cyclic graphs, branching trees, and disconnected forests.
* **Memory & Undefined Behavior Hygiene:** Built and tested under strict compiler warning flags (`-Wall -Wextra -Wpedantic -Wconversion -Wshadow` with scoped `-Werror`) and runtime sanitizers (`-fsanitize=address,undefined` and TSan).

---

## Design Notes
* **`std::expected` for recoverable errors:** Parse failures, graph topology errors, and IK non-convergence return explicit `std::expected` values instead of throwing exceptions or aborting control loops.
* **`std::variant` + `std::visit` for joint dispatch:** Joint kinematics (`Revolute`, `Prismatic`, `Fixed`) are resolved via static value-type visitation—eliminating heap allocations, virtual vtable indirection, and base-class pointer ownership.
* **Pure geometric FK with explicit limit enforcement:** Forward kinematics evaluates the pure geometric map $T(q)$ without silently clamping joint angles, keeping analytical derivatives smooth while enforcing hard joint bounds explicitly inside the IK update step.
* **Hybrid Eigen boundary:** Core 3D geometry is hand-rolled for zero-overhead `constexpr` evaluation, while **Eigen** is isolated strictly to dense $6 \times N$ matrix decompositions (`LDLT`) inside the IK solver.

---

## Build & Use

### Requirements
* **Compiler:** C++23 compliant toolchain (**GCC 13+** or **Clang 17+** required for full `std::expected` and multidimensional subscript `operator[i, j]` support).
* **Build System:** CMake **3.20+** (dependencies `Eigen 3.4.0`, `tinyxml2 11.0.0`, and `GoogleTest v1.14.0` are fetched automatically via `FetchContent`).

### Build & Run Tests
```bash
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_WERROR=ON \
  -DENABLE_SANITIZERS=ON

cmake --build build -j
ctest --test-dir build --output-on-failure
```

### Integrate via CMake `FetchContent`
```cmake
include(FetchContent)
FetchContent_Declare(
  robotics_cpp
  GIT_REPOSITORY https://github.com/<your-username>/robotics_cpp.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(robotics_cpp)

target_link_libraries(your_target PRIVATE components_lib)
```
