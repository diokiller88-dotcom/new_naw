# Scan Context++ and IRIS Fusion

This module implements the Scan Context++ descriptor and search pipeline as an
independent part of `relocation_core`. The relocation rough-search path now uses
IRIS exact-Hamming Top-80 and SC++ Top-20 candidate union, dual-descriptor
ranking, and the existing GICP verification stage.

## Implemented features

- IRIS-compatible Polar Context (PC) defaults: 10 rings, 360 sectors, 10 m range
- IRIS-compatible Cart Context (CC) defaults: 20 by 20 bins over `[-10, 10] m`
- Original paper PC/CC configurations remain available as explicit factories
- Augmented Polar Context (A-PC): virtual lateral roots at `+2 m` and `-2 m`
- Cart Context (CC) with paper defaults: 40 longitudinal by 40 lateral bins
- Augmented Cart Context (A-CC): double-axis descriptor flip
- Maximum-height bin encoding and optional voxel filtering
- L1 retrieval keys and alignment keys
- Exact dynamic-dimensional KD-tree retrieval-key index
- Aligning-key pre-alignment and local full-descriptor refinement
- Column-wise cosine distance for false-positive rejection
- Relative yaw for PC and relative lateral displacement for CC

The implementation follows the algorithm described in:

> G. Kim, S. Choi, and A. Kim, "Scan Context++: Structural Place Recognition
> Robust to Rotation and Lateral Variations in Urban Environments," IEEE
> Transactions on Robotics, 2022.

## Basic usage

```cpp
#include "relocation/scan_context.hpp"

auto config = relocation::ScanContextConfig::IrisPolar();

relocation::ScanContextPlusPlus sc(config);
sc.AddPlace(map_keyframe_0, 0);
sc.AddPlace(map_keyframe_1, 1);
sc.BuildIndex();

relocation::ScanContextMatch match = sc.Query(query_cloud);
if (match.matched) {
    // match.place_id
    // match.distance
    // match.relative_yaw_rad
}
```

`QueryCandidates()` returns Top-K unique places, while `ComparePlace()` performs
an exact comparison against every augmented descriptor of one place.

`AddPlace()` stores the original descriptor and the enabled augmentation
variants under one place id. `Query()` creates only the original query
descriptor and searches the combined original/augmented map index.

For Cart Context:

```cpp
auto config = relocation::ScanContextConfig::IrisCartesian();
relocation::ScanContextPlusPlus cc(config);
```

The default constructor also uses `IrisPolar()`.

The sign convention is:

- `column_shift`: circular shift applied to the query descriptor
- PC `relative_yaw_rad`: negative query shift multiplied by radians per column
- CC `relative_lateral_m`: negative query shift multiplied by metres per column

## IRIS-compatible defaults

The default configuration follows the current LiDAR-Iris spatial limits and
search granularity:

| Setting | IRIS | SC++ default |
|---|---:|---:|
| Maximum radius | 10 m | 10 m |
| Radial bins | 10 | 10 |
| Angular bins | 360 | 360 |
| Height range | 0–2 m | 0–2 m |
| Internal voxel filtering | none | none |
| Final rough candidates | 8 | 8 |
| Local angular refinement | ±2 columns | ±2 columns |

For the original Scan Context++ paper settings, use:

```cpp
auto pc = relocation::ScanContextConfig::PaperPolar();
auto cc = relocation::ScanContextConfig::PaperCartesian();
```

The full relocation test stores SC++ descriptors in `history_db_sc.bin`. The
sidecar is versioned and stores the IRIS database content fingerprint, map
point-cloud fingerprint, file size, and point count. `location` enables fusion
only when all identity fields, the descriptor configuration, and the contiguous
place-ID set match. A missing, legacy, or mismatched sidecar falls back to
IRIS-only retrieval; the interactive test regenerates it automatically from the
current history database and map.

## Test

```bash
colcon build --packages-select relocation
source install/setup.bash
ros2 run relocation scan_context_test
```

The standalone test covers polar rotation invariance, yaw estimation, A-PC
generation, Cartesian double-flip matching, Top-K/direct-place comparison,
angular seam wrapping, database identity validation, serialization, indexing,
and clearing.

The complete IRIS/SC++/GICP pipeline can also run without a display:

```bash
ros2 run relocation test_node --headless-smoke
```
