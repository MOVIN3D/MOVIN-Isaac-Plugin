# Third-party notices

This repository is licensed under the Apache License, Version 2.0 (see `LICENSE`
and `NOTICE`). It includes or redistributes the following third-party
components, which keep their own licenses.

## MOVIN SDK Python

`movin_sdk_python` is a git submodule of
[MOVIN-SDK-Python](https://github.com/MOVIN3D/MOVIN-SDK-Python). Installing the
plugin (`pip install -e .`, or a wheel built from this repository) installs the
SDK's Python package and assets, which is a redistribution of the SDK.

```
MOVIN SDK Python
Copyright 2026 MOVIN Inc.

This product is licensed under the Apache License, Version 2.0.
```

The full license text is in `movin_sdk_python/LICENSE` and the notice file in
`movin_sdk_python/NOTICE`.

## NVIDIA Isaac Sim Grid environment

`data/ground/default_environment.usd` and its textures in
`data/ground/Materials/Textures/` (`Wireframe_blue.png`,
`WireframeBlur_basecolor.png`, `WireframeBlur_blue.png`) are copied unchanged
from the Grid environment of the NVIDIA Isaac Sim 5.1 assets
(`Isaac/Environments/Grid/`), so that the ground plane loads without the cloud
asset fetch. They are NVIDIA's assets and are **not** covered by this
repository's Apache License: they remain subject to NVIDIA's license terms for
Isaac Sim assets. The Isaac Sim 5.1 packages the plugin runs with ship the
[NVIDIA Omniverse License Agreement](https://docs.omniverse.nvidia.com/dev-guide/latest/common/NVIDIA_Omniverse_License_Agreement.html).

## Unitree G1 robot assets

The Unitree G1 robot models used for retargeting (`unitree_g1`,
`unitree_g1_with_hands`: MJCF and STL meshes) ship with MOVIN SDK Python
(`movin_sdk_python/movin_sdk_python/retargeter/assets/unitree_g1/`) and are
based on the Unitree G1 description in
[unitree_ros](https://github.com/unitreerobotics/unitree_ros) by Unitree
Robotics. They are subject to Unitree's own licensing terms; refer to that
repository for details. The plugin converts them to USD at runtime (cached in
`data/usd_output/`, which is not part of this repository).

## GMR: General Motion Retargeting

The retargeting approach used by the MOVIN SDK `Retargeter` is derived from
[GMR](https://github.com/YanjieZe/GMR). See the SDK repository for details.
