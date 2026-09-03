# Third-party assets

This directory contains files redistributed for offline use by the G1 web
control panel.

- Unitree `unitree_ros`, commit
  `f3772ce54c56ef2d34c6aee8100bc768896c7d19`: G1 URDF and STL model assets,
  BSD-3-Clause. See `unitree/LICENSE.unitree_ros.txt`.
- Three.js `0.164.1`: JavaScript 3D renderer and selected addons, MIT. See
  `vendor/three/LICENSE.txt`.
- `urdf-loader` `0.13.1`, package commit
  `24feef2c267496d22d17bbbb5c1bf5b86dcf174f`: Apache-2.0. See
  `vendor/urdf-loader/LICENSE.txt`.

The vendored JavaScript module import specifiers were changed to local relative
paths so the control panel can run without an npm resolver or internet access.

The Unitree model is rendered as supplied. The application does not infer or
display dexterous-hand joint telemetry that is absent from its 29-joint data
schema.
