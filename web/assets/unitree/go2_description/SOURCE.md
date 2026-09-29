# Go2 model source

These assets are copied from Unitree Robotics' official `unitree_ros`
repository:

- Repository: `https://github.com/unitreerobotics/unitree_ros.git`
- Commit: `ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee`
- Source path: `robots/go2_description`
- Imported URDF: `urdf/go2_description.urdf`
- Imported meshes: `dae/*.dae`
- License: BSD 3-Clause, copied as `LICENSE.unitree_ros`

The import is intentionally limited to the URDF and mesh files required by
the existing Manifest-driven Web viewer.

`go2_web.urdf` and `stl/*.stl` are retained local conversion experiments and
are not selected by the runtime because the conversion does not preserve every
DAE scene transform. The Web viewer uses the unchanged official URDF and DAE
files above.
