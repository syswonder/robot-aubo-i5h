# Sources and licenses

- Deployment files and `primitives/aubo_arm/`: MulanPSL-2.0, as declared by
  the adapter package. Full license in `LICENSE`.
- `skills/wave/`: derived from the colleague-provided AUBO Robonix wave
  package, with feedback freshness, completion and timeout handling changes.
  The original package declares Apache-2.0; full license in
  `skills/wave/LICENSE`.
- `model/aubo_i5H.urdf` and `model/meshes/aubo_i5H/`: derived from
  [AuboRobot/aubo_description](https://github.com/AuboRobot/aubo_description).
  Upstream package version 1.3.3 names Allen Liu
  (`liug@our-robotics.com`) as author and declares `BSD` in
  [package.xml](https://github.com/AuboRobot/aubo_description/blob/master/package.xml).
  That declaration does not specify a BSD variant; no different license is
  asserted here for these assets. Meshes are unchanged. The URDF uses the
  integrated robot's controller-calibrated joint origins, limits and TCP,
  removes the vendor world joint, and uses relative mesh paths.
- Robonix and `pyaubo_sdk` are external dependencies and are not bundled.
