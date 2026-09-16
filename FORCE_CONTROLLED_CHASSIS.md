# Force-controlled chassis migration

Ported from `E:/CODE/2026-Syscore-robot`, commit
`b640cffc78ecbcf53623a4f576ee9083f187aaba`, to WPILib 2026 and the existing
10541 hardware configuration. The source license ships in the robot JAR under
`META-INF/licenses/force-chassis-LICENSE.txt`.

`runTwist` now profiles chassis velocity and acceleration, allocates inertial
wheel forces, and composes the complete drive current on the roboRIO. The
traction governor scales slipping wheels; the limiter budgets the composed
current; MK5N drive motors receive `TorqueCurrentFOC`. Steering retains its
position loop. `runTwistWithTorque` uses caller-supplied module currents as
feedforward within the same software velocity loop.

The existing chassis limit setters, custom center of rotation, stop, X-lock,
drive brake setting, pose estimator, and odometry queue protection remain
available. Planned steering from rest bypasses the low-speed noise freeze.
Both simulation module implementations support current commands.

10541 retains its CAN IDs, encoder offsets, inversion, gearing, wheel diameter,
drive gains, 20 ms loop, and hardware current limits. The profile defaults to
4 m/s, 15 m/s² acceleration, and 25 m/s² braking. Angular velocity is also
bounded by module free speed and chassis geometry. Profile jerk limits come
from the source controller; the legacy unlimited setting remains subject to
physical speed bounds and the profile's bounded acceleration limits.

New parameter groups are `Params/Swerve/{Compose,Profile,Limiter,Traction,BodyVel}`.
They use the project's existing `ENABLE_NT_PARAMS` policy. Composition scale
defaults to 1.0, the P measurement filter to 3 Hz, and current limiting and
traction control to enabled. The 400 A limiter setting is a supply-current
budget; each drive motor retains its 100 A stator and 65 A supply hardware limits.

Yaw inertia is initially estimated as a uniform rectangle from the existing
46 kg real chassis mass and module dimensions; center of mass is initially at
the geometric center. These are estimates requiring CAD or hardware calibration.
Motor resistance comes from the WPILib Kraken X60 FOC model. No source robot
center-of-mass offset or mass-calibration trim is applied.

Pack voltage comes from `RobotController.getBatteryVoltage()`. This project has
no configured whole-pack current sensor: by default, the limiter budgets the
drivetrain alone. `setTotalCurrentSupplier(pdh::getTotalCurrent)` can supply a
configured PDH/PDP measurement so other mechanism loads are subtracted first.
The Pigeon acceleration yaw frame follows its configured mount yaw; setting
`accelFrameYaw` explicitly to 0 makes the software yaw correction an identity
for firmware or mounting arrangements that already report robot-frame acceleration.

All 122 checks passed, including the imported 5 ms replay fixtures and a complete
20 ms current-driven simulation. The build passed with Spotless scoped to the
migration files; pre-existing Hopper edits were left unchanged. Test sources and
compiled test artifacts were removed after verification, as requested, and the
robot JAR was rebuilt without them.

Compilation and simulation verification do not establish hardware calibration.
Nothing is deployed to the robot.
