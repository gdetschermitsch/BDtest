# CRUXTAIN Browser VIO/SLAM Proof v1

GitHub Pages, one iPhone, no beacon. This build implements the browser-accessible front half of the agreed architecture: camera feature tracks (Lucas–Kanade), robust outlier rejection, gyro rotational compensation, gravity vertical reference, compass heading seed, persistent pose state, and rejection/hold behavior.

## IMPORTANT accuracy boundary
The XYZ shown in v1 is **PROPORTIONAL**, not meters. A monocular camera cannot determine metric scale from vision alone. This build deliberately labels that fact instead of pretending otherwise. The next accuracy stage is a true multi-view essential-matrix/triangulation + metric-scale constraint and map reprojection optimizer.

## Use
Host this folder on HTTPS GitHub Pages. On iPhone Safari: Start → grant motion/camera → move naturally. No measured walk, beacon, second device, or return loop is required.

## Acceptance for this stage
- Feature count should remain populated on textured scenes.
- Inliers should remain substantial during normal motion.
- Rotation in place should cause much less false translation than uncompensated optical flow.
- A bad frame must show HOLDING LAST GOOD POSE instead of moving the world.

This is an instrumented engineering proof, not a claim that browser tracking already exceeds ARKit.
