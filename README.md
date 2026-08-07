# CRUXTAIN Browser VIO/SLAM v2

This build continues the agreed one-iPhone GitHub Pages architecture.

## Implemented
- OpenCV `goodFeaturesToTrack` instead of the failed homemade detector.
- Pyramidal Lucas–Kanade temporal tracking.
- Essential-matrix RANSAC and `recoverPose` when exposed by the loaded OpenCV.js build.
- Geometric inlier display.
- Keyframe gating by inlier count and parallax.
- Triangulated landmark candidates.
- Last-good-pose hold: rejected frames cannot move the world.
- Gravity, heading and gyro diagnostics.
- No beacon, acoustic ranging, WebRTC or second device.

## Deliberately not misrepresented
Essential-matrix translation is only known up to scale. XYZ is displayed as **map units**, not meters. This package does not claim metric accuracy until visual–inertial scale alignment or another real metric constraint is implemented and physically validated.

## Physical test
1. Upload all files to an HTTPS GitHub Pages repository.
2. Open on iPhone Safari.
3. Tap Start and grant camera/motion permission.
4. Confirm visible feature dots appear.
5. Move naturally with some sideways component, not only pure rotation.
6. Watch for:
   - nonzero Features and Tracked;
   - green geometric inliers;
   - accepted geometric keyframes;
   - triangulated landmark count increasing;
   - rejected frames holding the last pose.

No measured movement, return loop, or second device is required to operate this proof.
