# Quadrotor Simulator
This is a simple simulator for quadrotor in MATLAB.

## Requirements
- Optimization Toolbox.

## How to use
1. `testTrajectoryGeneration.m`.
   1. Run it in MATLAB.
   2. Set waypoints using cursor, and press ENTER to finish.
   3. It will show the minimum-snap trajectory.
2. `testTimeAllocation.m`.
   1. The same as `testTrajectoryGeneration.m`'s 1 and 2.
   2. It will show the spatial-temporal and spatial trajectory.
   3. The spatial trajectory has the same time allocation for each piece of trajectory.
3. `testTrajectoryTracking.m`
   1. The same as `testTrajectoryGeneration.m`'s 1.
   2. Set waypoints using cursor, and then enter the height in `Command Window`. Finally, press ENTER to finish.
   3. It will show a quadrotor tracking the generated trajectory.
