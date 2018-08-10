# Road Runner Quickstart

An example FTC project using [Road Runner](https://github.com/acmerobotics/road-runner). **Note:** Road Runner is in alpha and many of its APIs are incubating.

## Installation

For more detailed instructions on getting Road Runner setup in your own project, see the [Road Runner README](https://github.com/acmerobotics/road-runner#core).

1. Download or clone this repo with `git clone --recurse-submodules https://github.com/acmerobotics/road-runner-quickstart`.

1. Open the project in Android Studio and build `TeamCode` like any other FTC robot controller.

## Getting Started

For maximal flexibility, Road Runner has no dependencies on Android or the [FTC SDK](https://github.com/ftctechnh/ftc_app). Consequently, there is some code required to bridge the two. This project is intended to demonstrate a typical example of this integration. It also presents common routines for tuning/determining the various parameters required for proper operation.

### Drive Classes

As of now, only tank/differential and mecanum drives are supported (more to come in the future) by Road Runner. To use the path and trajectory followers as well as the drive characterization routines, you must create your own drive class.

The process is relatively similar for tank and mecanum drives. Simply extend `TankDrive` or `MecanumDrive`, create a constructor that initializes the appropriate `DcMotor` instances and use them to implement `getWheelPositions()`/`setMotorPowers()`.

### Drive Track Width

Although the track width of a robot (distance between a pair of wheels on opposite sides) can be determined physically, that number doesn't always match up with the drive's behavior due to friction and other factors. Instead, it's better to empirically calculate the track width by turning the robot in place a fixed angle and measuring the change in drive encoder positions.

This routine is implemented in `TrackWidthCalibrationOpMode`. Before running the op mode, make sure the drive's track width is set to 1. Additionally, the value returned corresponds to a wheel base of 0.

### Drive Characterization

Additionally, it's necessary to determine the relationship between robot velocity/acceleration and motor voltage. That is, we want to find the coefficients to satisfy the following relation: power = kV * v + kA * a (in reality, there's also a constant feedforward term to counter static friction; for more details see [this paper](https://www.chiefdelphi.com/media/papers/3402)). In theory, kV = 1 / max velocity and kA = 1 / max acceleration but you'll want to empirically compute them like the track width.

**Note**: In FTC, the easiest way to achieve good performance is to use the built-in velocity PID modes of the motor controllers. This effectively removes the acceleration term from the previous equation (power is now proportional to velocity, not voltage). Additionally, FTC batteries are pretty far from ideal voltage sources, creating further issues for acceleration feedforward (or direct voltage control of any sort).

This routine is implemented in `FeedforwardTuningOpMode`. Like the last op mode, this one will walk you through the process of tuning the coefficients. Again, from a practical standpoint in FTC, kV alone without kStatic and kA paired with motor velocity PID is the best configuration for beginners.

### Following a Trajectory

Now your tuned drive class can be used to follow trajectories/paths. Begin by instantiating your follower with the necessary arguments and calling `followPath()`/`followTrajectory()`. Then poll `update()` with the current pose estimate until it's finished (pose estimates can be computed with `Drive.updatePoseEstimate()` or another external mechanism).

## Dashboard

This project also contains some code for interfacing Road Runner with [FTC Dashboard](https://github.com/acmerobotics/ftc-dashboard).