# Road Runner Quickstart

An example FTC project using [Road Runner](https://github.com/acmerobotics/road-runner). **Note:** Road Runner is in alpha and many of its APIs are incubating.

## Acknowledgements

The quickstart uses the fantastic [RevExtensions2 library](https://github.com/OpenFTC/RevExtensions2) to take advantage of REV Expansion Hub bulk reads (thanks @FROGbots-4634!).

## Installation

For more detailed instructions on getting Road Runner setup in your own project, see the [Road Runner README](https://github.com/acmerobotics/road-runner#core).

1. Download or clone this repo with `git clone https://github.com/acmerobotics/road-runner-quickstart`.

1. Open the project in Android Studio and build `TeamCode` like any other `ftc_app` project.

1. If you have trouble with multidex, enable proguard by changing `useProguard` to `true` in `build.common.gradle`.

## Drive Getting Started

For maximal flexibility, Road Runner has no dependencies on Android or the [FTC SDK](https://github.com/ftctechnh/ftc_app). Consequently, there is some code required to bridge the two. This project is intended to demonstrate a typical example of this integration. It also presents common routines for tuning/determining the various parameters required for proper operation.

### Drive Classes

The process is pretty much the same for all drives. Simply extend `TankDrive`, `MecanumDrive`, or `SwerveDrive`, create a constructor that initializes the appropriate `DcMotor` instances, and use them to implement `getWheelPositions()`/`setMotorPowers()`. Often, it's also a good idea to put follower-related logic into the drive classes (this is the approach taken in the quickstart). If you decide to use one of the prebuilt drive classes (e.g., `SampleMecanumDriveOptimzed`), **make sure you update `DriveConstants` to reflect your robot**.

### Drive Velocity PID Tuning (optional)

If you plan on using the built-in velocity PID (i.e., `RUN_USING_ENCODER`), it's important to tune the coefficients for your robot (it is recommended you use the built-in velocity PID). Run `DriveVelocityPIDTuner` and adjust the PID gains to minimize the error as best you can. Finally, uncomment the `setPIDCoefficients()` stub at the bottom of your drive constructor and fill in the new coefficients.

### Drive Characterization

To determine the proper open loop powers, it's necessary to determine the relationship between robot velocity/acceleration and motor voltage. That is, we want to find `kV`, `kStatic`, and `kA` in the following relation: `power = kV * v + kA * a + kStatic` (there are some subtleties here; for more details see [this paper](https://www.chiefdelphi.com/media/papers/3402)). In theory, `kV = 1 / max velocity` and `kA = 1 / max acceleration`.

**Note**: In FTC, the easiest way to achieve good performance is to use the built-in velocity PID modes of the motor controllers (i.e., `RUN_USING_ENCODER`). This effectively removes the acceleration and static terms from the previous equation (power is now proportional to velocity, not voltage). Additionally, FTC users are cautioned against relying on `kA` as FTC batteries are far from ideal voltage sources.

An automated tuning routine is implemented in `DriveFFTuningOpMode`. Simply run it and follow the telemetry prompts. Again, from a practical standpoint in FTC, `kV` alone without `kStatic` and `kA` paired with motor velocity PID is the best configuration for beginners.

### Drive Track Width

Although the track width of a robot (distance between a pair of wheels on opposite sides) can be determined physically, that number doesn't always match up with the drive's behavior due to friction and other factors. Instead, it's better to empirically calculate the track width by turning the robot in place a fixed angle and measuring the change in drive encoder positions.

This routine was previously implemented in `TrackWidthCalibrationOpMode` although **it has been deprecated in favor of `NewTrackWidthCalibrationOpMode`**.

### Following a Trajectory

Now your tuned drive class can be used to follow trajectories/paths. The logic for creating followers is located in the base drive classes (e.g., `SampleMecanumDriveBase`). There are several op modes that show how to declare trajectories and follow them: `StraightTestOpMode`, `TurnTestOpMode`, and `SplineTestOpMode`. The former two test the drive characterization and track width calibration, respectively. The latter is a more sophisticated move for making sure all drive parameters have been tuned properly. **Make sure you verify the open loop responses are good before turning to feedback.**

### Next steps

Once the open loop response is good, you may begin adding positional feedback control using `FollowerPIDTuner`.

## Elevator Getting Started

The basic approach to using Road Runner with elevators is quite similar to drives:

1. Fill in the constants in `Elevator`.

1. Run `ElevatorFFTuningOpMode`.

1. Verify proper operation with `ElevatorTestOpMode`.

1. Tune the PID coefficients.

## FTC Dashboard

This project also contains some code for interfacing Road Runner with [FTC Dashboard](https://github.com/acmerobotics/ftc-dashboard). In fact, some tuning op modes require the dashboard. See [this page](https://acmerobotics.github.io/ftc-dashboard/gettingstarted#usage) for instructions on using the dashboard.