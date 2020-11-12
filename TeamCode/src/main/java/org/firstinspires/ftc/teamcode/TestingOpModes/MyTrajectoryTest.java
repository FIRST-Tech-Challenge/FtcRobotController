package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "My Trajectory Test", group = "Testing")
public class MyTrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Starting pose - Blue outer line facing target Marker.
        Pose2d myPose = new Pose2d(-64,48, Math.toRadians(-15));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        //WRITE OR UPDATE TRAJECTORY HERE

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
    }
}

/* Trajectory syntax examples from https://www.learnroadrunner.com/trajectorybuilder-functions.html

// Drives forward 40 inches
new TrajectoryBuilder(new Pose2d())
  .forward(40)
  .build()

// Drives backward 40 inches
new TrajectoryBuilder(new Pose2d())
  .back(40)
  .build()

// Strafes left 40 inches
new TrajectoryBuilder(new Pose2d())
  .strafeLeft(40)
  .build()

// Strafes right 40 inches
new TrajectoryBuilder(new Pose2d())
  .strafeRight(40)
  .build()

// Robot moves to the specified coordinates.
// It keeps the same heading as when you start the move.
// So, if you start at a 90 degree angle, it will keep that angle the entire path.
// strafeTo() is simply a shorthand forward lineToConstantHeading() / lineTo()
new TrajectoryBuilder(new Pose2d())
  .strafeTo(new Vector2d(40, 40))
  .build()

// Robot moves to the specified coordinates while linearly
// interpolating between your current heading and a specified end heading.
new TrajectoryBuilder(new Pose2d())
  .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)))
  .build()

// Robot moves to the specified coordinates while spline
// interpolating between your current heading and a specified end heading.
new TrajectoryBuilder(new Pose2d())
  .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)))
  .build()

// Robot moves to the specified coordinates in a spline path
// while following a tangent heading interpolator.
new TrajectoryBuilder(new Pose2d())
  .splineTo(new Vector2d(40, 40), Math.toRadians(0))
  .build()

// Robot moves to the specified coordinates in a spline path
// while keeping the heading constant.
// The heading is kept at the heading at the start of the movement.
// However, setting the `endTangent` does affect the spline shape.

new TrajectoryBuilder(new Pose2d())
  .splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(0))
  .build()

// Robot moves to the specified coordinates in a spline path
// while separately linearly interpolating the heading
// The heading interpolates to the heading specified in `endPose`.
// Setting `endTangent` affects the shape of the spline path itself.
// Due to the holonomic nature of mecanum drives, the bot is able
// to make such a movement while independently controlling heading.
new TrajectoryBuilder(new Pose2d())
  .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
  .build()

// Robot moves to the specified coordinates in a spline path
// while separately spline interpolating the heading
// The heading interpolates to the heading specified in `endPose`.
// Setting `endTangent` affects the shape of the spline path itself.
// Due to the holonomic nature of mecanum drives, the bot is able
// to make such a movement while independently controlling heading.
new TrajectoryBuilder(new Pose2d())
  .splineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
  .build()

//Markers
//=======
//PS : Don't use sleep() in markers!

//Temporal Marker
drive.trajectoryBuilder(new Pose2d())
  .splineTo(new Vector2d(36, 36), Math.toRadians(0))
  .addTemporalMarker(2, () -> {
      // This marker runs two seconds into the trajectory
      // Run your action in here!
  })
  .splineTo(new Vector2d(72, 0), Math.toRadians(0))
  .build()

//Inline Displacement Markers
drive.trajectoryBuilder(new Pose2d())
  .splineTo(new Vector2d(36, 36), Math.toRadians(0))
  .addDisplacementMarker(() -> {
      // This marker runs after the first splineTo()
      // Run your action in here!
  })
  .splineTo(new Vector2d(72, 0), Math.toRadians(0))
  .build()

//Global Displacement Markers
drive.trajectoryBuilder(new Pose2d())
  .splineTo(new Vector2d(36, 36), Math.toRadians(0))
  .addDisplacementMarker(20, () -> {
      // This marker runs 20 inches into the trajectory
      // Run your action in here!
  })
  .splineTo(new Vector2d(72, 0), Math.toRadians(0))
  .build()

//Spatial Markers
drive.trajectoryBuilder(new Pose2d())
  .splineTo(new Vector2d(36, 36), Math.toRadians(0))
  .addSpatialMarker(new Vector2d(20, 20), () -> {
      // This marker runs at the point that gets
      // closest to the (20, 20) coordinate
      // Run your action in here!
  })
  .splineTo(new Vector2d(72, 0), Math.toRadians(0))
  .build()


 */