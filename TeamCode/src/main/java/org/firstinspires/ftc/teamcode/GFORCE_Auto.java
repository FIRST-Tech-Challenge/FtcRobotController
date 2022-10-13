package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.GFORCE_KiwiDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name="G-FORCE AUTO", group = "advanced")
public class GFORCE_Auto extends LinearOpMode {

    boolean headingLock = false;
    double  headingSetpoint = 0;

    @Override
    public void runOpMode() throws InterruptedException{


        // Initialize GFORCE_KiwiDrive
        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(new Pose2d(new Vector2d(35,-62), Math.toRadians(90)));

        TrajectorySequence ourTrajectory = null;

        telemetry.addData("Trajectory", "press X or B button to select.");
        telemetry.update();

        while (opModeInInit()) {
            // Select the desired trajectory

            if (gamepad1.x) {
                telemetry.addData("Trajectory", "Far Right Floor");
                telemetry.update();
                ourTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(27)
                        .strafeLeft(72)
                        //.forward(12)
                        .build();
            }

            if (gamepad1.b) {
                telemetry.addData("Trajectory", "Cross Near Mid");
                telemetry.update();
                drive.setExternalHeading(Math.toRadians(90));
                drive.setPoseEstimate(new Pose2d(new Vector2d(35,-62), Math.toRadians(90)));
                ourTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(35, -36))
                        .strafeTo(new Vector2d(48, -36))
                        .build();
            }

            if (gamepad1.y) {
                telemetry.addData("Trajectory", "Other Corner Spline ");
                telemetry.update();
                drive.setExternalHeading(Math.toRadians(90));
                drive.setPoseEstimate(new Pose2d(new Vector2d(35,-62), Math.toRadians(90)));
                ourTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(35, -48))
                        .splineTo(new Vector2d(24,-36), Math.toRadians(180))
                        .lineTo(new Vector2d(0,-36))
                        .splineTo(new Vector2d(-12,-24), Math.toRadians(90))
                        .lineTo(new Vector2d(-12, 24))
                        .splineTo(new Vector2d(-24,36), Math.toRadians(180))
                        .lineTo(new Vector2d(-36, 36))
                        .build();
            }

            if (gamepad1.a) {
                telemetry.addData("Trajectory", "Phils office ");
                telemetry.update();
                drive.setExternalHeading(Math.toRadians(90));
                drive.setPoseEstimate(new Pose2d(new Vector2d(35,-62), Math.toRadians(90)));
                ourTrajectory = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(35,-62), Math.toRadians(90)))
                        .lineTo(new Vector2d(35,35))
                        .setReversed(true)
                        .lineTo(new Vector2d(35,24))
                        .splineTo(new Vector2d(24,12), Math.toRadians(180))
                        .lineTo(new Vector2d(12,12))
                        .setReversed(false)
                        .lineTo(new Vector2d(24,12))
                        .splineTo(new Vector2d(35,0), Math.toRadians(-90))
                        .lineTo(new Vector2d(35,-62))
                        .setReversed(true)
                        .lineTo(new Vector2d(35,35))
                        .build();
                telemetry.addData("Trajectory", "Phils office COMPLETE");
                telemetry.update();
            }
        }

        if (opModeIsActive()) {
            // if we have a trajectory, run it.
            if (ourTrajectory != null) {
                drive.followTrajectorySequence(ourTrajectory);
                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading gyro.", Math.toDegrees(drive.getExternalHeading()));
                telemetry.addData("heading odo.", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.update();
            }

            // Wait for button prrss to exit;
            while (opModeIsActive()) {
                sleep(10);
            }
        }
    }
}
