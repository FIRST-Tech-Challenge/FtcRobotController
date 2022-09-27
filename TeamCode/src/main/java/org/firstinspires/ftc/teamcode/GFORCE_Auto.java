package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.GFORCE_KiwiDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
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
    public void runOpMode() throws InterruptedException {


        // Initialize GFORCE_KiwiDrive
        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(new Pose2d(new Vector2d(-62,-35), Math.toRadians(0)));

        waitForStart();
sleep(100);
        if (isStopRequested()) return;

        if (gamepad1.x) {
            TrajectorySequence trajectoryForward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(51)
                    .strafeLeft(59)
                    .build();
            drive.followTrajectorySequence(trajectoryForward);
            drive.update();


        }
        if (gamepad1.b) {
                TrajectorySequence trajectoryForward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(85)
                        .splineTo(new Vector2d(42,-41), Math.toRadians(-45))
                        .build();
                drive.followTrajectorySequence(trajectoryForward);
                drive.update();
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading gyro.", Math.toDegrees(drive.getExternalHeading()));
        telemetry.addData("heading odo.", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
        while (opModeIsActive() && !gamepad1.y) {
        }
    }

}
