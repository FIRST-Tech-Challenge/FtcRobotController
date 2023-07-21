package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.Objects;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        SampleMecanumDrive roadrun = new SampleMecanumDrive(this.hardwareMap);
        Pose2d startPose = new Pose2d(35.25, 57.75, Math.toRadians(270));
        roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roadrun.setPoseEstimate(startPose);
        waitForStart();

        while (!isStopRequested()) {
            roadrun.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            roadrun.update();

            Pose2d poseEstimate = roadrun.getPoseEstimate();
            telemetry.addData("angle", poseEstimate.getHeading());
            telemetry.addData("angleVel", Objects.requireNonNull(roadrun.getPoseVelocity()).getHeading());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.update();
            logger.log("/RobotLogs/GeneralRobot", "Vel"+roadrun.getPoseVelocity().getX());
            logger.log("/RobotLogs/GeneralRobot", "Vel"+roadrun.getPoseVelocity().getY());
        }
    }
}
