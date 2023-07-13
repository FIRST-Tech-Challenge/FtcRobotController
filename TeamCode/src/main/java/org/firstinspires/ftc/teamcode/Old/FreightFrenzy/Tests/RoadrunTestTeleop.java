package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Tests;


import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(name = "RoadrunTestTeleop")
public class RoadrunTestTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PwPRobot robot = new PwPRobot(this, true);

        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.setPoseEstimate(new Pose2d(-34,-58,0));

        waitForStart();

        while (!isStopRequested()) {
            robot.roadrun.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            robot.roadrun.update();
            Pose2d poseEstimate = robot.roadrun.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading()*180/PI);
            telemetry.update();
        }
    }
}
