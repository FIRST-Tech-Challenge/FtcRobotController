package org.firstinspires.ftc.team00000.v2.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team00000.v2.RobotHardware;
import org.firstinspires.ftc.team00000.roadrunner.MecanumDrive;

import java.lang.Math;

@Config
@Disabled
@Autonomous(name = "Neutral", group = "Autonomous")

public class Neutral extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -61.25, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize all the hardware, using the hardware class.
        robot.init();

        Action phaseTest = drive.actionBuilder(initialPose)

                // score FIRST SPECIMEN
                .lineToYLinearHeading(-31.125, Math.PI / 2)

                // drive to SAMPLES
                .setTangent(-Math.PI /2)
                .splineToConstantHeading(new Vector2d(35.25, -31.125), Math.PI/2)
                .waitSeconds(0)
                .splineToConstantHeading(new Vector2d(42.375, -8.625), -Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(-47)
                .waitSeconds(0)
                .splineToConstantHeading(new Vector2d(51.375, -8.625), -Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(-47)
                .waitSeconds(0)
                .splineToConstantHeading(new Vector2d(59.875, -8.625), -Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(-49)
                .setTangent(5 * Math.PI / 6)

                // move and turn to intake specimen
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                // score SECOND SPECIMEN
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(15, -31.125, Math.PI/2), Math.PI/2)

                // move to STAGING POSITION
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                // score THIRD SPECIMEN
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(13.5, -31.125, Math.PI/2), Math.PI/2)

                // move to STAGING POSITION
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                // score FOURTH SPECIMEN
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(10.5, -31.125, Math.PI/2), Math.PI/2)

                // move to STAGING POSITION
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                // score FIFTH SPECIMEN
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(9, -31.125, Math.PI/2), Math.PI/2)

                // move to STAGING POSITION
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(47, -54, -Math.PI / 4), 0)

                .build();

        // Prepare robot before start
        Actions.runBlocking(robot.moveShoulder(robot.SHOULDER_WINCH_ROBOT));
        Actions.runBlocking(robot.moveClaw(robot.CLAW_OPEN));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Waiting for Start");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Execute the trajectory
        Actions.runBlocking(
                new SequentialAction(
                        robot.moveShoulder(robot.SHOULDER_WINCH_ROBOT),
                        robot.moveArm(robot.ARM_RETRACTED),
                        robot.moveWrist(robot.WRIST_STRAIGHT),
                        robot.moveClaw(robot.CLAW_OPEN),
                        phaseTest
                )
        );
    }

    // Helper method to create a sleep action
    private Action sleepAction(long milliseconds) {
        return packet -> {
            sleep(milliseconds);
            return false;
        };
    }
}