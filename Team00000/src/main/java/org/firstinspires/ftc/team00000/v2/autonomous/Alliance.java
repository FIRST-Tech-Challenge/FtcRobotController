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
@Autonomous(name = "Alliance", group = "Autonomous")

public class Alliance extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -61.25, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize all the hardware, using the hardware class.
        robot.init();

        Action phase1 = drive.actionBuilder(initialPose)
                // pick up FIRST SPECIMEN
                .lineToYLinearHeading(-59.25, Math.PI/2)
                .build();

        Action phase2 = drive.actionBuilder(new Pose2d(0, -59.25, Math.PI / 2))
                // score FIRST SPECIMEN
                .lineToYLinearHeading(-31.125, Math.PI /2)
                .build();

        Action phase3 = drive.actionBuilder(new Pose2d(0, -31.125, Math.PI / 2))
                // push ALL SAMPLES into OBSERVATION ZONE
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
                .splineToConstantHeading(new Vector2d(61.875, -8.625), -Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(-49)
                .setTangent(5 * Math.PI / 6)

                // move and turn to intake specimen
                .splineToLinearHeading(new Pose2d(47, -54, Math.PI / 2), Math.PI / 2)
                .build();

        Action phase4 = drive.actionBuilder(new Pose2d(27, -45, -Math.PI / 3))
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(0, -27, Math.PI/2), Math.PI/2)
                .build();

        Action phase0 = drive.actionBuilder(new Pose2d(0, -36, Math.PI/2))
                .lineToYLinearHeading(-48, Math.PI/2)
                .build();

        Action phase5 = drive.actionBuilder(new Pose2d(0, -36, Math.PI / 2))
                .setTangent(-Math.PI / 4)
                .splineToLinearHeading(new Pose2d(28, -40, Math.PI / 6), 0)
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

                        new ParallelAction(
                                robot.moveShoulder(robot.SHOULDER_SAMPLE_RETRACTED),
                                phase1
                        ),

                        robot.moveClaw(robot.CLAW_CLOSE),
                        sleepAction(250),

                        new ParallelAction(
                                robot.moveShoulder(robot.SHOULDER_HIGH_CHAMBER),
                                robot.moveWrist(robot.WRIST_FLIP),
                                phase2
                        ),

                        robot.moveArm(robot.ARM_HIGH_CHAMBER),

                        new ParallelAction(
                                robot.moveArm(robot.ARM_RETRACTED),
                                robot.moveWrist(robot.WRIST_STRAIGHT),
                                robot.moveClaw(robot.CLAW_OPEN),
                                phase3,
                                sleepAction(500),
                                robot.moveShoulder(robot.SHOULDER_SAMPLE_RETRACTED)
                        )
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