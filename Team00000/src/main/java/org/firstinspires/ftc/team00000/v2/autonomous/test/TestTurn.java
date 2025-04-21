package org.firstinspires.ftc.team00000.v2.autonomous.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team00000.v2.RobotHardware;
import org.firstinspires.ftc.team00000.roadrunner.MecanumDrive;

@Disabled
@Autonomous(name = "Test 3: Turn and Move (Absolute Positioning)", group = "Autonomous")

public class TestTurn extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize all the hardware, using the hardware class.
        robot.init();

        Action test = drive.actionBuilder(initialPose)
                .turn(-Math.PI / 2)  // Rotate 90 degrees (now facing +X)
                .lineToX(24)  // Move right (field-centric X+)
                .turn(Math.PI / 2)  // Rotate back to face +Y
                .lineToY(24)  // Move forward
                .setTangent(Math.PI / 4)
                .lineToY(0)   // Return to start
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
                        test
                )
        );
    }
}
