package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Robot: Auto Drive until stop", group="Robot")
public class AutoDriveUntilStop  extends LinearOpMode{

    private RobotHardware robot = new RobotHardware(this);
    static final double FORWARD_SPEED = 0.2;

    @Override
    public void runOpMode() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Ready to run...");
            telemetry.update();

            robot.driveRobot(FORWARD_SPEED, 0);
            sleep(500);
            robot.driveRobot(0, FORWARD_SPEED);
            sleep(500);
            robot.driveRobot(FORWARD_SPEED, 0);
            sleep(500);
            robot.driveRobot(0, FORWARD_SPEED);
            sleep(500);
        }
    }
}