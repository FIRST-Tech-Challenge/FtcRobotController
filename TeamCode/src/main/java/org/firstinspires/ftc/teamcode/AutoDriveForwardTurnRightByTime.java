package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive Forward then Right by time", group="Robot")
public class AutoDriveForwardTurnRightByTime extends LinearOpMode{

    private RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();
    static final double FORWARD_SPEED = 0.2;
    static final int forwardPeriod = 5000; //in milli seconds
    static final int turnPeriod = 3000;
    @Override
    public void runOpMode() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.setDrivePower(0.5, 0.5);
        runtime.reset();
        // run until for 5000 ms
        while (opModeIsActive() && runtime.milliseconds() <forwardPeriod) {
            telemetry.addData("moving forward", "");
            telemetry.update();
        }

        robot.setDrivePower(0, 0);
        sleep(1000);

        robot.driveRobot(0.5, 1);
        runtime.reset();
        // run until for 3000 ms
        while (opModeIsActive() && runtime.milliseconds() <turnPeriod) {
            telemetry.addData("moving forward", "");
            telemetry.update();
        }
        robot.setDrivePower(0, 0);
        sleep(1000);    }
}