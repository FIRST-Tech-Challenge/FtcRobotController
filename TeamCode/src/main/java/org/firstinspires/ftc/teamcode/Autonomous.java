package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Autonomous  extends LinearOpMode{

    private RobotHardware robot = new RobotHardware(this);

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
            double speed = 0.2;

            robot.driveRobot(speed, 0);
            sleep(500);
            robot.driveRobot(0, speed);
            sleep(500);
            robot.driveRobot(speed, 0);
            sleep(500);
            robot.driveRobot(0, speed);
            sleep(500);
        }
    }
}