package org.firstinspires.ftc.teamcode.Swerve;
/*
 *-* Control configuration
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Swerve Automation", group = "Swerve")
public class SwerveAutomation extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    ComponentSwerve robot = new ComponentSwerve(this);

    @Override
    public void runOpMode() {
        double drive = 0;
        int turn = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick Y moves the robot rfwd and back, the X stick turns left and right.
            if (-gamepad1.left_stick_y > 0) {
                drive = Math.pow(-gamepad1.left_stick_y, 2);
            } else {
                drive = -Math.pow(-gamepad1.left_stick_y, 2);
            }


            //telemetry.addData("drive encoder",robot.encCntD(drive));
            turn = (int) (gamepad1.left_stick_x * 90);
            // determine drive action
            if (Math.abs(turn) > 1) {
                robot.driveRobot(turn);
            } else {
                robot.driveRobot(drive);
            }
        }
    }
}

