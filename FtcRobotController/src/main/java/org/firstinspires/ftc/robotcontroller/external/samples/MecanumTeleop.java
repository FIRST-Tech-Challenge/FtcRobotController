
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum: Teleop", group="Mecanum")
public class MecanumTeleop extends LinearOpMode {

    HardwareMecanum robot = new HardwareMecanum();

    @Override
    public void runOpMode() {
        double x1; // left/right
        double y1; // front/back


        robot.init(hardwareMap);



        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            y1 = -gamepad1.left_stick_y;
            x1 = gamepad1.right_stick_x;

            robot.frontLeftDrive.setPower(y1);
            robot.frontRightDrive.setPower(y1);
            robot.backLeftDrive.setPower(y1);
            robot.backRightDrive.setPower(y1);

            // Output the safe vales to the motor drives.

            telemetry.addData("x1",  "%.2f", x1);
            telemetry.addData("y1", "%.2f", y1);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
