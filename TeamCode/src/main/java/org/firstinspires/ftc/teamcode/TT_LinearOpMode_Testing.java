
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

@TeleOp(name = "Linear OpMode Testing", group = "Techtonics")
public class TT_LinearOpMode_Testing extends LinearOpMode {
    private TT_RobotHardware robot = new TT_RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.leftFrontDrive.setPower(.25);
            } else {
                robot.leftFrontDrive.setPower(0);
            }
            if (gamepad1.b) {
                robot.leftBackDrive.setPower(.25);
            } else {
                robot.leftBackDrive.setPower(0);
            }
            if (gamepad1.x) {
                robot.rightBackDrive.setPower(.25);
            } else {
                robot.rightBackDrive.setPower(0);
            }
            if (gamepad1.y) {
                robot.rightFrontDrive.setPower(.25);
            } else {
                robot.rightFrontDrive.setPower(0);
            }
            robot.displayTelemetry();
        }
    }
}
