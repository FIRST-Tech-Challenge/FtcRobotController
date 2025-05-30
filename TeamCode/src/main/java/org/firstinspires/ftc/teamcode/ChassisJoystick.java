package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Chassis Joystick TeleOp", group="TeleOp")
public class ChassisJoystick extends LinearOpMode {

    // Declare motor variables
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map the motors from the hardware configuration. Ensure your configuration names match.
        leftMotor  = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // Reverse one motor if needed so that both motors move the robot forward when set to positive power.
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            // Read the joystick's Y and X values.
            // The Y-axis is inverted (pushing up returns a negative value) so we negate it.
            double drive = -gamepad1.left_stick_y;
            // X-axis value will control turning.
            double turn = gamepad1.left_stick_x;

            // For a differential drive:
            // Left motor gets the forward power plus turning adjustment.
            // Right motor gets the forward power minus turning adjustment.
            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Apply the power to the motors.
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Send telemetry data to the driver station
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

            // Idle to let other processes run
            idle();
        }
    }
}