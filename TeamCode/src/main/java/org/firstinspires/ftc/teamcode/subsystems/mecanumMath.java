package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class mecanumMath {

    public static void driveMecanum(
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight,
            Gamepad gamepad
    ) {

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rotation = gamepad.right_stick_x;

        double frontLeftPower  = y + x + rotation;
        double frontRightPower = y - x - rotation;
        double backLeftPower   = y - x + rotation;
        double backRightPower  = y + x - rotation;

        double maxPower = Math.max(1.0,
                Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                )
        );

        frontLeft.setPower(frontLeftPower / maxPower);
        frontRight.setPower(frontRightPower / maxPower);
        backLeft.setPower(backLeftPower / maxPower);
        backRight.setPower(backRightPower / maxPower);
    }
}
