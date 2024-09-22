package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 Class designed to provide helper methods to operate mecanum wheels
 */
public class Mecanum {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    /**
     * Creates the Mecanum object. Sets private fields and configures motor directions.
     * @param frontRight Front Right Motor Object
     * @param frontLeft Front Left Motor Object
     * @param backRight Back Right Motor Object
     * @param backLeft Back Left Motor Object
     * @return A new Mecanum object
     */
    public static Mecanum Init(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft) {
        Mecanum m = new Mecanum();
        m.fl = frontLeft;
        m.fr = frontRight;
        m.br = backRight;
        m.bl = backLeft;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        m.fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m.br.setDirection(DcMotorSimple.Direction.REVERSE);
        return m;
    }


    /**
     * Analyses gamepad and sets power of motors appropriately
     * @param gp Gamepad object
     */
    public void Move(Gamepad gp) {
        double y = -gp.left_stick_y;
        double x = gp.left_stick_x * 1.1;
        double rx = gp.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }
}
