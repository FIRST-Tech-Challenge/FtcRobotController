package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 Class designed to provide helper methods to operate mecanum wheels
 */
public class Mecanum {
    private DcMotorSimple fr, fl, br, bl;

    public double PowerMultiplier = 1;

    /**
     * Creates the Mecanum object. Sets private fields and configures motor directions.
     * @param frontRight Front Right Motor Object
     * @param frontLeft Front Left Motor Object
     * @param backRight Back Right Motor Object
     * @param backLeft Back Left Motor Object
     * @return A new Mecanum object
     */
    public static Mecanum Init(DcMotorSimple frontRight, DcMotorSimple frontLeft, DcMotorSimple backRight, DcMotorSimple backLeft, double power) {
        Mecanum m = new Mecanum();
        m.fr = frontRight;
        m.fl = frontLeft;
        m.br = backRight;
        m.bl = backLeft;
        m.PowerMultiplier = power;
        return m;
    }

    /**
     * Analyses gamepad and sets power of motors appropriately
     * @param gp Gamepad object
     */
    public double[] Move(Gamepad gp, Telemetry telemetry) {
        if(!(PowerMultiplier > 0 && PowerMultiplier <= 1)) {
            telemetry.addLine("Power Multiplier should be between 0 and 1");
            return new double [] { 0, 0, 0, 0 };
        }

        double y = -gp.left_stick_y;
        double x = gp.left_stick_x;
        double rx = gp.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontRightPower = (y - x - rx) / denominator;
        double frontLeftPower = (y + x + rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;

        fr.setPower(frontRightPower * PowerMultiplier);
        fl.setPower(frontLeftPower * PowerMultiplier);
        br.setPower(backRightPower * PowerMultiplier);
        bl.setPower(backLeftPower * PowerMultiplier);

        return new double[]{ frontRightPower, frontLeftPower, backRightPower, backLeftPower };
    }
}
