package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeController {

    private static final double p_EPSILON = 0.001;
    private DcMotor p_leftIntake, p_rightIntake;
    private boolean p_smoothActivation = false;

    public IntakeController (DcMotor leftIntake, DcMotor rightIntake) {
        p_leftIntake = leftIntake;
        p_rightIntake = rightIntake;

        p_rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        p_leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        p_leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        p_rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update (double q) {
        p_leftIntake.setPower(getPower(q));
        p_rightIntake.setPower(getPower(q));
    }

    private double getPower (double q) {
        double ret;

        ret = activationFunction(q);

        if (Math.abs(ret) < p_EPSILON) {
            return 0;
        }
        return ret;
    }

    private double activationFunction (double a) {
        if (p_smoothActivation) {
            return Math.tanh(a);
        } else {
            if (a > 1)  return 1;
            if (a < -1) return -1;
            return a;
        }
    }
}
