package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TeleDriveController {

    private static final double EPSILON = 0.001;
    private DcMotor p_leftFront, p_rightFront, p_leftBack, p_rightBack;
    private boolean p_smoothActivation = false;

    public TeleDriveController(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        this.p_leftFront = leftFront;
        this.p_rightFront = rightFront;
        this.p_leftBack = leftBack;
        this.p_rightBack = rightBack;

        this.p_leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.p_rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.p_leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        this.p_rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update (double x, double y, double r) {
        p_leftFront.setPower(getPower(x, y, r, true, true));
        p_rightFront.setPower(getPower(x, y, r, false, true));
        p_leftBack.setPower(getPower(x, y, r, true, false));
        p_rightBack.setPower(getPower(x, y, r, false, false));
    }

    public void setSmoothActivation(boolean p_smoothActivation) {
        this.p_smoothActivation = p_smoothActivation;
    }

    private double getPower (double x, double y, double r, boolean isLeft, boolean isFront) {
        double ret;
        if ((isLeft && isFront) || (!isLeft && !isFront))
            ret = activationFunction(getTransXPrime(x, y) + (isLeft ? (r) : (-1 * r)));
        else
            ret = activationFunction(getTransYPrime(x, y) + (isLeft ? (r) : (-1 * r)));

        if (Math.abs(ret) < EPSILON) {
            return 0;
        }
        return ret;
    }

    private double getTransXPrime (double x, double y) {
        return ((x + y) / Math.sqrt(2));
    }

    private double getTransYPrime (double x, double y) {
        return ((y - x) / Math.sqrt(2));
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
