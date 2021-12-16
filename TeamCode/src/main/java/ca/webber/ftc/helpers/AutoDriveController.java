package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AutoDriveController {

    private static final double p_EPSILON = 0.001;
    private static final double p_MOVEMENT_SCALE = 100, p_ROTATION_SCALE = 100;
    private DcMotor p_leftFront, p_rightFront, p_leftBack, p_rightBack;
    private boolean p_smoothActivation = false;

    public AutoDriveController (DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        // assigns all the motors
        p_leftFront = leftFront;
        p_rightFront = rightFront;
        p_leftBack = leftBack;
        p_rightBack = rightBack;

        // sets the forward direction for all of the motors
        p_leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        p_rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        p_leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        p_rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // sets the zero power behaviour for all the motors
        p_leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        p_rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        p_leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        p_rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        p_rightBack.getCurrentPosition();
    }

    public boolean setMovement (double x, double y) {
        if (isBusy()) {
            return false;
        }

        p_leftFront.setTargetPosition(
                p_leftFront.getCurrentPosition() + (int) (p_MOVEMENT_SCALE * getPower(x, y, 0, true, true))
        );
        p_rightFront.setTargetPosition(
                p_rightFront.getCurrentPosition() + (int) (p_MOVEMENT_SCALE * getPower(x, y, 0, false, true))
        );
        p_leftBack.setTargetPosition(
                p_leftBack.getCurrentPosition() + (int) (p_MOVEMENT_SCALE * getPower(x, y, 0, true, false))
        );
        p_rightBack.setTargetPosition(
                p_rightBack.getCurrentPosition() + (int) (p_MOVEMENT_SCALE * getPower(x, y, 0, false, false))
        );

        return true;
    }

    public boolean setRotation (double r) {
        if (isBusy()) {
            return false;
        }

        p_leftFront.setTargetPosition(
                p_leftFront.getCurrentPosition() + (int) (p_ROTATION_SCALE * getPower(0, 0, r, true, true))
        );
        p_rightFront.setTargetPosition(
                p_rightFront.getCurrentPosition() + (int) (p_ROTATION_SCALE * getPower(0, 0, r, false, true))
        );
        p_leftBack.setTargetPosition(
                p_leftBack.getCurrentPosition() + (int) (p_ROTATION_SCALE * getPower(0, 0, r, true, false))
        );
        p_rightBack.setTargetPosition(
                p_rightBack.getCurrentPosition() + (int) (p_ROTATION_SCALE * getPower(0, 0, r, false, false))
        );

        return true;
    }

    public boolean isBusy () {
        if (
            p_leftFront.isBusy() ||
            p_rightFront.isBusy() ||
            p_leftBack.isBusy() ||
            p_rightBack.isBusy()
        )
            return true;
        return false;
    }

    public void setSmoothActivation (boolean p_smoothActivation) {
        this.p_smoothActivation = p_smoothActivation;
    }

    private double getPower (double x, double y, double r, boolean isLeft, boolean isFront) {
        double ret;
        if ((isLeft && isFront) || (!isLeft && !isFront))
            ret = activationFunction(getTransXPrime(x, y) + (isLeft ? (r) : (-1 * r)));
        else
            ret = activationFunction(getTransYPrime(x, y) + (isLeft ? (r) : (-1 * r)));

        if (Math.abs(ret) < p_EPSILON) {
            return 0;
        }
        return ret;
    }

    private double getTransXPrime (double x, double y) {
        return ((x - y) / Math.sqrt(2));
    }

    private double getTransYPrime (double x, double y) {
        return ((-y - x) / Math.sqrt(2));
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
