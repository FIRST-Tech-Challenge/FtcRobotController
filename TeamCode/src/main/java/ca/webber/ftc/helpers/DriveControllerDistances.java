package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveControllerDistances {

    private DcMotor p_leftFront, p_rightFront, p_leftBack, p_rightBack;
    private double[] p_movementVector = new double[2];
    private double p_rotationAmount = 0;
    private boolean p_smoothActivation = false;

    public DriveControllerDistances(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        /*
        * Sets the motors of the drive controller during initialization.
        */

        this.p_leftFront = leftFront;
        this.p_rightFront = rightFront;
        this.p_leftBack = leftBack;
        this.p_rightBack = rightBack;

        this.p_leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.p_rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.p_leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        this.p_rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void translate (double x, double y) {
        /*
        * Sets the desired movement vector in meters.
        */

        p_movementVector[0] = x;
        p_movementVector[1] = y;
    }

    public void rotate (double r) {
        /*
        * Sets the amount to rotate in radians. CCW is positive.
        */

        p_rotationAmount = r;
    }

    public void update () {
        /*
        * Changes the
        */
    }
}
