package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveController {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    public DriveController (DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;

        this.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update (double x, double y, double r) {
        leftFront.setPower(activationFunction(getTransXPrime(x, y) + r));
        rightFront.setPower(activationFunction(getTransYPrime(x, y) - r));
        leftBack.setPower(activationFunction(getTransYPrime(x, y) + r));
        rightBack.setPower(activationFunction(getTransXPrime(x, y) - r));
    }

    private double getTransXPrime (double x, double y) {
        return ((x + y) / Math.sqrt(2));
    }

    private double getTransYPrime (double x, double y) {
        return ((y - x) / Math.sqrt(2));
    }

    private double activationFunction (double a) {
//        if (a <= 1 && a >= -1) return a;
//        else if (a > 1) return 1;
//        else return -1;

//         optionally
         return Math.tanh(a);
    }

}
