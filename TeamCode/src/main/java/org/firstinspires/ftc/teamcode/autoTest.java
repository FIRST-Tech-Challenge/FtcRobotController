package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp (name = "Adam :)")
public class autoTest extends OpMode {

    private DcMotor backLeft; //making back left motor
    private DcMotor frontLeft; //making front left motor
    private DcMotor backRight; //making back right motor
    private DcMotor frontRight; //making front right motor
    private static double pY1; //making power variable
    private static double pX1; //making power variable
    private static double iY1; //making intake variable
    private static double iX1; //making intake variable
//    private DcMotor intake; //making intake motor

    @Override
    public void loop() {
        if (pX1 >= 0.3 && pY1 >= 0.3) { //runs goForwardRight when right stick is pushed right and up
            goForwardRight();
        }
        if (pX1 <= -0.3 && pY1 >= 0.3) { //runs goForwardLeft when right stick is pushed left and up
            goForwardLeft();
        }
        if (pX1 >= 0.3 && pY1 <= -0.3) { //runs goBackwardRight when right stick is pushed right and down
            goBackwardRight();
        }
        if (pX1 <= -0.3 && pY1 <= -0.3) { //runs goBackwardLeft when right stick is pushed left and down
            goBackwardLeft();
        }
        if (pY1 >= 0.3) { //runs goForward when right stick is pushed up
            goForward();
        }
        if (pY1 <= -0.3) { //runs goBackward when right stick is pushed down
            goBackward();
        }
        if (pX1 >= 0.3) { //runs goRight when right stick is pushed right
            goRight();
        }
        if (pX1 <= 0.3) { // runs goLeft when right stick is pushed left
            goLeft();
        }
    } //idk

    @Override
    public void init() { //initialize function
        backLeft = hardwareMap.get(DcMotor.class, "backLeft"); //mapping motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); //reversing left side cuz mecanum wheels
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        pY1 = gamepad1.right_stick_y; //mapping power variable to right stick up and down
        pX1 = gamepad1.right_stick_x; //mapping power variable to right stick left and right
    }

    public void goForward() { //go forward
        backLeft.setPower(pY1);
        frontLeft.setPower(pY1);
        backRight.setPower(pY1);
        frontRight.setPower(pY1);
    }

    public void goBackward() { //go backward
        backLeft.setPower(-1 * pY1);
        frontLeft.setPower(-1 * pY1);
        backRight.setPower(-1 * pY1);
        frontRight.setPower(-1 * pY1);
    }

    public void goRight() { //go right
        frontRight.setPower(pY1);
        frontLeft.setPower((pY1));
        backRight.setPower(pY1);
        backLeft.setPower(pY1);
    }

    public void goLeft() { //go left
        frontLeft.setPower(pY1);
        frontRight.setPower(pY1);
        backLeft.setPower(pY1);
        backRight.setPower(pY1);
    }

    public void goForwardRight() { // go forward and right
        frontLeft.setPower(pY1);
        backRight.setPower(pY1);
    }

    public void goForwardLeft() { //go forward and left
        frontRight.setPower(pY1);
        backLeft.setPower(pY1);
    }

    public void goBackwardRight() { //go backward and right
        frontLeft.setPower(pY1);
        backRight.setPower(pY1);
    }

    public void goBackwardLeft() { //go backward and left
        frontRight.setPower(pY1);
        backLeft.setPower(pY1);
    }
}
    /* public void intakeSpinIn() { //spin intake wheel to put brick in
        intake.setPower(iY1);
    }
    public void intakeSpinOut() { //spin intake wheel to spit brick out
        intake.setPower(iY1);
    }
}*/
