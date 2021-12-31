package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Mechanisms {
    public DcMotor armDC = null;
    private DcMotor carouselLeft = null;
    private DcMotor carouselRight = null;
    private DcMotor intakeDC = null;
    private Servo balanceServo = null;
    private Servo releaseServo = null;
    public Mechanisms(DcMotor arm, DcMotor left, DcMotor right,
                              DcMotor intake, Servo balance, Servo release) {
        armDC = arm;
        carouselLeft = left;
        carouselRight = right;
        intakeDC = intake;
        balanceServo = balance;
        releaseServo = release;
    }
    public void rotateArm(int pos, double speed) {
        armDC.setPower(speed);
        armDC.setTargetPosition(pos);
    }
    public void maintainBalance() {
        balanceServo.setPosition(Range.clip(armDC.getCurrentPosition()/500,balanceServo.MIN_POSITION,balanceServo.MAX_POSITION)); //TODO: TUNE THIS
    }
    public void rotateCarousel(double speed, boolean left) {
        if(left) carouselLeft.setPower(speed);
        else carouselRight.setPower(speed);
    }
    public void moveIntake(double speed) {
        intakeDC.setPower(speed);
    }
    public void releaseServoMove(double pos) {
        releaseServo.setPosition(pos);
    }

    public void reset() {
        releaseServo.setPosition(0);
        carouselLeft.setPower(0);
        carouselRight.setPower(0);
        intakeDC.setPower(0);
        armDC.setTargetPosition(0);
        balanceServo.setPosition(0);
    }
}
