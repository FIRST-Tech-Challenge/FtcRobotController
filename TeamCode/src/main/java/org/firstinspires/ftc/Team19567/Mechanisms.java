package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms {
    private DcMotor armDC = null;
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
        balanceServo.setPosition(armDC.getCurrentPosition()/1000);
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
}
