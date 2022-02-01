package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//
public class Mechanisms {
    public DcMotor armDC = null;
    private DcMotor carouselLeft = null;
    private DcMotor carouselRight = null;
    private DcMotor intakeDC = null;
    private Servo balanceServo = null;
    private Servo releaseServo = null;
    private Telemetry telemetry = null;

    public Mechanisms(HardwareMap hardwareMap, Telemetry t) {
        armDC = hardwareMap.get(DcMotor.class,"armDC");
        carouselLeft = hardwareMap.get(DcMotor.class,"carouselLeft");
        carouselRight = hardwareMap.get(DcMotor.class,"carouselRight");
        intakeDC = hardwareMap.get(DcMotor.class,"intakeDC");
        balanceServo = hardwareMap.get(Servo.class,"balanceServo");
        releaseServo = hardwareMap.get(Servo.class,"releaseServo");
        telemetry = t;
    }

    public Mechanisms() {}

    public void rotateArm(int pos, double speed) {
        armDC.setPower(speed);
        armDC.setTargetPosition(Range.clip(pos,0,1000));
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void rotateArm(int pos) {
        armDC.setPower(0.5);
        armDC.setTargetPosition(Range.clip(pos,0,1000));
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void rotateCarousel(double speed, boolean left) {
        if(left) carouselLeft.setPower(speed);
        else carouselRight.setPower(speed);
    }

    public void rotateCarousel(double leftSpeed) {
        carouselLeft.setPower(leftSpeed);
        carouselRight.setPower(-leftSpeed);
    }

    public void moveIntake(double speed) {
        intakeDC.setPower(speed);
    }

    public void releaseServoMove(double pos) {
        releaseServo.setPosition(Range.clip(pos,releaseServo.MIN_POSITION,releaseServo.MAX_POSITION));
    }

    public void balanceServoMove(double pos) {
        balanceServo.setPosition(Range.clip(pos,balanceServo.MIN_POSITION,balanceServo.MAX_POSITION));
    }

    public void reset() {
        releaseServo.setPosition(0);
        carouselLeft.setPower(0);
        carouselRight.setPower(0);
        intakeDC.setPower(0);
        armDC.setTargetPosition(0);
        balanceServo.setPosition(0);
    }

    public void sharedHub() {
        telemetry.addData("Mechanoosism","uurbadusefirstlevel");
        firstLevel();
    }

    public void firstLevel() {
        rotateArm(1000,0.9);
    }

    public void secondLevel() {
        rotateArm(800,0.9);
    }

    public void thirdLevel() {
        rotateArm(600,0.9);
    }

    public void maintainBalance() {
        balanceServo.setPosition(Range.clip((armDC.getCurrentPosition())/1050.5,balanceServo.MIN_POSITION,balanceServo.MAX_POSITION)); //TODO: TUNE THIS
    }
}
