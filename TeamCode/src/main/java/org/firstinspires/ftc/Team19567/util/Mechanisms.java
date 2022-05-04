package org.firstinspires.ftc.Team19567.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.BALANCE_COEFFICIENT;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.PPR_RATIO;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.MAX_POS;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.POTENTIOMETER_COEFFICIENT;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.POW_COEFFICIENT;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.BALANCE_SERVO_DEFAULT;

import androidx.annotation.NonNull;

public class Mechanisms {
    public DcMotor armDC;
    public DcMotor carouselLeft;
    public DcMotor carouselRight;
    public DcMotor intakeDC;
    public Servo balanceServo;
    public Servo releaseServo;
    public AnalogInput potentiometer;
    public Telemetry telemetry;

    public Mechanisms(@NonNull HardwareMap hardwareMap, Telemetry t) {
        armDC = hardwareMap.get(DcMotor.class,"armDC");
        carouselLeft = hardwareMap.get(DcMotor.class,"carouselLeft");
        carouselRight = hardwareMap.get(DcMotor.class,"carouselRight");
        intakeDC = hardwareMap.get(DcMotor.class,"intakeDC");
        balanceServo = hardwareMap.get(Servo.class,"balanceServo");
        releaseServo = hardwareMap.get(Servo.class,"releaseServo");
        potentiometer = hardwareMap.get(AnalogInput.class,"potentiometer");
        telemetry = t;
    }

    public void setModes() {
        armDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDC.setTargetPosition(0);
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDC.setDirection(DcMotor.Direction.REVERSE);
        balanceServo.setDirection(Servo.Direction.REVERSE);
        intakeDC.setDirection(DcMotor.Direction.REVERSE);
    }

    public void rotateArm(int pos, double speed) {
        armDC.setPower(speed);
        armDC.setTargetPosition(Range.clip(pos,0,1900));
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void rotateArm(int pos) {
        armDC.setPower(1.0);
        armDC.setTargetPosition(Range.clip(pos,0,1900));
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
        releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);
        rotateCarousel(0);
        rotateArm(0,Utility_Constants.GOING_DOWN_POWER);
    }

    @Deprecated
    public void sharedHub() {
        telemetry.addData("Mechanisms","Note: use Mechanisms.firstLevel() instead.");
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
        //double balanceServoPos = Range.clip(Math.pow(Range.clip((3.31-potentiometer.getVoltage())/POTENTIOMETER_COEFFICIENT,0,1),POW_COEFFICIENT)+BALANCE_SERVO_DEFAULT,balanceServo.MIN_POSITION,balanceServo.MAX_POSITION);
        //balanceServo.setPosition(balanceServoPos);
        //telemetry.addData("Mechanisms","balanceServoPos(%.3f)",balanceServoPos); //TODO: TUNE THIs
        //telemetry.addData("Mechanisms","Potentiometer Voltage(%.3f)",potentiometer.getVoltage());
        //telemetry.addData("Mechanisms","Arm Position",armDC.getCurrentPosition());
        balanceServo.setPosition(Range.clip((armDC.getCurrentPosition()/(BALANCE_COEFFICIENT*PPR_RATIO)),balanceServo.MIN_POSITION,balanceServo.MAX_POSITION)); //TODO: TUNE THIS
    }
}