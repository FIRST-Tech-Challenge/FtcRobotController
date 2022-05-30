package org.firstinspires.ftc.Team19567.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
/*
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.BALANCE_COEFFICIENT;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.PPR_RATIO;
 */

import static org.firstinspires.ftc.Team19567.util.Utility_Constants.MAX_POS;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.POTENTIOMETER_COEFFICIENT;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.POW_COEFFICIENT;
import static org.firstinspires.ftc.Team19567.util.Utility_Constants.BALANCE_SERVO_DEFAULT;

/**
 * Custom mechanisms class with functions for basically every mechanism used. <br>
 * Really useful functions are located here, especially stuff like {@link #maintainBalance()} or {@link #setModes()}.
 * TODO: Develop more sort-of-subsystem work like this (and definitely actual subsystems).
 */
public class Mechanisms {
    //Declare hardware and other members (i.e. telemetry)
    public DcMotor armDC;
    public DcMotor carouselLeft;
    public DcMotor carouselRight;
    public DcMotor intakeDC;
    public Servo balanceServo;
    public Servo releaseServo;
    public AnalogInput potentiometer;
    public Telemetry telemetry;

    /**
     * "Constructs the mechanisms from the robot's hardware"
     * @param hardwareMap The robot's hardwareMap that will be used to initialize all of the hardware
     * @param t Telemetry used to broadcast debug info
     */
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
    /**
     * Sets all of the modes of the hardware to what they need to be for standard operation
     */
    public void setModes() {
        armDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDC.setTargetPosition(0);
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDC.setDirection(DcMotor.Direction.REVERSE);
        balanceServo.setDirection(Servo.Direction.REVERSE);
        intakeDC.setDirection(DcMotor.Direction.REVERSE);
    }
    /**
     * Rotates the arm at a designed speed and to a designated position
     * @param pos Position to rotate the arm to
     * @param speed Speed to run the arm at
     */
    public void rotateArm(int pos, double speed) {
        armDC.setPower(speed);
        armDC.setTargetPosition(Range.clip(pos,0,MAX_POS)); //Prevents the arm from "setting itself too far" and potentially breaking stuff
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    /**
     * Rotates the arm at max speed to a designated position
     * @param pos Position to rotate the arm to
     */
    public void rotateArm(int pos) {
        armDC.setPower(1.0);
        armDC.setTargetPosition(Range.clip(pos,0,MAX_POS));
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    /** Rotates either the left or right carousel at a designated speed
     * @param speed Speed to spin the designated motor at
     * @param left Whether to spin the left motor or the right motor (true => left)
     */
    public void rotateCarousel(double speed, boolean left) {
        if(left) carouselLeft.setPower(speed);
        else carouselRight.setPower(speed);
    }
    /**
     * Rotates both carousels at a designated speed
     * @param leftSpeed Speed to rotate the left carousel at; the right carousel will opposite
     */
    public void rotateCarousel(double leftSpeed) {
        carouselLeft.setPower(leftSpeed);
        carouselRight.setPower(-leftSpeed);
    }
    /**
     * Moves the intake at the designated speed
     * @param speed Speed to move the intake at
     */
    public void moveIntake(double speed) {
        intakeDC.setPower(speed);
    }
    /**
     * Moves the release servo to the designated position
     * @param pos Position to move the release servo
     */
    public void releaseServoMove(double pos) {
        //Prevents the release servo from moving "out of bounds"
        releaseServo.setPosition(Range.clip(pos,releaseServo.MIN_POSITION,releaseServo.MAX_POSITION));
    }
    /** Moves the balance servo to the designated position
     * @param pos Position to move the balance servo
     */
    public void balanceServoMove(double pos) {
        balanceServo.setPosition(Range.clip(pos,balanceServo.MIN_POSITION,balanceServo.MAX_POSITION));
    }
    /** Resets all mechanisms to their default positions and speeds */
    public void reset() {
        releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);
        rotateCarousel(0);
        rotateArm(0,Utility_Constants.GOING_DOWN_POWER);
    }
    /** Effectively equivalent to firstLevel() */
    @Deprecated
    public void sharedHub() {
        telemetry.addData("Mechanisms","Use Mechanisms.firstLevel() instead of Mechanisms.sharedHub()");
        firstLevel();
    }
    /** Function to rotate the arm to the first level */
    @Deprecated
    public void firstLevel() {
        rotateArm(1000,0.9);
    }
    /** Function to rotate the arm to the second level */
    @Deprecated
    public void secondLevel() {
        rotateArm(800,0.9);
    }
    /** Function to rotate the arm to the third level */
    @Deprecated
    public void thirdLevel() {
        rotateArm(600,0.9);
    }
    /**
     * Really useful function to rotate the balance servo in order to balance the release box as the arm moves.
     */
    public void maintainBalance() {
        /*
        Balancing mechanism using the potentiometer's voltage reading
        Range.clip() is used to prevent the servo's position from exceeding the servo's actual operating limits; the balancing is not linear,
        as it needs to be less aggressive as the arm rotates to the bottom.
        The math here is pretty self-explanatory.
         */
        double balanceServoPos = Range.clip(Math.pow(Range.clip((3.31-potentiometer.getVoltage())/POTENTIOMETER_COEFFICIENT,0,1),POW_COEFFICIENT)+BALANCE_SERVO_DEFAULT,balanceServo.MIN_POSITION,balanceServo.MAX_POSITION);
        balanceServo.setPosition(balanceServoPos);

        /*
        Debug telemetry:

        telemetry.addData("Mechanisms","balanceServoPos(%.3f)",balanceServoPos); //TODO: TUNE THIS
        telemetry.addData("Mechanisms","Potentiometer Voltage(%.3f)",potentiometer.getVoltage());
        telemetry.addData("Mechanisms","Arm Position",armDC.getCurrentPosition());
        */

        //Balancing mechanism using the arm's encoders rather than the potentiometer's read voltage
        //balanceServo.setPosition(Range.clip((armDC.getCurrentPosition()/(BALANCE_COEFFICIENT*PPR_RATIO)),balanceServo.MIN_POSITION,balanceServo.MAX_POSITION)); //TODO: TUNE THIS
    }
}