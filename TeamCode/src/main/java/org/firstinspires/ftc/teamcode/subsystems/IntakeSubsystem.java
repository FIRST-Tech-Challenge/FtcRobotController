package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

import java.lang.Math;

public class IntakeSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;

    private final DcMotorEx horizontalSlideLeftMotor;
    private final DcMotorEx horizontalSlideRightMotor;

    private final ServoEx wristServo;
    private final CRServo activeIntakeServo;

    private double speedMultiplier = 1.0;

    private double leftHorizontalSlideTicks;
    private double rightHorizontalSlideTicks;
    private double horizontalSlideMaximumDistanceTraveled;
    private double horizontalSlideMinimumDistanceTraveled;

    private final OpMode opMode;

    private enum intakeSubsystemState{
        EXTENDING,
        GETTING_BLOCK,
        RETRACTING,
        RELEASING_BLOCK,
        AT_BAY
    }

    public IntakeSubsystem(HardwareMap hardwareMap,OpMode opMode){
        this.opMode= opMode;//a little fix so that the subsystem itself can add things into telemetry

        // intake arms motor creation
        horizontalSlideLeftMotor = hardwareMap.get(DcMotorEx.class,IntakeConstants.HORIZONTAL_SLIDE_LEFT_MOTOR_NAME);
        horizontalSlideRightMotor = hardwareMap.get(DcMotorEx.class,IntakeConstants.HORIZONTAL_SLIDE_RIGHT_MOTOR_NAME);

        // wrist servo creation
        wristServo = new SimpleServo(hardwareMap,IntakeConstants.INTAKE_WRIST_SERVO_NAME, IntakeConstants.INTAKE_WRIST_SERVO_MIN_ANGLE,IntakeConstants.INTAKE_WRIST_SERVO_MAX_ANGLE);
        activeIntakeServo = hardwareMap.get(CRServo.class, IntakeConstants.ACTIVE_INTAKE_SERVO_NAME);

        // intake arms direction
        horizontalSlideLeftMotor.setDirection(Constants.IntakeConstants.HORIZONTAL_SLIDE_LEFT_MOTOR_DIRECTION);
        horizontalSlideRightMotor.setDirection(Constants.IntakeConstants.HORIZONTAL_SLIDE_RIGHT_MOTOR_DIRECTION);

        // brake checker if nothing is pressed
        horizontalSlideLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlideRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

    }

    public void updateHorizontalSlideDistance() {
        leftHorizontalSlideTicks = horizontalSlideLeftMotor.getCurrentPosition();
        rightHorizontalSlideTicks = horizontalSlideRightMotor.getCurrentPosition();
    }

    public void resetEncoders() {
        // reset and stop arm motors
        horizontalSlideLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // start arm motors
        horizontalSlideLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlideRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void changeHorizontalSlideSpeedMultiplier(){
        if(speedMultiplier == 1){
            speedMultiplier = 0.1;
        } else{
            speedMultiplier = 1;
        }
    }

    public double getDistanceTraveled(){
        if(leftHorizontalSlideTicks >= rightHorizontalSlideTicks){
            horizontalSlideMaximumDistanceTraveled = (leftHorizontalSlideTicks / Constants.MOTOR_TICKS_PER_REVOLUTION) * Constants.SLIDE_PULLEY_CIRCUMFERENCE;
            horizontalSlideMinimumDistanceTraveled = (rightHorizontalSlideTicks / Constants.MOTOR_TICKS_PER_REVOLUTION) * Constants.SLIDE_PULLEY_CIRCUMFERENCE;
        } else{
            horizontalSlideMaximumDistanceTraveled = (rightHorizontalSlideTicks / Constants.MOTOR_TICKS_PER_REVOLUTION) * Constants.SLIDE_PULLEY_CIRCUMFERENCE;
        }

        return (horizontalSlideMaximumDistanceTraveled + horizontalSlideMinimumDistanceTraveled) / 2;
    }

    /*
     * gets input from controllers and retracts/extends the arms
     */
    public void runHorizontalSlides(double input){

        input = Math.abs(input) >= Constants.DriveConstants.DEADZONE ? input : 0;

        if (horizontalSlideMaximumDistanceTraveled >= IntakeConstants.MAXIMUM_FORWARD_EXTENSION) {
            input = -Math.abs(input);
        }
        if(horizontalSlideMinimumDistanceTraveled <= IntakeConstants.MINIMUM_BACKWARD_EXTENSION){
            input = Math.abs(input);
        }

        horizontalSlideLeftMotor.setPower(Range.clip(input, -1, 1) * speedMultiplier);
        horizontalSlideRightMotor.setPower(Range.clip(input, -1, 1) * speedMultiplier);
    }

    public void extentHorizontalSlides(){                                                               //Tolerance = 2
        while(horizontalSlideMaximumDistanceTraveled <= (IntakeConstants.MAXIMUM_FORWARD_EXTENSION - IntakeConstants.EXTENSION_TOLERANCE_INCHES) ){
            horizontalSlideLeftMotor.setPower(1);
            horizontalSlideRightMotor.setPower(1);
        }
        while(horizontalSlideMaximumDistanceTraveled <= IntakeConstants.MAXIMUM_FORWARD_EXTENSION) {
            horizontalSlideLeftMotor.setPower(0.2);
            horizontalSlideRightMotor.setPower(0.2);
        }
        servoDownPosition();
        horizontalSlideLeftMotor.setPower(0);
        horizontalSlideRightMotor.setPower(0);
    }

    public void retractHorizontalSlides(){
        while(horizontalSlideMaximumDistanceTraveled >= (IntakeConstants.MINIMUM_BACKWARD_EXTENSION + 2) ){
            horizontalSlideLeftMotor.setPower(-1);
            horizontalSlideRightMotor.setPower(-1);
        }
        while(horizontalSlideMaximumDistanceTraveled >= IntakeConstants.MINIMUM_BACKWARD_EXTENSION) {
            horizontalSlideLeftMotor.setPower(-0.2);
            horizontalSlideRightMotor.setPower(-0.2);
        }
        horizontalSlideLeftMotor.setPower(0);
        horizontalSlideRightMotor.setPower(0);
    }

    public void ActiveIntakeServoIn(){
        activeIntakeServo.setPower(1.0);
    }

    public void ActiveIntakeServoOut(){
        activeIntakeServo.setPower(-1.0);
    }

    public void stopActiveIntakeServo(){
        activeIntakeServo.setPower(0.0);
    }

    /*
    moves the wrist servo into position to retract
    */
    public void servoUpPosition(){
        wristServo.turnToAngle(IntakeConstants.INTAKE_WRIST_SERVO_UP_POSITION);
    }

    /*
    moves the wrist servo into position to pick up samples
     */
    public void servoDownPosition(){
        wristServo.turnToAngle(IntakeConstants.INTAKE_WRIST_SERVO_DOWN_POSITION);
    }


}
