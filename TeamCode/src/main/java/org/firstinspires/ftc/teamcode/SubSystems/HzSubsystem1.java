package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of HzIntake Class <BR>
 *
 * HzIntake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>INTAKE_MOTOR_STATE for telling if intake is running, stopped, or reversing </emsp> <BR>
 *     <emsp>INTAKE_REVERSE_BUTTON_STATE sees if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: <BR>
 *     <emsp>runIntakeMotor checks if the intake is not running and runs the intake </emsp> <BR>
 *     <emsp>stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED </emsp> <BR>
 *     <emsp>reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class HzSubsystem1 {

    public DcMotor subsystem1Motor = null;
    public Servo subsystem1Servo = null;

    public static final double SUBSYSTEM1_SERVO_LEVEL1 = 0.6;
    public static final double SUBSYSTEM1_SERVO_LEVEL2 = 0.35;

    public enum SUBSYSTEM1_MOTOR_STATE {
        STATE1,
        STATE2,
        STATE3
    }

    public SUBSYSTEM1_MOTOR_STATE subsystem1MotorState = SUBSYSTEM1_MOTOR_STATE.STATE1;

    public double subsystem1MotorPower1 = 0.95;//0.9;
    public double subsystem1MotorPower2 = 0.8;

    public enum SUBSYSTEM1_BUTTON_STATE {
        ON,
        OFF
    }
    public SUBSYSTEM1_BUTTON_STATE subsystem1ButtonState;

    public HzSubsystem1(HardwareMap hardwareMap) {
        subsystem1Motor = hardwareMap.dcMotor.get("frmotor");
        subsystem1Servo = hardwareMap.servo.get("servotest");
    }

    public void initSubsystem1(){

    }

    /**
     * runIntakeMotor checks if the intake is not running and runs the intake
     */
    public void startForwardSubsystem1Motor() {
        if(subsystem1MotorState != SUBSYSTEM1_MOTOR_STATE.STATE1) {
            runSubsystem1Motor(DcMotor.Direction.REVERSE, subsystem1MotorPower1);
            subsystem1MotorState = SUBSYSTEM1_MOTOR_STATE.STATE1;
        }
    }

    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void stopSubsystem1Motor() {
        if(subsystem1MotorState != SUBSYSTEM1_MOTOR_STATE.STATE2) {
            runSubsystem1Motor(DcMotor.Direction.FORWARD, 0.0);
            subsystem1MotorState = SUBSYSTEM1_MOTOR_STATE.STATE2;
       }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * ets intake motor state to REVERSING
     */
    public void startReverseSubsystem1Motor() {
        if(subsystem1MotorState != SUBSYSTEM1_MOTOR_STATE.STATE3) {
            runSubsystem1Motor(DcMotor.Direction.FORWARD, subsystem1MotorPower2);
            subsystem1MotorState = SUBSYSTEM1_MOTOR_STATE.STATE3;
        }
    }

    private void runSubsystem1Motor(DcMotor.Direction direction, double power){
        subsystem1Motor.setDirection(direction);
        subsystem1Motor.setPower(power);
    }

    /**
     * set Intake gripper position to hold.. to ensure intake is within robot dimensions at start
     */
    public void setIntakeReleaseHold(){
        subsystem1Servo.setPosition(SUBSYSTEM1_SERVO_LEVEL1);
    }

    /**
     * set Intake gripper position to release
     */
    public void setIntakeReleaseOpen(){
        subsystem1Servo.setPosition(SUBSYSTEM1_SERVO_LEVEL2);
    }

    /**
     * Returns Intake motor state
     */
    public SUBSYSTEM1_MOTOR_STATE getSubsystemMotorState() {
        return subsystem1MotorState;
    }
}