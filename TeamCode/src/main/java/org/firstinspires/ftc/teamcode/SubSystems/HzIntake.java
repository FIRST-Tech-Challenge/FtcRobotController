package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
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
public class HzIntake {

    public DcMotor intakeMotor = null;
    public Servo intakeRelease = null;

    public static final double INTAKE_RELEASE_HOLD = 0.6;
    public static final double INTAKE_RELEASE_OPEN = 0.35;

    public enum INTAKE_MOTOR_STATE {
        RUNNING,
        STOPPED,
        REVERSING
    }

    public INTAKE_MOTOR_STATE intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;

    public double intakePower = 0.95;//0.9;
    public double intakeReversePower = 0.8;

    public enum INTAKE_BUTTON_STATE {
        ON,
        OFF
    }
    public INTAKE_BUTTON_STATE intakeButtonState;


    public enum INTAKE_REVERSE_BUTTON_STATE {
        ON,
        OFF
    }
    public INTAKE_REVERSE_BUTTON_STATE intakeReverseButtonState;


    public HzIntake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake_rightenc");
        intakeRelease = hardwareMap.servo.get("intake_release_servo");
    }

    public void initIntake(){

    }

    /**
     * runIntakeMotor checks if the intake is not running and runs the intake
     */
    public void runIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.RUNNING) {
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(intakePower);
            intakeMotorState = INTAKE_MOTOR_STATE.RUNNING;
        }
    }

    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void stopIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.STOPPED) {
            intakeMotor.setPower(0.0);
            intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
       }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * ets intake motor state to REVERSING
     */
    public void reverseIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.REVERSING) {
            intakeMotor.setDirection((DcMotor.Direction.FORWARD));
            intakeMotor.setPower(intakeReversePower);
            intakeMotorState = INTAKE_MOTOR_STATE.REVERSING;
        }
    }

    /**
     * set Intake gripper position to hold.. to ensure intake is within robot dimensions at start
     */
    public void setIntakeReleaseHold(){
        intakeRelease.setPosition(INTAKE_RELEASE_HOLD);
    }

    /**
     * set Intake gripper position to release
     */
    public void setIntakeReleaseOpen(){
        intakeRelease.setPosition(INTAKE_RELEASE_OPEN);
    }

    /**
     * Returns Intake motor state
     */
    public INTAKE_MOTOR_STATE getIntakeState() {
        return intakeMotorState;
    }
}