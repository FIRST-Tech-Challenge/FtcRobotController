package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public double intakePower = 0.9;
    public double intakeReversePower = 0.9;

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

    public void runIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.RUNNING) {
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(intakePower);
            intakeMotorState = INTAKE_MOTOR_STATE.RUNNING;
        }
    }

    public void stopIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.STOPPED) {
            intakeMotor.setPower(0.0);
            intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
       }
    }

    public void reverseIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.REVERSING) {
            intakeMotor.setDirection((DcMotor.Direction.FORWARD));
            intakeMotor.setPower(intakeReversePower);
            intakeMotorState = INTAKE_MOTOR_STATE.REVERSING;
        }
    }

    public void setIntakeReleaseHold(){
        intakeRelease.setPosition(INTAKE_RELEASE_HOLD);
    }

    public void setIntakeReleaseOpen(){
        intakeRelease.setPosition(INTAKE_RELEASE_OPEN);
    }

    public INTAKE_MOTOR_STATE getIntakeState() {
        return intakeMotorState;
    }
}