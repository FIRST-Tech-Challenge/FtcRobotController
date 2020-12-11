package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HzIntake {

    public DcMotor intakeMotor = null;

    public enum INTAKE_MOTOR_STATE {
        RUNNING,
        STOPPED,
        REVERSING
    }

    public INTAKE_MOTOR_STATE intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;

    public double intakePower = 0.8;
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

    public INTAKE_MOTOR_STATE getIntakeState() {
        return intakeMotorState;
    }
}