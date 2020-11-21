package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotor intakeMotor = null;

    public enum INTAKE_MOTOR_STATE {
        INTAKE_MOTOR_RUNNING,
        INTAKE_MOTOR_STOPPED,
        INTAKE_MOTOR_REVERSING
    }
    public INTAKE_MOTOR_STATE intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");
    }

    public void initIntake(){

    }

    public void runIntakeMotor(double intakePower) {
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_RUNNING) {
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setPower(intakePower);
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_RUNNING;
        }
    }

    public void stopIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED) {
            intakeMotor.setPower(0);
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED;
       }
    }

    public void reverseIntakeMotor(double power) {
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING) {
            intakeMotor.setDirection((DcMotor.Direction.REVERSE));
            intakeMotor.setPower(power);
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING;
        }
    }

    public void reverseIntake(boolean reverseIntakeFlag) {
        if(reverseIntakeFlag) {
            reverseIntakeMotor(0.5);
        }
    }

    public INTAKE_MOTOR_STATE getIntakeState() {
        return intakeMotorState;
    }
}