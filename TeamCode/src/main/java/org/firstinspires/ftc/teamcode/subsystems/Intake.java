package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    DcMotorEx intakeMotor, transferMotor;
    Servo polliRollerServo;

    //constants
    public double POLLI_UP_POS = 0.73, POLLI_MID_POS = 0.505, POLLI_DOWN_POS = 0.32;
    private double INTAKE_POWER = 0.8, IDLE_INTAKE_POWER = 0.3;
    private double TRANSFER_POWER = 0.6, IDLE_TRANSFER_POWER = 0.17625;

    public Intake(HardwareMap h) {
        intakeMotor = h.get(DcMotorEx.class, "intake");
        transferMotor = h.get(DcMotorEx.class, "transfer");

        polliRollerServo = h.get(Servo.class,"polliRoller");

        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    //dynamic polli setters
    public void setPolliUp() { polliRollerServo.setPosition(POLLI_UP_POS); }
    public void setPolliMid() { polliRollerServo.setPosition(POLLI_MID_POS); }
    public void setPolliDown() { polliRollerServo.setPosition(POLLI_DOWN_POS); }

    //dynamic intake setters
    public void setIntakeSpeed(double speed) { intakeMotor.setPower(speed); }
    public void setTransferSpeed(double speed) { transferMotor.setPower(speed); }

    //static setters for abstraction
    public void startIntake() { intakeMotor.setPower(INTAKE_POWER); }
    public void idleIntake() { intakeMotor.setPower(IDLE_INTAKE_POWER); }
    public void stopIntake() { intakeMotor.setPower(0); }
    public void startTransfer() { transferMotor.setPower(TRANSFER_POWER); }
    public void idleTransfer() { transferMotor.setPower(IDLE_TRANSFER_POWER); }
    public void stopTransfer() { transferMotor.setPower(0); }
}
