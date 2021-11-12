package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ContinuousIntake {
    final static double forwardPower = 1;
    DcMotor intakeMotor;

    public ContinuousIntake(HardwareMap hardwareMap, String motorName) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeOn() {
        intakeMotor.setPower(forwardPower);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void intakeReverse() {
        intakeMotor.setPower(-forwardPower);
    }

    public void setMotorPower(double power) {
        intakeMotor.setPower(power);
    }
}
