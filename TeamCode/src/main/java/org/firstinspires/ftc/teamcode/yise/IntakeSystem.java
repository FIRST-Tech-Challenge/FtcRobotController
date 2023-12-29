package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSystem {
    public DcMotor intake;

    public IntakeSystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void runIntakeSystem(double motorPower) {
        intake.setPower(motorPower);
    }
}
