package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CandyCane {
    private final DcMotor motor;

    private static final String CANDY_CANE_MOTOR_NAME = "CandyCaneMotor";

    public CandyCane(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, CANDY_CANE_MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0);
    }
}
