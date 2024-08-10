package com.kalipsorobotics.fresh.mechanisms;


import com.kalipsorobotics.fresh.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeWheelMotor {
    private final DcMotor intake;
    private IntakeWheelMotor(DcMotor intake) {
        this.intake = intake;
    }
    public IntakeWheelMotor(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().get(DcMotorEx.class,"intake"));
    }
    public void on() {
        intake.setPower(1);
    }
    public void off() {
        intake.setPower(0);
    }
    public void reverse() {
        intake.setPower(-1);
    }
}
