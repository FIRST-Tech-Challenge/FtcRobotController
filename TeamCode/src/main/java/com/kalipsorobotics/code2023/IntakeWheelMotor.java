package com.kalipsorobotics.code2023;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.kalipsorobotics.utilities.OpModeUtilities;

class IntakeWheelMotor {
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
