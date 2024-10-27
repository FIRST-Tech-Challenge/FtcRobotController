package org.firstinspires.ftc.teamcode.NewStuff.actions.code2023;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;

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
