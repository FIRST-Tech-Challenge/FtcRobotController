package org.firstinspires.ftc.teamcode.Subsystems.Control;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class Control extends Subsystem {
    DcMotorEx intake;

    public Control(DcMotorEx intake) {
        this.intake = intake;
        telemetry.telemetry(2, "Control initialized", "Control initialized");

    }
    public void setIntake(boolean status) {
        if(status) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }
    }

    public void setIntakeReverse(boolean status) {
        if(status) {
            intake.setPower(-0.8);
        } else {
            intake.setPower(0.0);
        }
    }

}
