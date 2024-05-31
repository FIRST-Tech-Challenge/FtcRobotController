package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Subsystem;

public class Intake extends Subsystem {
    public final DcMotor motor;

    public Intake(DcMotor motor) {
        this.motor = motor;
    }

    public void run(double speed) {
        motor.setPower(Range.clip(speed, -1.0, 1.0));
    }
}
