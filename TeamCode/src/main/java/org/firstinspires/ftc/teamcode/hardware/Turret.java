package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Turret.MAX_TICKS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Turret.MIN_TICKS;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Turret.SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Turret {
    private final DcMotorEx motor;
    private final boolean installed;

    public Turret(DcMotorEx motor) {
        this.motor = motor;
        this.installed = motor != null;

        if (installed) {
            // Assumes the turret is centered when the robot starts
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public boolean isInstalled() {
        return installed;
    }

    /** power: -1 to 1. Stops automatically at the soft limits. */
    public void setPower(double power) {
        if (!installed) return;
        int pos = motor.getCurrentPosition();
        if (pos >= MAX_TICKS && power > 0) power = 0;
        if (pos <= MIN_TICKS && power < 0) power = 0;
        motor.setPower(power * SPEED);
    }

    public int getPosition() {
        return installed ? motor.getCurrentPosition() : 0;
    }
}
