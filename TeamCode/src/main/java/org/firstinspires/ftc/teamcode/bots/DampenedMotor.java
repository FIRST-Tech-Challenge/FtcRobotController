package org.firstinspires.ftc.teamcode.bots;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class DampenedMotor {

    private final String TAG = "DampenedMotor";
    protected DcMotorEx theMotor = null;
    protected double dFactor = 1.0;

    protected double currentPower = 0;
    protected double targetPower = 0;

    protected void update() {
        double delta = this.targetPower - this.currentPower;
        double newPower = currentPower + (delta * dFactor);
        newPower = Range.clip(newPower, -1.0, +1.0);
        if (this.theMotor != null) {
            theMotor.setPower(newPower);
        }
        this.currentPower = newPower;
    }

    public DampenedMotor (DcMotorEx motor, DcMotor.Direction direction, double dampeningFactor) {
        if (motor != null) {
            this.theMotor = motor;
            this.dFactor = Range.clip(dampeningFactor, -1.0, 1.0);
            this.theMotor.setDirection(direction);
            this.theMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.theMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            Log.d(TAG, "DampenedMotor initialized with a null motor object. No movement will occur");
        }
    }

    public void stop() {
        if (this.theMotor != null) {
            this.theMotor.setPower(0);
        }
    }
    public void setTargetPower(double power) {
        this.targetPower = Range.clip(power, -1.0, 1.0);
        update();
    }

    public double getTargetPower() {
        return this.targetPower;
    }

    public double getCurrentPower() {
        return this.currentPower;
    }

    public double getCurrentPosition() {
        if (this.theMotor != null) {
            return this.theMotor.getCurrentPosition();
        }
        return 0;
    }
}
