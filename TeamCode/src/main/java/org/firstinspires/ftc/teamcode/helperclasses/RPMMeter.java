package org.firstinspires.ftc.teamcode.helperclasses;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RPMMeter {

    private DcMotor motor;

    private int rpm;
    private double updateSpeed;

    public RPMMeter(DcMotor motor, double updateSpeed) {
        this.motor = motor;
        this.updateSpeed = updateSpeed;
    }

    /**
     * Start measuring the rotations per minute of a motor
     */
    public void start() {

    }

    /**
     *
     * @return rpm of the motor
     */
    public int getRPM() {
        return rpm;
    }
}
