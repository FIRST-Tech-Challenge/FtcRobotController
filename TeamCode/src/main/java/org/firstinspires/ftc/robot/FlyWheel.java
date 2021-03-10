package org.firstinspires.ftc.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FlyWheel {

    public Motor flywheel;

    private double flywheelSpeed = 0;
    private double flywheelDirection = Vals.flywheel_direction;

    private double lastTimeStamp = 0;
    private double lastVelocity = 0;
    private final double TIME_CONSTANT = 0.5;


    private int ticks = 0;

    public FlyWheel(Motor flywheel) {
        this.flywheel = flywheel;


        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(Vals.flywheel_kp, Vals.flywheel_ki, Vals.flywheel_kd);
        this.flywheel.setFeedforwardCoefficients(Vals.flywheel_ks, Vals.flywheel_kv);
    }

    private void set() {

        this.flywheel.set(this.flywheelDirection * this.flywheelSpeed);
    }

    public void on() {
        if(!this.isOn()) {
            this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
            this.flywheelSpeed = Vals.flywheel_speed;
        }

        this.set();
    }

    public void off() {
        if(this.isOn()) {
            this.flywheel.setRunMode(Motor.RunMode.RawPower);
            this.flywheelSpeed = 0;
        }

        this.set();
    }

    public boolean isOn() {
        return this.flywheelSpeed != 0;
    }

    public void flipDirection() {
        this.flywheelDirection *= -1;

        this.set();
    }

    public boolean isReady() {
        double velocity = Math.abs(flywheel.getCorrectedVelocity());

        if(lastTimeStamp == 0) {
            lastVelocity = velocity;
            lastTimeStamp = System.nanoTime() / 1e9;
        } else {
            double currentTime = (double)System.nanoTime() / 1e9;
            double dt = lastTimeStamp - currentTime;
            lastTimeStamp = currentTime;

            double k = Math.exp(dt / TIME_CONSTANT);

            double newVelocity = k * lastVelocity + (1 - k) * velocity;
            lastVelocity = newVelocity;
            Vals.flywheel_filtered_speed = lastVelocity;
        }

        if(lastVelocity >= Vals.flywheel_ready_min_speed && lastVelocity <= Vals.flywheel_ready_max_speed) ticks++;
        else ticks = 0;

        if(ticks >= Vals.flywheel_ready_ticks) {
            lastVelocity = 0;
            lastTimeStamp = 0;
            ticks = 0;
            return true;
        }
        return false;


    }
}
