package org.firstinspires.ftc.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FlyWheel {

    public Motor flywheel;

    private double flywheelSpeed = 0;
    private double flywheelDirection = Vals.flywheel_direction;

    private static final int MIN_SPEED = 960;
    private static final int MAX_SPEED = 990;

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
        if(velocity >= MIN_SPEED && velocity <= MAX_SPEED) ticks++;
        else ticks = 0;
        
        if(ticks >= 5) {
            ticks = 0;
            return true;
        }
        return false;


    }
}
