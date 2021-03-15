package org.firstinspires.ftc.robot;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FlyWheel {

    public Motor flywheel;
    public PIDController pidFlywheel;
    private Telemetry telemtry;

    private double flywheelSpeed = 0;
    private double flywheelDirection = Vals.flywheel_direction;

    private double lastTimeStamp = 0;
    private double lastVelocity = 0;
    private final double TIME_CONSTANT = 0.5;
    private int ticks = 0;

    public FlyWheel(Motor flywheel, Telemetry telemetry) {
        this.telemtry = telemetry;
        pidFlywheel = new PIDController(Vals.flywheel_kp, Vals.flywheel_ki, Vals.flywheel_kd);
        pidFlywheel.setTolerance(Vals.flywheel_tolerance);

        this.flywheel = flywheel;

        this.flywheel.ACHIEVABLE_MAX_TICKS_PER_SECOND = Vals.flywheel_max_achievable_ticks;
//        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
//        this.flywheel.setVeloCoefficients(Vals.flywheel_kp, Vals.flywheel_ki, Vals.flywheel_kd);
//        this.flywheel.setFeedforwardCoefficients(Vals.flywheel_ks, Vals.flywheel_kv);
        this.flywheel.setRunMode(Motor.RunMode.RawPower);
        lastTimeStamp = 0;
        lastVelocity = 0;
    }

    private void set() {
        updateVelocity();
        double power = 0;
        double pidOutput = pidFlywheel.calculate(lastVelocity, this.flywheelSpeed);
        if(this.flywheelSpeed > 0) {
            power = Math.min(pidOutput + Vals.flywheel_ff, 1);
        }
        telemtry.addData("Flywheel Set Power: ", power);
        telemtry.addData("Flywheel PID Output: ", pidOutput);
        telemtry.addData("Flywheel Set Speed", this.flywheelSpeed);
        telemtry.addData("Flywheel Last Velocity", lastVelocity);
        this.flywheel.set(this.flywheelDirection * power);
    }

    public void on() {
        if(!this.isOn()) {
//            this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
            pidFlywheel.setPID(Vals.flywheel_kp, Vals.flywheel_ki, Vals.flywheel_kd);
            pidFlywheel.setTolerance(Vals.flywheel_tolerance);
            this.flywheelSpeed = Vals.flywheel_speed;
        }

        this.set();
    }

    public void off() {
        if(this.isOn()) {
            this.flywheel.setRunMode(Motor.RunMode.RawPower);
            this.flywheelSpeed = 0;
            this.lastTimeStamp = 0;
            this.lastVelocity = 0;
            pidFlywheel.reset();
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
//        updateVelocity();

        if(this.lastVelocity >= Vals.flywheel_ready_min_speed && lastVelocity <= Vals.flywheel_ready_max_speed) ticks++;
        else ticks = 0;

        if(ticks >= Vals.flywheel_ready_ticks) {
//            lastVelocity = 0;
//            lastTimeStamp = 0;
            ticks = 0;
            return true;
        }
        return false;
    }

    private void updateVelocity() {
        double velocity = Math.abs(flywheel.getCorrectedVelocity());
        if(lastTimeStamp == 0) {
            lastVelocity = velocity;
            lastTimeStamp = System.nanoTime() / 1e9;
        } else {
            double currentTime = (double)System.nanoTime() / 1e9;
            double dt = lastTimeStamp - currentTime;
            telemtry.addData("Delta T", dt);
            lastTimeStamp = currentTime;

            double k = Math.exp(dt / TIME_CONSTANT);

            double newVelocity = k * lastVelocity + (1 - k) * velocity;
            lastVelocity = newVelocity;
            Vals.flywheel_filtered_speed = lastVelocity;
        }
    }
}
