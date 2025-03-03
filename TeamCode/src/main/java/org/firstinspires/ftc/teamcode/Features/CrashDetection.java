package org.firstinspires.ftc.teamcode.Features;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CrashDetection {

    public static volatile boolean hasCrashed = false;

    //TODO: tune crash velocity threshold and time of collision
    private final double velocity_threshold = 0.0;
    private final double time_threshold = 0.0;

    private ElapsedTime time;
    
    private double previous_time = 0.0;
    private boolean under_monitor = false;

    public CrashDetection() {
        time = new ElapsedTime();
    }

    private boolean threshold_check(double value, double target, double threshold) {
        return value <= target + threshold && value >= target - threshold;
    }

    public void crashCheck(double current_vel, double target_vel) {
        if (!hasCrashed && target_vel > velocity_threshold &&
            threshold_check(current_vel, target_vel, velocity_threshold)) {
            if (!under_monitor) {
                under_monitor = true;
                previous_time = time.milliseconds();
            } else {
                if (time.seconds() - previous_time >= time_threshold) {
                    hasCrashed = true;
                }
            }
        }
    }

    public void update() {
        hasCrashed = false;
    }
}