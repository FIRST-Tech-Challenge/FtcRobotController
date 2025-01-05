package org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class BucketSpecimenAction implements Action {
    private ViperSlide viperSlide;
    private final ElapsedTime timer;
    private final long delay;
    private boolean hasStarted;

    public BucketSpecimenAction(ViperSlide viperSlide, long delay) {
        this.viperSlide = viperSlide;
        this.delay = delay;
        this.timer = new ElapsedTime();
        this.timer.reset(); // Initialize the timer when the action is created
        this.hasStarted = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!hasStarted) {
            // Reset the timer only when the action is actually triggered
            timer.reset();
            hasStarted = true; // Mark that the action has started
        }

        if (timer.milliseconds() > delay) {
            viperSlide.bucketSpecimen();
            hasStarted = false;
            return false;
        }
        return true;
    }
}

