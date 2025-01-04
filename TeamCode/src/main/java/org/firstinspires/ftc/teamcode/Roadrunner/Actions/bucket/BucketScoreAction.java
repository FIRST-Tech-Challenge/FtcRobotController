package org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class BucketScoreAction implements Action {
    private ViperSlide viperSlide;

    public BucketScoreAction(ViperSlide viperSlide) {
        this.viperSlide = viperSlide;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        viperSlide.bucketScore();

        return false;
    }
}

