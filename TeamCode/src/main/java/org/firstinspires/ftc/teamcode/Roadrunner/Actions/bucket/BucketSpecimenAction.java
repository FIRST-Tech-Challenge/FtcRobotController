package org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class BucketSpecimenAction implements Action {
    private ViperSlide viperSlide;

    public BucketSpecimenAction(ViperSlide viperSlide) {
        this.viperSlide = viperSlide;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        viperSlide.bucketSpecimen();

        return false;
    }
}

