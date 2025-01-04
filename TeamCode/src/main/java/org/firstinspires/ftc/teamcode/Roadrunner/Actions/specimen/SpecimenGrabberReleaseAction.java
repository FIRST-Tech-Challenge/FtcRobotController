package org.firstinspires.ftc.teamcode.Roadrunner.Actions.specimen;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class SpecimenGrabberReleaseAction implements Action {
    private ViperSlide viperSlide;

    public SpecimenGrabberReleaseAction(ViperSlide viperSlide) {
        this.viperSlide = viperSlide;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        viperSlide.releaseSpecimen();

        return false;
    }
}
