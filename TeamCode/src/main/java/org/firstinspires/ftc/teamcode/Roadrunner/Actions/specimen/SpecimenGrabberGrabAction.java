package org.firstinspires.ftc.teamcode.Roadrunner.Actions.specimen;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class SpecimenGrabberGrabAction implements Action {
    private ViperSlide viperSlide;

    public SpecimenGrabberGrabAction(ViperSlide viperSlide) {
        this.viperSlide = viperSlide;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        viperSlide.grabSpecimen();

        return false;
    }
}
