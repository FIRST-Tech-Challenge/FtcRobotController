package org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class ViperStopAction implements Action {
    private ViperSlide viperSlide;

    public ViperStopAction(ViperSlide viperSlide) {
        this.viperSlide = viperSlide;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        viperSlide.stop();


        return false;

    }
}
