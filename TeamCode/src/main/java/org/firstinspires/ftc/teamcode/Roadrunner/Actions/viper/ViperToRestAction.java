package org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class ViperToRestAction implements Action {
    private ViperSlide viperSlide;
    private int position;

    public ViperToRestAction(ViperSlide viperSlide) {
        this.viperSlide = viperSlide;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(viperSlide.getCurrentPower() <= 5) {
            viperSlide.setPower(-1);
        }
        else {
            viperSlide.stop();
        }


        return viperSlide.getCurrentPower() <= 5;

    }
}
