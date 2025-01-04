package org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class ViperToPositionAction implements Action {
    private ViperSlide viperSlide;
    private int position;

    public ViperToPositionAction(ViperSlide viperSlide, int position) {
        this.viperSlide = viperSlide;
        this.position = position;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (viperSlide.getPos() < position) {
            viperSlide.setPower(1);
        }
        else if(viperSlide.getPos() > position) {
            viperSlide.setPower(-1);
        }
        else {
            viperSlide.stop();
        }

        return viperSlide.getPos() < (position - 25) || viperSlide.getPos() > (position + 25);

    }
}
