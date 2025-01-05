package org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class ViperToPositionAction implements Action {
    private ViperSlide viperSlide;
    private int position;
    private double power;

    public ViperToPositionAction(ViperSlide viperSlide, int position, double power) {
        this.viperSlide = viperSlide;
        this.position = position;
        this.power = power;
    }

    public ViperToPositionAction(ViperSlide viperSlide, int position) {
        this(viperSlide, position, 1);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//        if (viperSlide.getPos() < position) {
//            viperSlide.setPower(power);
//        }
//        else if(viperSlide.getPos() > position) {
//            viperSlide.setPower(-power);
//        }
//        else {
//            viperSlide.stop();
//        }
//
//        return viperSlide.getPos() < (position - 25) || viperSlide.getPos() > (position + 25);

        boolean inPositionRange = viperSlide.getPos() >= (position - 25) && viperSlide.getPos() <= (position + 25);

        if (!inPositionRange) {
            if (viperSlide.getPos() < position) {
                viperSlide.setPower(power);
            }
            else if (viperSlide.getPos() > position) {
                viperSlide.setPower(-power);
            }
        }
        else {
            viperSlide.stop();
        }

        return !inPositionRange;
    }
}
