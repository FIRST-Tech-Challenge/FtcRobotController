package org.firstinspires.ftc.teamcode.Roadrunner.Actions.hSlide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;

public class hSlideToPositionAction implements Action {
    private HorizontalSlide hSlide;
    private int position;

    public hSlideToPositionAction(HorizontalSlide hSlide, int position) {
        this.hSlide = hSlide;
        this.position = position;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        boolean inPositionRange = hSlide.getPos() >= (position - 25) && hSlide.getPos() <= (position + 25);

        if (!inPositionRange) {
            if (hSlide.getPos() < position) {
                hSlide.setPower(1);
            }
            else if (hSlide.getPos() > position) {
                hSlide.setPower(-1);
            }
        }
        else {
            hSlide.stopMotor();
        }

        return !inPositionRange;
    }
}
