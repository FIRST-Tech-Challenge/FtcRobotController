package org.firstinspires.ftc.teamcode.Roadrunner.Actions.wrist;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.Intake;

public class WristUpAction implements Action {
    private Intake intake;

    public WristUpAction(Intake intake) {
        this.intake = intake;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        intake.wristUp();
        return false;
    }
}
