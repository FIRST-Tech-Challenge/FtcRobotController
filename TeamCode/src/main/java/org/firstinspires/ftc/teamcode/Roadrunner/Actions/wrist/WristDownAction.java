package org.firstinspires.ftc.teamcode.Roadrunner.Actions.wrist;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.Intake;

public class WristDownAction implements Action {
    private Intake intake;

    public WristDownAction(Intake intake) {
        this.intake = intake;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        intake.wristDown();
        return false;
    }
}
