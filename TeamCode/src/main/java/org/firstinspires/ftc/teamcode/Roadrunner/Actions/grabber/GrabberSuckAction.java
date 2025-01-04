package org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber;


import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.Intake;

public class GrabberSuckAction implements Action {
    private Intake intake;
    private int time;

    public GrabberSuckAction(Intake intake, int time) {
        this.intake = intake;
        this.time = time;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        intake.grabberSuck();
        sleep(time);
        intake.grabberOff();
        return false;
    }
}
