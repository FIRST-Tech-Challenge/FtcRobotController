package org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber;


import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Intake;

public class GrabberSuckAction implements Action {
    private final Intake intake;
    private final ElapsedTime timer;
    private final long time;
    private boolean hasStarted;

    public GrabberSuckAction(Intake intake, long time) {
        this.intake = intake;
        this.time = time;
        this.timer = new ElapsedTime();
        this.timer.reset(); // Initialize the timer when the action is created
        this.hasStarted = false;
    }

    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!hasStarted) {
            // Reset the timer only when the action is actually triggered
            timer.reset();
            hasStarted = true; // Mark that the action has started
        }

        if (timer.milliseconds() < time) {
            // Keep the grabber running while the time is less than the target time
            intake.grabberSuck();
            return true;
        } else {
            // Ensure the grabber is turned off after the specified time
            intake.grabberOff();
            hasStarted = false; // Reset the flag when the action is completed
            return false;
        }
    }
}
