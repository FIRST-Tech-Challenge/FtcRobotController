package org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber;


import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Intake;

public class GrabberSpitAction implements Action {
    private final Intake intake;
    private final ElapsedTime timer;
    private final long time;
    private final long delay;
    private boolean hasStarted;

    public GrabberSpitAction(Intake intake, long time, long delay) {
        this.intake = intake;
        this.time = time;
        this.delay = delay;
        this.timer = new ElapsedTime();
        this.timer.reset(); // Initialize the timer when the action is created
        this.hasStarted = false;
    }

    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//        if (!hasStarted) {
//            // Reset the timer only when the action is actually triggered
//            timer.reset();
//            hasStarted = true; // Mark that the action has started
//        }
//
//        if (timer.milliseconds() < time) {
//            if (timer.milliseconds() > delay) {
//                // Keep the grabber running
//                intake.grabberSpit();
//            }
//        } else {
//            // Stop the grabber after the elapsed time
//            intake.grabberOff();
//            hasStarted = false; // Reset the flag when the action is completed
//        }
//
//        // Return true if the action is still ongoing, false if completed
//        return timer.milliseconds() < time;

        if (!hasStarted) {
            // Reset the timer only when the action is actually triggered
            timer.reset();
            hasStarted = true; // Mark that the action has started
        }

        if (timer.milliseconds() < time) {
            if (timer.milliseconds() > delay) {
                // Keep the grabber running
                intake.grabberSpit();
                return true;
            }
            return true;
        }
        else {
            // Ensure the grabber is turned off after the specified time
            intake.grabberOff();
            hasStarted = false; // Reset the flag when the action is completed
            return false;
        }

    }
}
