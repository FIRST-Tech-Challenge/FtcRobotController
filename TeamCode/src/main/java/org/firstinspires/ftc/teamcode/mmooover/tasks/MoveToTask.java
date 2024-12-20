package org.firstinspires.ftc.teamcode.mmooover.tasks;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.RightAuto;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Motion;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;
import org.jetbrains.annotations.NotNull;

import java.util.Set;

import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskTemplate;

@SuppressLint("DefaultLocale")
public class MoveToTask extends TaskTemplate {
    protected Pose target;
    protected final EncoderTracking tracker;
    protected final Speed2Power speed2Power;
    protected final Ramps ramps;
    protected final Telemetry telemetry;
    protected final LoopStopwatch loopTimer;
    protected final Hardware hardware;
    protected ElapsedTime targetTime = new ElapsedTime();
    protected ElapsedTime runTime = new ElapsedTime();
    protected boolean finished = false;

    public MoveToTask(
            @NotNull Scheduler scheduler,
            @NotNull Hardware hardware,
            @NotNull Pose target,
            @NotNull EncoderTracking tracker,
            @NotNull LoopStopwatch loopTimer,
            @NotNull Speed2Power speed2Power,
            @NotNull Ramps ramps,
            @NotNull Telemetry telemetry
    ) {
        super(scheduler);
        this.target = target;
        this.tracker = tracker;
        this.speed2Power = speed2Power;
        this.ramps = ramps;
        this.telemetry = telemetry;
        this.loopTimer = loopTimer;
        this.hardware = hardware;
    }

    @Override
    public void invokeOnStart() {
        targetTime.reset();
        runTime.reset();
    }

    @Override
    public void invokeOnTick() {
        Pose current = tracker.getPose();

        double linear = current.linearDistanceTo(target);
        double angular = current.subtractAngle(target);
        if (linear > RightAuto.ACCEPT_DIST || abs(angular) > RightAuto.ACCEPT_TURN) {
            targetTime.reset();
        }
        // Waits at the target for one second
        if (targetTime.time() > .5) {
            finished = true;
            return;
        }
        // figure out how to get to the target position
        Motion action = tracker.getMotionToTarget(target, hardware);
        double dToTarget = sqrt(
                action.forward() * action.forward()
                        + action.right() * action.right()
                        + action.turn() * action.turn());
        double now = runTime.time();
        double speed = ramps.ease(
                now,
                dToTarget,
                0.75
        );
        action.apply(hardware.driveMotors, RightAuto.CALIBRATION, speed, speed2Power);
        telemetry.addData("forward", action.forward());
        telemetry.addData("right", action.right());
        telemetry.addData("turn (deg)", Math.toDegrees(action.turn()));
        String message = String.format(
                "##%.3f##{\"pose\":[%.6f,%.6f,%.6f],\"motion\":[%.6f,%.6f,%.6f],\"speed\":%.6f," +
                        "\"frontLeft\":%.6f,\"frontRight\":%.6f,\"backLeft\":%.6f,\"backRight\":%.6f," +
                        "\"dToTarget\":%.6f,\"timer\":%.4f,\"avgTickTime\":%.6f}##",
                System.currentTimeMillis() / 1000.0,
                current.x(), current.y(), current.heading(),
                action.forward(), action.right(), action.turn(),
                speed,
                action.getLastFL(), action.getLastFR(), action.getLastBL(), action.getLastBR(),
                dToTarget, now,
                loopTimer.getAvg() * 1000
        );
        Log.d("DataDump", message);
    }

    @Override
    public boolean invokeIsCompleted() {
        return finished;
    }

    @Override
    public void invokeOnFinish() {
        hardware.driveMotors.setAll(0.0);
    }

    private static final Set<SharedResource> REQUIREMENTS = Set.of(
            Hardware.Locks.DriveMotors
    );

    @Override
    public @NotNull Set<SharedResource> requirements() {
        return REQUIREMENTS;
    }
}
