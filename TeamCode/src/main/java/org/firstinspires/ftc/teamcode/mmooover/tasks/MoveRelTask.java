package org.firstinspires.ftc.teamcode.mmooover.tasks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;
import org.jetbrains.annotations.NotNull;

import dev.aether.collaborative_multitasking.Scheduler;

public class MoveRelTask extends MoveToTask {

    @NotNull
    private final Pose offset;

    public MoveRelTask(
            @NotNull Scheduler scheduler,
            @NotNull Hardware hardware,
            @NotNull Pose offset,
            @NotNull EncoderTracking tracker,
            @NotNull LoopStopwatch loopTimer,
            @NotNull Speed2Power speed2Power,
            @NotNull Ramps ramps,
            @NotNull Telemetry telemetry
    ) {
        super(scheduler, hardware, new Pose(0, 0, 0), tracker, loopTimer, speed2Power, ramps, telemetry);
        this.offset = offset;
    }

    @Override
    public void invokeOnStart() {
        Pose current = tracker.getPose();
        target = current.add(offset);
        super.invokeOnStart();
    }
}
