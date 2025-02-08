package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.jetbrains.annotations.NotNull;

import java.util.Set;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.ITaskWithResult;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskTemplate;
import dev.aether.collaborative_multitasking.TaskWithResultTemplate;

public class VLiftProxy extends TaskTemplate {
    // Speed and limits are in Lift.kt file now!

    private static final Set<SharedResource> requires = Set.of(Hardware.Locks.VerticalSlide);
    private static int INSTANCE_COUNT = 0;
    /// If you hold this lock, you have exclusive control over the Lift (by proxy of this task.)
    public final SharedResource CONTROL = new SharedResource("LiftBackgroundTask" + (++INSTANCE_COUNT));
    private final Lift lift;
    private final Set<SharedResource> provides = Set.of(CONTROL);
    private int targetPosition = 0;

    public VLiftProxy(@NotNull Scheduler scheduler, Lift lift) {
        super(scheduler);
        this.lift = lift;
    }

    @Override
    @NotNull
    public Set<SharedResource> requirements() {
        return requires;
    }

    @Override
    public boolean getDaemon() {
        return true;
    }

    @Override
    public void invokeOnTick() {
        lift.setTargetPosition(targetPosition);
        lift.update();
    }

    public void commitCurrent() {
        targetPosition = lift.getCurrentPosition();
    }

    @Override
    public void invokeOnStart() {
        commitCurrent();
        lift.setTargetPosition(targetPosition);
    }

    public ITask target(int target) {
        return new TaskTemplate(getScheduler()) {
            @Override
            public void invokeOnStart() {
                targetPosition = target;
            }

            @Override
            public boolean invokeIsCompleted() {
                return true;
            }
        };
    }

    public ITaskWithResult<Boolean> moveTo(int target, int range, double maxDuration) {
        ITaskWithResult<Boolean> result;
        // This version has a timer
        if (maxDuration >= 0) result = new TaskWithResultTemplate<Boolean>(getScheduler()) {
            private final ElapsedTime t = new ElapsedTime();

            @Override
            @NotNull
            public Set<SharedResource> requirements() {
                return provides;
            }

            @Override
            public void invokeOnStart() {
                targetPosition = target;
                t.reset();
            }

            @Override
            public boolean invokeIsCompleted() {
                int pos = lift.getCurrentPosition();
                if (t.time() >= maxDuration) {
                    Log.w("VLiftProxy", String.format("giving up, at %d, target %d +- %d", pos, target, range));
                    setResult(false);
                    return true;
                }
                if (Math.abs(pos - target) < range) {
                    setResult(true);
                    return true;
                }
                return false;
            }

            @Override
            public void invokeOnFinish() {
                setResultMaybe(false);
            }
        };
            // This version doesn't
        else result = new TaskWithResultTemplate<Boolean>(getScheduler()) {
            @Override
            @NotNull
            public Set<SharedResource> requirements() {
                return provides;
            }

            @Override
            public void invokeOnStart() {
                targetPosition = target;
            }

            @Override
            public boolean invokeIsCompleted() {
                if (Math.abs(lift.getCurrentPosition() - target) < range) {
                    setResult(true);
                    return true;
                }
                return false;
            }
        };
        return result;
    }
}
