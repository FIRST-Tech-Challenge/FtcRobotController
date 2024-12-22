package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private static final double SPEED = 1.0;
    private static final int MAX_VERTICAL_LIFT_TICKS = 2300;
    private static final int MIN_VERTICAL_LIFT_TICKS = 0;
    private static final Set<SharedResource> requires = Set.of(Hardware.Locks.VerticalSlide);
    private static int INSTANCE_COUNT = 0;
    /// If you hold this lock, you have exclusive control over the Lift (by proxy of this task.)
    public final SharedResource CONTROL = new SharedResource("LiftBackgroundTask" + (++INSTANCE_COUNT));
    private final DcMotor lift;
    private final Set<SharedResource> provides = Set.of(CONTROL);
    private final Scheduler scheduler;
    private boolean manualAdjustMode = false;
    private int targetPosition = 0;

    public VLiftProxy(@NotNull Scheduler scheduler, DcMotor lift) {
        super(scheduler);
        this.lift = lift;
        this.scheduler = scheduler;
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
    }

    public void commitCurrent() {
        targetPosition = lift.getCurrentPosition();
    }

    @Override
    public void invokeOnStart() {
        commitCurrent();
        lift.setTargetPosition(targetPosition);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(SPEED);
    }

    private void startManual() {
        // forcibly grab the lock from whatever has it at the moment
        scheduler.filteredStop(it -> it.requirements().contains(CONTROL));
        scheduler.manualAcquire(CONTROL);
        manualAdjustMode = true;
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopManual() {
        manualAdjustMode = false;
        commitCurrent();
        lift.setTargetPosition(targetPosition);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(SPEED);
        scheduler.manualRelease(CONTROL);
    }

    public void controlManual(boolean goUp, boolean goDown) {
        if (!manualAdjustMode) {
            if (goUp || goDown) startManual();
            return;
        }
        int currentPosition = lift.getCurrentPosition();
        if (goUp && goDown) {
            lift.setPower(0);
            return;
        }
        if (goUp) {
            if (currentPosition < MAX_VERTICAL_LIFT_TICKS) {
                lift.setPower(SPEED);
                return;
            }
        }
        if (goDown) {
            if (currentPosition > MIN_VERTICAL_LIFT_TICKS) {
                lift.setPower(-SPEED);
                return;
            }
        }
        stopManual();
    }

    public boolean isManualAdjustModeEnabled() {
        return manualAdjustMode;
    }

    public ITask target(int target) {
        return new TaskTemplate(scheduler) {
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
        if (maxDuration >= 0) result = new TaskWithResultTemplate<Boolean>(scheduler) {
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
                if (t.time() >= maxDuration) {
                    setResult(false);
                    return true;
                }
                if (Math.abs(lift.getCurrentPosition() - target) < range) {
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
        else result = new TaskWithResultTemplate<Boolean>(scheduler) {
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
