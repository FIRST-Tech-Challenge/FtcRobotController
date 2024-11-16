package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Motion;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;
import org.jetbrains.annotations.NotNull;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskTemplate;
import dev.aether.collaborative_multitasking.ext.Pause;
import kotlin.Unit;

@Autonomous // Appear on the autonomous drop down
// STOP POSTING ABOUT DefaultLocale IM TIRED OF SEEING IT
@SuppressLint("DefaultLocale")
public class RightAuto extends LinearOpMode {
    final static class BackgroundTasks extends TaskTemplate {
        EncoderTracking tracker;
        LoopStopwatch timer;

        public BackgroundTasks(
                @NotNull Scheduler scheduler,
                @NotNull EncoderTracking tracker,
                @NotNull LoopStopwatch timer
        ) {
            super(scheduler);
            this.tracker = tracker;
            this.timer = timer;
        }

        @Override
        public boolean getDaemon() {
            return true;
        }

        @Override
        @NotNull
        public String getName() {
            return "Background Items";
        }

        @Override
        public void invokeOnStart() {
            timer.clear();
        }

        @Override
        public void invokeOnTick() {
            timer.click();
            tracker.step();
        }
    }

    final static class MoveToTask extends TaskTemplate {
        private final Pose target;
        private final EncoderTracking tracker;
        private final Speed2Power speed2Power;
        private final Ramps ramps;
        private final Telemetry telemetry;
        private final LoopStopwatch loopTimer;
        private final Hardware hardware;
        private ElapsedTime targetTime = new ElapsedTime();
        private ElapsedTime runTime = new ElapsedTime();
        private boolean finished = false;

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
            if (linear > ACCEPT_DIST || abs(angular) > ACCEPT_TURN) {
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
            action.apply(hardware.driveMotors, CALIBRATION, speed, speed2Power);
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

    private static final RuntimeException NOT_IMPLEMENTED = new RuntimeException("This operation is not implemented");

    // Constants //
    public static final double ACCEPT_DIST = 1; // inch. euclidean distance
    public static final double ACCEPT_TURN = Math.toRadians(5);

    // power biases
    public static final Motion.Calibrate CALIBRATION = new Motion.Calibrate(1.0, 1.0, 1.0); // Calibration factors for strafe, forward, and turn.

    Hardware hardware;
    EncoderTracking tracker;
    final Pose PARK = new Pose(4, -36, Math.toRadians(0));
    private Ramps ramps;
    private LoopStopwatch loopTimer;
    private Speed2Power speed2Power;

    private MoveToTask moveTo(Scheduler s, Pose target) {
        return new MoveToTask(
                s, hardware, target, tracker, loopTimer, speed2Power, ramps, telemetry
        );
    }

    @Override
    public void runOpMode() {
        MultitaskScheduler scheduler = new MultitaskScheduler();

        hardware = new Hardware(hardwareMap);
        tracker = new EncoderTracking(hardware);
        ElapsedTime scoreTimer = new ElapsedTime();
        ElapsedTime finalizeTimer = new ElapsedTime();
        AtomicReference<Double> scoredIn = new AtomicReference<>((double) 0);
        double doneIn = 0;
        loopTimer = new LoopStopwatch();
        speed2Power = new Speed2Power(0.20); // Set a speed2Power corresponding to a speed of 0.15 seconds
        ramps = new Ramps(
                Ramps.linear(2.0),
                Ramps.linear(1 / 12.0),
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );

        scheduler.task(new BackgroundTasks(
                scheduler, tracker, loopTimer
        ));
        scheduler.task(new OneShot(scheduler, () -> {
            hardware.claw.setPosition(0.55);
            hardware.wrist.setPosition(0.28);
            hardware.twist.setPosition(0.17);
            hardware.arm.setTargetPosition(0);
            hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }));
        scheduler
                .task(moveTo(scheduler, new Pose(6, -40, 0)))
                .then(new OneShot(scheduler, () -> hardware.wrist.setPosition(0.44)))
                .then(new Pause(scheduler, 0.500))
                .then(new OneShot(scheduler, () -> hardware.claw.setPosition(0.02)))
                .then(new Pause(scheduler, 0.500))
                .then(new OneShot(scheduler, () -> {
                    hardware.wrist.setPosition(0.28);
                    hardware.claw.setPosition(0.55);
                }))
                .then(moveTo(scheduler, new Pose(6, -48, 0)))
                .then(moveTo(scheduler, new Pose(4, -48, 0)));
//                .then(moveTo(scheduler,
//                        new Pose(65, -12, Math.toRadians(0))))
        // park

        telemetry.addLine("Initialized.");
        telemetry.addLine(String.format("%d in queue.", scheduler.taskCount()));
        telemetry.update();

        waitForStart(); // Wait for start button

        telemetry.update();
        scoreTimer.reset();
        finalizeTimer.reset();

        while (scheduler.hasJobs() && opModeIsActive()) {
            scheduler.tick();
            scheduler.displayStatus(true, true,
                    str -> {
                        telemetry.addLine(str);
                        return Unit.INSTANCE;
                    });
            telemetry.update();
        }
        doneIn = finalizeTimer.time();
        while (opModeIsActive()) {
            hardware.driveMotors.setAll(0);
            telemetry.addLine(String.format("done in %.2fs", doneIn));
            telemetry.addLine(String.format("scored in %.2fs", scoredIn.get()));
            telemetry.addData("x", tracker.getPose().x()); // Print x attribute for pose
            telemetry.addData("y", tracker.getPose().y()); // Print y attribute for pose
            telemetry.addData("heading (rad)", tracker.getPose().heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(tracker.getPose().heading())); // Print the heading in degrees
            telemetry.addLine(String.format("While running: %.2fms per loop", loopTimer.getAvg() * 1000));
            telemetry.update();
        }
    }
}

