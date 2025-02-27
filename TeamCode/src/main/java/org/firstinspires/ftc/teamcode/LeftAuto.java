package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.AscentProxy;
import org.firstinspires.ftc.teamcode.hardware.HClawProxy;
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy;
import org.firstinspires.ftc.teamcode.hardware.VLiftProxy;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Motion;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveToTask;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;
import org.jetbrains.annotations.NotNull;

import java.util.function.Consumer;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.TaskGroup;
import dev.aether.collaborative_multitasking.TaskTemplate;
import dev.aether.collaborative_multitasking.ext.Pause;
import kotlin.Unit;

@Autonomous // Appear on the autonomous drop down
// STOP POSTING ABOUT DefaultLocale IM TIRED OF SEEING IT
@SuppressLint("DefaultLocale")
public class LeftAuto extends LinearOpMode {
    // power biases
    public static final Motion.Calibrate CALIBRATION = new Motion.Calibrate(1.0, 1.0, 1.0); // Calibration factors for strafe, forward, and turn.
    private static final RuntimeException NOT_IMPLEMENTED = new RuntimeException("This operation is not implemented");
    final Pose SCORE_HIGH_BASKET = new Pose(9.9216797, 18.0783203, Math.toRadians(-45));
    final Pose PARK_BAD = new Pose(10.6286797, 17.3713203, Math.toRadians(0));
    final Pose PARK1 = new Pose(57.5, 0, Math.toRadians(0));
    final Pose PARK2 = new Pose(55.5, -11, Math.toRadians(0));
    final Pose START = new Pose(0, 4.66, Math.toRadians(0));
    Hardware hardware;
    EncoderTracking tracker;
    private VLiftProxy vLiftProxy;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;
    private AscentProxy ascentProxy;
    private Ramps ramps;
    private LoopStopwatch loopTimer;
    private Speed2Power speed2Power;
    private MultitaskScheduler scheduler;

    private ITask run(Runnable it) {
        return new OneShot(scheduler, it);
    }

    private ITask wait(double seconds) {
        return new Pause(scheduler, seconds);
    }

    private ITask await(double milliseconds) {
        return wait(milliseconds / 1000);
    }

    private TaskGroup groupOf(Consumer<Scheduler> contents) {
        return new TaskGroup(scheduler).with(contents);
    }

    private MoveToTask moveTo(Pose target) {
        return new MoveToTask(
                scheduler, hardware, target, tracker, loopTimer, speed2Power, ramps, telemetry
        );
    }

    private ITask transfer() {
        return groupOf(
                it -> it.add(vLiftProxy.moveTo(0, 5, 1.0))
                        .then(run(() -> {
                            hClawProxy.setClaw(Hardware.FRONT_CLOSE);
                            hardware.claw.setPosition(Hardware.CLAW_OPEN);
                            hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
                            hardware.wrist.setPosition(0);
                            //hardware.arm.setTargetPosition(Hardware.ARM_TRANSFER_POS);
                        }))
                        .then(await(250))
                        .then(run(() -> {

                        }))
                        .then(await(400))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(150))
                        .then(run(() -> {
                            //hardware.arm.setTargetPosition(0);
                            hClawProxy.setClaw(Hardware.FRONT_OPEN);
                        }))
                        .then(await(100))
                        .then(run(() -> hardware.clawFront.setPosition(0.6)))
                        .then(await(250))
                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_BACK)))
        );
    }

    private ITask pickUpYellow() {
        final double flipThird = 0.66;
        ITask result = groupOf(inner -> inner
                .add(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                .then(await(500))
                .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE))
                .then(await(250))
                .then(hClawProxy.aSetFlip(flipThird)));
        result
                .then(hSlideProxy.moveIn())
                .then(hClawProxy.aSetFlip(Hardware.FLIP_UP))
                .then(transfer());
        return result;
    }

    /* Add Ryan's fourthSample() from FixStuffTeleOp.java here */
    private ITask fourthYellow() {
        ITask result = groupOf(inner -> inner.add(hClawProxy.aSetFlipClaw(0.167, Hardware.FRONT_OPEN))
                        .then(await(200))
                        .then(run(() -> hardware.clawTwist.setPosition(0.26)))
//                .then(await(500))
                        .then(hSlideProxy.moveOut())
                        .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                        .then(await(200))
                        .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE))
                        .then(await(300))
                        .then(run(() -> hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT)))
                        .then(await(200))
        );
        result.then(hSlideProxy.moveIn())
                .then(hClawProxy.aSetFlip(Hardware.FLIP_UP))
                .then(transfer());
        return result;
    }

    private ITask scoreHighBasket() {
        return groupOf(inner -> inner.add(groupOf(a -> {
                            // all of these:
                            a.add(vLiftProxy.moveTo(Hardware.VLIFT_SCORE_HIGH, 10, 1.2));
                            //a.add(run(() -> hardware.arm.setTargetPosition(222)));
                            a.add(await(250)); // minimum duration
                        }))
                        .then(run(() -> hardware.wrist.setPosition(0.94)))
                        .then(await(100))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(await(100))
                        .then(run(() -> hardware.wrist.setPosition(0.28)))
                        //.then(run(() -> hardware.arm.setTargetPosition(0)))
        );
    }

    private void hardwareInit() {
        tracker = new EncoderTracking(hardware, START);
        loopTimer = new LoopStopwatch();
        speed2Power = new Speed2Power(0.20); // Set a speed2Power corresponding to a speed of 0.20 seconds
        ramps = new Ramps(
                Ramps.linear(5.0),
                Ramps.linear(1 / 12.0),
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );

        hardware.sharedHardwareInit();
    }

    @Override
    public void runOpMode() {
        scheduler = new MultitaskScheduler();
        hardware = new Hardware(hardwareMap);

        hardwareInit();

        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, hardware.verticalLift));
        hSlideProxy = scheduler.add(new HSlideProxy(scheduler, hardware));
        hClawProxy = scheduler.add(new HClawProxy(scheduler, hardware));
        ascentProxy = scheduler.add(new AscentProxy(scheduler, hardware.ascent));

        hardware.ascent.calibrate(Hardware.ASCENT_INIT_POS);

        ElapsedTime finalizeTimer = new ElapsedTime();
        double doneIn = 0;

        hardware.ascent.setTargetPosition(Hardware.ASCENT_INIT_POS);

        scheduler.add(new BackgroundTasks(
                scheduler, tracker, loopTimer
        ));
        scheduler.add(ascentProxy.target(Hardware.ASCENT_UP_POS));
        scheduler
                .add(moveTo(SCORE_HIGH_BASKET))
                .then(scoreHighBasket())
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(18.0, 13.5, Math.toRadians(0))));
                    a.add(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                        .then(hSlideProxy.moveOut());
                    a.add(vLiftProxy.moveTo(0, 5, 1.0));
                }))
                .then(pickUpYellow())
                .then(moveTo(SCORE_HIGH_BASKET))
                .then(scoreHighBasket())
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(18.0, 23.75, Math.toRadians(0))));
                    a.add(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                            .then(hSlideProxy.moveOut());
                    a.add(vLiftProxy.moveTo(0, 5, 1.0));
                }))
                .then(pickUpYellow())
                .then(moveTo(SCORE_HIGH_BASKET))
                .then(scoreHighBasket())
                // 30.75 + sin(15 deg) * 2.0
                // 12.75 + cos(15 deg) * 2.0
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(31.26763809, 14.68185165, Math.toRadians(75))));
                    a.add(vLiftProxy.moveTo(0, 5, 1.0));
                }))
                .then(fourthYellow())
                .then(moveTo(SCORE_HIGH_BASKET))
                .then(scoreHighBasket())
                .then(groupOf(a -> {
                    a.add(moveTo(PARK_BAD));
                    a.add(vLiftProxy.moveTo(0, 5, 1.0));
                }))
                .then(run(() -> hardware.driveMotors.setAll(0)));
        ;

        telemetry.addLine("Initialized.");
        telemetry.addLine(String.format("%d in queue.", scheduler.taskCount()));
        telemetry.update();

        while (!isStarted() && !isStopRequested()) hardware.ascent.update();

        telemetry.update();
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
            telemetry.addData("x", tracker.getPose().x()); // Print x attribute for pose
            telemetry.addData("y", tracker.getPose().y()); // Print y attribute for pose
            telemetry.addData("heading (rad)", tracker.getPose().heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(tracker.getPose().heading())); // Print the heading in degrees
            telemetry.addData("lift power", hardware.verticalLift.getPower()); // Print the heading in radians
            telemetry.addLine(String.format("While running: %.2fms per loop", loopTimer.getAvg() * 1000));
            telemetry.update();
        }
    }

//    private ScoreHighBasket scoreHighBasket() {
//        return new ScoreHighBasket(scheduler, hardware);
//    }

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

}

