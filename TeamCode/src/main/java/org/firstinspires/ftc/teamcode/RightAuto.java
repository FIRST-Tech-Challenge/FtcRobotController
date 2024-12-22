package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BlinkLightsTask;
import org.firstinspires.ftc.teamcode.hardware.HClawProxy;
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy;
import org.firstinspires.ftc.teamcode.hardware.VLiftProxy;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Motion;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveRelTask;
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
import kotlin.Pair;
import kotlin.Unit;

@Autonomous // Appear on the autonomous drop down
// STOP POSTING ABOUT DefaultLocale IM TIRED OF SEEING IT
@SuppressLint("DefaultLocale")
public class RightAuto extends LinearOpMode {
    // Constants //
    public static final double ACCEPT_DIST = 1; // inch. euclidean distance
    public static final double ACCEPT_TURN = Math.toRadians(5);
    // power biases
    public static final Motion.Calibrate CALIBRATION = new Motion.Calibrate(1.0, 1.0, 1.0); // Calibration factors for strafe, forward, and turn.
    public static final double CLAW_OPEN = 0.55;
    public static final double CLAW_CLOSE = 0.02;
    public static final double WRIST_UP = 0.46;
    public static final double WRIST_BACK = 0.30;
    public static final double VERTICAL_SLIDE_SPEED = 0.75;
    private static final RuntimeException NOT_IMPLEMENTED = new RuntimeException("This operation is not implemented");
    final Pose PARK = new Pose(4, -36, Math.toRadians(0));
    Hardware hardware;
    private final Runnable setup = () -> {
        hardware.claw.setPosition(CLAW_CLOSE);
        hardware.wrist.setPosition(0.28);
        hardware.arm.setTargetPosition(0);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.2);
    };
    EncoderTracking tracker;
    private VLiftProxy vLiftProxy;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;
    private Ramps ramps;
    private Ramps rampsSlowEdition;
    private LoopStopwatch loopTimer;
    private Speed2Power speed2Power;
    private MultitaskScheduler scheduler;

    private MoveRelTask moveRel(Pose offset) {
        return new MoveRelTask(
                scheduler, hardware, offset, tracker, loopTimer, speed2Power, ramps, telemetry
        );
    }

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

    private MoveToTask moveToSlow(Pose target) {
        return new MoveToTask(
                scheduler, hardware, target, tracker, loopTimer, speed2Power, rampsSlowEdition, telemetry
        );
    }

    private BlinkLightsTask blinkenlights(double seconds) {
        return new BlinkLightsTask(
                scheduler, hardware,
                seconds, true, 4 /* Hz */,
                Hardware.LAMP_RED, Hardware.LAMP_ORANGE, Hardware.LAMP_PURPLE
        );
    }

    private BlinkLightsTask lightColor(double color) {
        return new BlinkLightsTask(
                scheduler, hardware,
                0.0, false, 0,
                0, 0, color
        );
    }

    private void hardwareInit() {
        tracker = new EncoderTracking(hardware);
        loopTimer = new LoopStopwatch();
        speed2Power = new Speed2Power(0.20); // Set a speed2Power corresponding to a speed of 0.20 seconds
        ramps = new Ramps(
                Ramps.linear(2.0),
                Ramps.linear(1 / 12.0),
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );
        rampsSlowEdition = new Ramps(
                x -> 0.25,
                Ramps.linear(1 / 12.0),
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );

        hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.verticalSlide.setTargetPosition(0);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.clawFlip.setPosition(Hardware.FLIP_UP);
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);

        hardware.arm.setTargetPosition(0);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.3);
        hardware.wrist.setPosition(0.28);
        hardware.claw.setPosition(Hardware.CLAW_CLOSE);

        // we don't have the proxy object to handle this for us
        // so manually implement the inversion
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1 - Hardware.RIGHT_SLIDE_IN);

        hardware.lightLeft.setPosition(Hardware.LAMP_PURPLE);
        hardware.lightRight.setPosition(Hardware.LAMP_PURPLE);
    }

    private Pair<ITask, ITask> specimenWallPick() {
        ITask first = scheduler.add(run(() -> hardware.claw.setPosition(CLAW_OPEN)));
        ITask last = first
                .then(wait(1.000))
                .then(run(() -> hardware.wrist.setPosition(WRIST_UP)))
                .then(wait(1.000))
                .then(run(() -> hardware.arm.setTargetPosition(45)))
                .then(wait(0.500))
                .then(run(() -> hardware.claw.setPosition(CLAW_CLOSE)))
                .then(wait(1.000))
                .then(run(() -> {
                    hardware.verticalSlide.setTargetPosition(300);
                    hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hardware.verticalSlide.setPower(VERTICAL_SLIDE_SPEED);
                }))
                .then(wait(1.000))
                .then(run(() -> hardware.wrist.setPosition(WRIST_BACK)))
                .then(wait(1.000))
                .then(run(() -> hardware.arm.setTargetPosition(10)))
                .then(wait(1.000))
                .then(run(() -> hardware.verticalSlide.setTargetPosition(0)));
        return new Pair<>(first, last);
    }

    private ITask grab() {
        final double flipThird = 0.66;
        ITask result = groupOf(inner -> inner.add(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                .then(hSlideProxy.moveOut())
                .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                .then(await(500))
                .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE))
                .then(await(250))
                .then(hClawProxy.aSetFlip(flipThird))
                .then(hSlideProxy.moveIn())
                .then(hClawProxy.aSetFlip(Hardware.FLIP_UP)));
        return result;
    }

    private ITask transfer() {
        return groupOf(
                it -> it.add(vLiftProxy.moveTo(0, 5, 1.0))
                        .then(run(() -> {
                            hClawProxy.setClaw(Hardware.FRONT_CLOSE);
                            hardware.claw.setPosition(Hardware.CLAW_OPEN);
                        }))
                        .then(await(250))
                        .then(run(() -> {
                            hardware.wrist.setPosition(0);
                            hardware.arm.setTargetPosition(-28);
                        }))
                        .then(await(500))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(250))
                        .then(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                        .then(await(250))
                        .then(run(() -> {
                            hardware.arm.setTargetPosition(0);
                            hardware.wrist.setPosition(Hardware.WRIST_BACK);
                        }))
        );
    }

    private ITask drop() {
        return groupOf(it -> it.add(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
                .then(run(() -> hardware.wrist.setPosition(0.75)))
                .then(await(200))
                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                .then(await(200))
                .then(run(() -> {
                    hardware.wrist.setPosition(Hardware.WRIST_BACK);
                    hardware.arm.setTargetPosition(0);
                }))
        );
    }

    private ITask preScoreSpecimen() {
        return groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(-99))))
                .then(vLiftProxy.moveTo(710, 5, 1.0)));
    }

    private ITask postScoreSpecimen() {
        return groupOf(it -> it.add(run(() -> {
                    hardware.wrist.setPosition(0.28);
                    hardware.arm.setTargetPosition(Hardware.deg2arm(0));
                }))
                .then(await(450))
                .then(vLiftProxy.moveTo(0, 5, .25)));
    }

    private ITask scoreSpecimen() {
        return groupOf(it -> it.add(run(() -> hardware.wrist.setPosition(1)))
                .then(await(300))
                .then(moveRel(new Pose(-4.0, 0, 0)))
                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                .then(await(200))
        );
    }

    private ITask pickSpecimen() {
        return groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(await(200))
                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_UP)))
                        .then(await(200))
                        .then(run(() -> hardware.arm.setTargetPosition(50)))
                        .then(await(500))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(200))
                        .then(vLiftProxy.moveTo(225, 5, 0.4))
                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_BACK)))
//                        .then(await(200))
                        .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
                        .then(await(200))
                        .then(vLiftProxy.moveTo(0, 5, 0))
        );
    }

    public void runAuto() {
        scheduler.add(new OneShot(scheduler, setup))
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(30, 12, 0)));
                    a.add(preScoreSpecimen());
                }))
                .then(scoreSpecimen())
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(15.5, -36, 0)));
                    a.add(postScoreSpecimen());
                }))
                .then(grab())
                .then(groupOf(a -> {
                    a.add(transfer())
                            .then(drop());
                    a.add(moveTo(new Pose(11.5, -36, 0)));
                }))
                .then(moveTo(new Pose(15.5, -46.25, 0)))
                .then(grab())
                .then(groupOf(a -> {
                    a.add(transfer())
                            .then(drop());
                    a.add(moveTo(new Pose(11.5, -46.25, 0)));
                }))
//                .then(moveTo(new Pose(14, -27, 0)))
//                .then(await(500))
                .then(moveTo(new Pose(6, -27, 0)))
                .then(blinkenlights(1.0))
                .then(moveToSlow(new Pose(2, -27, 0)))
                .then(run(() -> hardware.driveMotors.setAll(-0.40)))
                .then(await(500))
                .then(run(() -> hardware.driveMotors.setAll(0)))
                .then(pickSpecimen())
                .then(lightColor(Hardware.LAMP_PURPLE))
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(30, 6, 0)));
                    a.add(preScoreSpecimen());
                }))
                .then(scoreSpecimen())
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(6, -27, 0)));
                    a.add(postScoreSpecimen());
                }));
    }

    @Override
    public void runOpMode() {
        scheduler = new MultitaskScheduler();
        hardware = new Hardware(hardwareMap);

        hardwareInit();

        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, hardware.verticalSlide));
        hSlideProxy = scheduler.add(new HSlideProxy(scheduler, hardware));
        hClawProxy = scheduler.add(new HClawProxy(scheduler, hardware));

        ElapsedTime finalizeTimer = new ElapsedTime();
//        AtomicReference<Double> scoredIn = new AtomicReference<>((double) 0);
        double doneIn = 0;

        // queue everything up
        scheduler.add(new BackgroundTasks(
                scheduler, tracker, loopTimer
        ));
//        mainAuto();
        runAuto();
        // park
        hardware.claw.setPosition(CLAW_CLOSE);

        telemetry.addLine("Initialized.");
        telemetry.addLine(String.format("%d in queue.", scheduler.taskCount()));
        telemetry.update();

        waitForStart(); // Wait for start button

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
//            telemetry.addLine(String.format("scored in %.2fs", scoredIn.get()));
            telemetry.addData("x", tracker.getPose().x()); // Print x attribute for pose
            telemetry.addData("y", tracker.getPose().y()); // Print y attribute for pose
            telemetry.addData("heading (rad)", tracker.getPose().heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(tracker.getPose().heading())); // Print the heading in degrees
            telemetry.addLine(String.format("While running: %.2fms per loop", loopTimer.getAvg() * 1000));
            telemetry.update();
        }
    }

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

