package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskGroup;
import dev.aether.collaborative_multitasking.TaskTemplate;
import dev.aether.collaborative_multitasking.ext.Pause;
import kotlin.Pair;
import kotlin.Unit;

@Autonomous // Appear on the autonomous drop down
// STOP POSTING ABOUT DefaultLocale IM TIRED OF SEEING IT
@SuppressLint("DefaultLocale")
public class LeftAuto extends LinearOpMode {
    // Constants //
    public static final double ACCEPT_DIST = 1; // inch. euclidean distance
    public static final double ACCEPT_TURN = Math.toRadians(5);
    public static final double VERTICAL_SLIDE_SPEED = 0.75;
    public static final int HIGH_BASKET_TICKS = 2180;
    public static final int PICK_UP_TICKS = 224;
    public static final int PICK_UP_TICKS_2 = 75;
    public static final int PICK_UP_TICKS_3 = 200;
    public static final int ARM_BASKET_TICKS = 222;
    public static final int ARM_PICK_UP_TICKS = 67;
    public static final double FLIP_UP = 0.98;
    public static final double FLIP_DOWN = 0.04;
    public static final double H_SLIDE_OUT = 0.35;
    public static final double H_SLIDE_IN = 0.1;
    public static final double CLAW_OPEN = 0.55;
    public static final double CLAW_CLOSE = 0.02;
    public static final double WRIST_UP = 0.46;
    public static final double WRIST_BACK = 0.30;
    // power biases
    public static final Motion.Calibrate CALIBRATION = new Motion.Calibrate(1.0, 1.0, 1.0); // Calibration factors for strafe, forward, and turn.
    private static final RuntimeException NOT_IMPLEMENTED = new RuntimeException("This operation is not implemented");
    final Pose SCORE_HIGH_BASKET = new Pose(9.8786797, 18.1213203, Math.toRadians(-45));
    final Pose PARK = new Pose(12, 16, Math.toRadians(0));
    Hardware hardware;
    EncoderTracking tracker;
    private VLiftProxy vLiftProxy;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;
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

    private Pair<ITask, ITask> flipOut() {
        ITask first = scheduler.add(run(() -> hardware.horizontalSlide.setPosition(H_SLIDE_OUT)));
        ITask last = first
                .then(wait(0.500))
                .then(run(() -> hardware.clawFlip.setPosition(FLIP_DOWN)))
                .then(wait(0.500));
        return new Pair<>(first, last);
    }

    private Pair<ITask, ITask> flipIn() {
        ITask first = scheduler.add(run(() -> hardware.clawFlip.setPosition(FLIP_UP)));
        ITask last = first
                .then(wait(0.500))
                .then(run(() -> hardware.horizontalSlide.setPosition(H_SLIDE_IN)))
                .then(wait(0.500));
        return new Pair<>(first, last);
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

    private ITask pickUpYellow() {
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
        result.then(transfer());
        return result;
    }

    private ITask scoreHighBasket() {
        return groupOf(inner -> inner.add(vLiftProxy.moveTo(HIGH_BASKET_TICKS, 5, 2.0))
                .then(run(() -> hardware.arm.setTargetPosition(222)))
                .then(await(700))
                .then(run(() -> hardware.wrist.setPosition(0.94)))
                .then(await(700))
                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                .then(await(100))
                .then(run(() -> hardware.wrist.setPosition(0.28)))
                .then(run(() -> hardware.arm.setTargetPosition(0)))
                .then(vLiftProxy.moveTo(0, 5, 2.0)));
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

    @Override
    public void runOpMode() {
        scheduler = new MultitaskScheduler();
        hardware = new Hardware(hardwareMap);

        hardwareInit();

        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, hardware.verticalSlide));
        hSlideProxy = scheduler.add(new HSlideProxy(scheduler, hardware));
        hClawProxy = scheduler.add(new HClawProxy(scheduler, hardware));

        ElapsedTime finalizeTimer = new ElapsedTime();
        double doneIn = 0;

        scheduler.add(new BackgroundTasks(
                scheduler, tracker, loopTimer
        ));
        scheduler
                .add(moveTo(SCORE_HIGH_BASKET))
                .then(scoreHighBasket())
                .then(moveTo(new Pose(16.5, 13.5, Math.toRadians(0))))
                .then(pickUpYellow())
                .then(moveTo(SCORE_HIGH_BASKET))
                .then(scoreHighBasket())
                .then(moveTo(new Pose(16.5, 23.75, Math.toRadians(0))))
                .then(pickUpYellow())
                .then(moveTo(SCORE_HIGH_BASKET))
                .then(scoreHighBasket())
//                .then(moveTo(SCORE_HIGH_BASKET))
//                .then(scoreHighBasket())
//                .then(moveTo(new Pose(28, 22, Math.toRadians(-180))))
//                .then(pickUpYellow(scheduler))
//                .then(moveTo(SCORE_HIGH_BASKET))
//                .then(scoreHighBasket())
//                .then(new OneShot(scheduler, () -> scoredIn.set(finalizeTimer.time())))
                .then(moveTo(PARK));
//                .then(moveTo(scheduler,
//                        new Pose(65, -12, Math.toRadians(0))))
        ;
        // park

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
            telemetry.addData("x", tracker.getPose().x()); // Print x attribute for pose
            telemetry.addData("y", tracker.getPose().y()); // Print y attribute for pose
            telemetry.addData("heading (rad)", tracker.getPose().heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(tracker.getPose().heading())); // Print the heading in degrees
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

    final static class PickUpYellow extends TaskTemplate {
        private final DcMotor verticalSlide;
        private final DcMotor arm;
        private final Servo wrist;
        private final Servo claw;
        private final Scheduler scheduler = getScheduler();
        private final Set<SharedResource> requirements = Set.of(
                Hardware.Locks.DriveMotors,
                Hardware.Locks.VerticalSlide,
                Hardware.Locks.ArmAssembly
        );
        private ITask target = null;
        private List<ITask> subTasks = new ArrayList<>();

        public PickUpYellow(@NotNull Scheduler scheduler, Hardware hardware) {
            super(scheduler);
            verticalSlide = hardware.verticalSlide;
            arm = hardware.arm;
            wrist = hardware.wrist;
            claw = hardware.claw;
            ITask head = scheduler
                    .task(that -> {
                        that.canStart(() -> this.getState() == State.Ticking);
                        that.isCompleted(() -> true);
                        return Unit.INSTANCE;
                    });
            target = head
                    .then(run(() -> {
                        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        verticalSlide.setPower(VERTICAL_SLIDE_SPEED);
                        verticalSlide.setTargetPosition(PICK_UP_TICKS);
                    }))
                    .then(pause(0.500))
                    .then(run(() -> {
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.5);
                        arm.setTargetPosition(ARM_PICK_UP_TICKS);
                        wrist.setPosition(0.94);
                        claw.setPosition(0.02);
                    }))
                    .then(pause(0.750))
                    .then(run(() -> verticalSlide.setTargetPosition(PICK_UP_TICKS_2)))
                    .then(pause(0.500))
                    .then(run(() -> claw.setPosition(0.55)))
                    .then(pause(0.500))
                    .then(run(() -> verticalSlide.setTargetPosition(PICK_UP_TICKS_3)))
                    .then(pause(0.500))
                    .then(run(() -> wrist.setPosition(0.28)))
                    .then(run(() -> arm.setTargetPosition(0)));
        }

        private ITask run(Runnable r) {
            ITask t = new OneShot(scheduler, r);
            subTasks.add(t);
            return t;
        }

        private ITask pause(double seconds) {
            ITask t = new Pause(scheduler, seconds);
            subTasks.add(t);
            return t;
        }

        @Override
        @NotNull
        public Set<SharedResource> requirements() {
            return requirements;
        }

        @Override
        public void invokeOnStart() {
        }

        @Override
        public void invokeOnFinish() {
            scheduler.filteredStop(subTasks::contains, true);
        }

        @Override
        public boolean invokeIsCompleted() {
            return target.getState() == State.Finished || target.getState() == State.Cancelled;
        }
    }
}

