package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Locks;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Set;
import java.util.function.Consumer;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.ITaskWithResult;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskGroup;
import dev.aether.collaborative_multitasking.TaskTemplate;
import dev.aether.collaborative_multitasking.TaskWithResultTemplate;
import dev.aether.collaborative_multitasking.ext.Pause;
import dev.aether.collaborative_multitasking.ext.While;
import kotlin.jvm.functions.Function0;


@TeleOp
public class MecanumTeleOp2 extends LinearOpMode {
    private Hardware hardware;
    double wristPos = 0.28;
    double twistPos = 0.17;
    double VerticalSlideSpeed = 0.75;
    double ClawFrontPos = 0.5;
    double ClawFlipPos = 0.5;
    double hslidepos = 0.05;
    double horizontalSlide = 0;

    private LiftBackgroundTask liftProxy;

    /**
     * How much you need to push the joysticks to stop autonomous code.
     */
    public static final double PUSH_TO_BYPASS = 0.20;

    private MultitaskScheduler scheduler;
    private NavxMicroNavigationSensor navxMicro;

    /**
     * Forces any tasks using this lock to be stopped immediately.
     *
     * @param theLockInQuestion which lock to forcibly acquire
     */
    private void abandonLock(SharedResource theLockInQuestion) {
        scheduler.filteredStop(it -> it.requirements().contains(theLockInQuestion));
    }

    private OneShot run(Runnable target) {
        return new OneShot(scheduler, target);
    }

    private Pause await(double milliseconds) {
        return new Pause(scheduler, milliseconds / 1000.0);
    }

    private While doWhile(Function0<Boolean> condition, Runnable action) {
        return new While(scheduler, condition, action);
    }

    private TaskGroup groupOf(Consumer<Scheduler> contents) {
        return new TaskGroup(scheduler).with(contents);
    }

    private void hardwareInit() {
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.verticalSlide.setTargetPosition(0);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hardware.arm.setTargetPosition(0);
        armTargetPosDeg = 0.0;
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.2);
        hardware.wrist.setPosition(0.28);
        hardware.twist.setPosition(twistPos);

        navxMicro = hardware.gyro;
    }

    private static class LiftBackgroundTask extends TaskTemplate {
        private static final double SPEED = .75;
        private static int INSTANCE_COUNT = 0;
        private static final int MAX_VERTICAL_LIFT_TICKS = 2300;
        private static final int MIN_VERTICAL_LIFT_TICKS = 0;

        private boolean manualAdjustMode = false;

        private final DcMotor lift;

        private int targetPosition = 0;

        /// If you hold this lock, you have exclusive control over the Lift (by proxy of this task.)
        public final SharedResource CONTROL = new SharedResource("LiftBackgroundTask" + (++INSTANCE_COUNT));
        private final Set<SharedResource> provides = Set.of(CONTROL);

        private static final Set<SharedResource> requires = Set.of(Locks.VerticalSlide);
        private final Scheduler scheduler;

        @Override
        @NotNull
        public Set<SharedResource> requirements() {
            return requires;
        }

        public LiftBackgroundTask(@NotNull Scheduler scheduler, DcMotor lift) {
            super(scheduler);
            this.lift = lift;
            this.scheduler = scheduler;
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
            if (maxDuration > 0) result = new TaskWithResultTemplate<Boolean>(scheduler) {
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

    @Override
    public void runOpMode() {
        // this scheduler will only be used in init
        scheduler = new MultitaskScheduler();

        hardware = new Hardware(hardwareMap);
        hardwareInit();

        ElapsedTime timer = new ElapsedTime();

        telemetry.log().add("Calibrating gyroscope. Don't touch...");

        scheduler.add(doWhile(
                navxMicro::isCalibrating,
                () -> {
                    telemetry.addData("Calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
                    telemetry.update();
                }
        ));

        // Wait until the gyro calibration is complete
        timer.reset();

        scheduler.runToCompletion(() -> !isStopRequested());

        // this is the scheduler that will be used during the main program
        scheduler = new MultitaskScheduler();
        liftProxy = scheduler.add(new LiftBackgroundTask(scheduler, hardware.verticalSlide));

        telemetry.log().clear();
        telemetry.log().add("Set and ready to roll!");
        telemetry.clear();

        waitForStart();
        if (isStopRequested()) return;

        double yaw_offset = 0.0;
        while (opModeIsActive()) {

            Orientation angles = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (gamepad1.back) {
                yaw_offset = angles.firstAngle;
            }
            double botheading = angles.firstAngle - yaw_offset;
            telemetry.addData("Heading", formatAngle(angles.angleUnit, botheading));
//                    .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
//                    .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
//                    .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle))

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (abs(y) + abs(x) >= PUSH_TO_BYPASS) abandonLock(Locks.DriveMotors);

            if (!scheduler.isResourceInUse(Locks.DriveMotors)) {
                double rotX = x * Math.cos(-botheading) - y * Math.sin(-botheading);
                double rotY = x * Math.sin(-botheading) + y * Math.cos(-botheading);
                rotX *= 1.1; // Counteract imperfect strafing

                telemetry.addLine()
                        .addData("rotX", rotX)
                        .addData("rotY", rotY);
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(abs(rotY) + abs(rotX) + abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                double maxSpeed = 1.0;
                double slowSpeed = 0.5;
                double currentSpeed = maxSpeed;
                if (gamepad1.left_bumper) {
                    currentSpeed = slowSpeed;
                }
                hardware.frontLeft.setPower(frontLeftPower * currentSpeed);
                hardware.backLeft.setPower(backLeftPower * currentSpeed);
                hardware.frontRight.setPower(frontRightPower * currentSpeed);
                hardware.backRight.setPower(backRightPower * currentSpeed);

                telemetry.addData("fl power", frontLeftPower);
                telemetry.addData("fr power", frontRightPower);
                telemetry.addData("bl power", backLeftPower);
                telemetry.addData("br power", backRightPower);
            }
            wrist();
            trasfer();
            claw();
//            twist();
            stepper();
            lift();
            HSlide();
            arm();

            if (gamepad2.y) {
                ScoreHighBasket();
            }
            if (gamepad2.x) {
                PickUpYellow();
            }
            if (gamepad2.b) {
                specimenWallPick();
            }
            if (gamepad2.dpad_left) {
                score();
            }
            if (gamepad1.right_trigger > 0.5) Flipin();
            if (gamepad1.left_trigger > 0.5) Flipout();

            int verticalPosition = hardware.encoderVerticalSlide.getCurrentPosition();
            telemetry.addData("Wrist Position", hardware.wrist.getPosition());
            telemetry.addData("Claw Position", hardware.claw.getPosition());
            telemetry.addData("Vertical position", verticalPosition);
            telemetry.update();
        }

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    @SuppressLint("DefaultLocale")
    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /////////////////////////////////////////////

    // These are in the LiftProxy
    int highChamberTicks = 790;
    int highBasketTicks = 2180;

    // lifts the vertical slides to a target position in ticks

    private ITaskWithResult<Boolean> targetLift(int targetPosition) {
        abandonLock(liftProxy.CONTROL);
        return scheduler.add(liftProxy.moveTo(targetPosition, 5, 3.0));
    }

    private @Nullable ITaskWithResult<Boolean> aButtonTask = null;

    private void lift() {
        liftProxy.controlManual(gamepad2.dpad_up, gamepad2.dpad_down);

        if (gamepad2.a) {
            if (aButtonTask == null || aButtonTask.getState() != ITask.State.Ticking) {
                if (aButtonTask != null) {
                    aButtonTask.requestStop(true);
                }
                aButtonTask = targetLift(0);
            }
        }
    }

    double armTargetPosDeg = 0.0;
    int liftMinClearanceTicks = 350;

    private double getArmPosDeg() {
        double rotations = hardware.arm.getCurrentPosition() / Hardware.spinTickPerRev;
        // 0 = straight down
        return rotations * 360.0;
    }

    private void arm() {
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        // 537.7 ppr
        DcMotor arm = hardware.arm;
        double stick_pos = -gamepad2.right_stick_y;
        double rotations = arm.getCurrentPosition() / Hardware.spinTickPerRev;
        double degrees = rotations * 360.0; // 0 = straight down
        // Negative: towards front;
        // Positive: towards back.
        // Exclusion zone 0 to -25deg whe lift < 6in.
        // [removed if statement, disabled]
        // Full* clearance
        if (stick_pos > 0.7 && armTargetPosDeg <= 110) {
            armTargetPosDeg += 1;
        }
        if (stick_pos < -0.7 && armTargetPosDeg >= -110) {
            armTargetPosDeg -= 1;
        }
        arm.setTargetPosition(Hardware.deg2arm(armTargetPosDeg));
        arm.setPower(0.3);
        telemetry.addData("arm deg", degrees);
    }

    public void wrist() {
        boolean write = false;
        if (gamepad2.left_stick_y >= 0.5 && gamepad2.left_stick_x >= -0.25 && gamepad2.left_stick_x <= 0.25) {
            wristPos += 0.01;
            write = true;
        } else if (gamepad2.left_stick_y <= -0.5 && gamepad2.left_stick_x >= -0.25 && gamepad2.left_stick_x <= 0.25) {
            wristPos -= 0.01;
            write = true;
        }
        if (write) {
            abandonLock(Locks.ArmAssembly);
            hardware.wrist.setPosition(wristPos);
        }
        telemetry.addData("Wrist Position", wristPos);
    }

    public void claw() {
        final double open = 0.02;
        final double close = 0.55;
        if (gamepad2.left_bumper) {
            abandonLock(Locks.ArmAssembly);
            hardware.claw.setPosition(open);
        } else if (gamepad2.right_bumper) {
            abandonLock(Locks.ArmAssembly);
            hardware.claw.setPosition(close);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    public void ScoreHighBasket() {
        scheduler.add(
                groupOf(inner -> inner.add(liftProxy.moveTo(highBasketTicks, 5, 2.0))
                        .then(run(() -> hardware.arm.setTargetPosition(222)))
                        .then(await(500))
                        .then(run(() -> hardware.wrist.setPosition(0.94)))
                        .then(await(500))
                        .then(run(() -> hardware.claw.setPosition(0.02)))
                        .then(await(500))
                        .then(run(() -> hardware.claw.setPosition(0.55)))
                        .then(await(100))
                        .then(run(() -> hardware.wrist.setPosition(0.28)))
                        .then(await(500))
                        .then(run(() -> hardware.arm.setTargetPosition(0)))
                        .then(await(500))
                        .then(liftProxy.moveTo(0, 5, 2.0))
                ).extraDepends(
                        Locks.ArmAssembly,
                        liftProxy.CONTROL
                )
        );
    }

    public void PickUpYellow() {
        // Scheduled to be rewritten -miles
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        hardware.verticalSlide.setTargetPosition(224);
        sleep(500);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.5);
        hardware.arm.setTargetPosition(67);
        sleep(500);
        hardware.wrist.setPosition(0.94);
        sleep(500);
        hardware.claw.setPosition(0.02);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(110);
        sleep(500);
        hardware.claw.setPosition(0.55);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(200);
        sleep(500);
        hardware.wrist.setPosition(0.28);
        sleep(500);
        hardware.arm.setTargetPosition(0);
        sleep(500);
    }

    private void PickUpSpecimen() {
        // Lift --> arm out --> Lift to 0 --> move wrist --> open claw
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        hardware.verticalSlide.setTargetPosition(224);
        sleep(500);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.5);
        hardware.arm.setTargetPosition(90);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(0);
        sleep(500);
        hardware.wrist.setPosition(1);
        sleep(500);
        hardware.claw.setPosition(0.02); // TBD
        sleep(500);
        hardware.twist.setPosition(0.17);
        sleep(500);
        hardware.arm.setTargetPosition(145);
        sleep(500);
        hardware.claw.setPosition(0.55);
        sleep(500);
        hardware.wrist.setPosition(0.5);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(300);
        sleep(500);
        hardware.arm.setTargetPosition(0);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(0);
        sleep(500);
        armTargetPosDeg = 0;
    }

    private void transfer() {
        // Technically still enabled in normal MechanumTeleOp, but likely error
        //hardware.arm.setPower(-0.5);
        hardware.verticalSlide.setTargetPosition(900);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        sleep(500);
        hardware.arm.setTargetPosition(-63);//This is in ticks
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.5);
        armTargetPosDeg = -30;//(752 ticks /360 degrees)
        sleep(500);
        hardware.wrist.setPosition(0.9);
        sleep(2000);
        hardware.claw.setPosition(0.02);
        sleep(2000);
        hardware.verticalSlide.setTargetPosition(735);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        sleep(2000);
        hardware.claw.setPosition(0.55);
        sleep(1000);
        hardware.wrist.setPosition(0.45);
        hardware.arm.setTargetPosition(0);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.5);
        armTargetPosDeg = 0;
        sleep(1000);
        hardware.verticalSlide.setTargetPosition(0);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);

        //To-Do: open horizontal claw


    }

    private void stepper() {
        // this deals with the horizontal slide stuff. we don't have locks (yet) so it's remaining
        // as-is for now
        if (gamepad1.dpad_left) {
            ClawFrontPos = 0.33;
        }
        if (gamepad1.dpad_right) {
            ClawFrontPos = 0.07;
        }
        if (gamepad1.dpad_up) {
            ClawFlipPos -= 0.01;
        }
        if (gamepad1.dpad_down) {
            ClawFlipPos += 0.01;
        }
        hardware.clawFlip.setPosition(ClawFlipPos);
        hardware.clawFront.setPosition(ClawFrontPos);
        // clawFront close is 0
        //clawFront open is 0.27

        telemetry.addData("FrontClawPos", ClawFrontPos);
        telemetry.addData("FlipClawPos", ClawFlipPos);
    }

    public void HSlide() {
        // same note as above.
        if (gamepad1.y && hslidepos < 1) {
            hslidepos += 0.01;
        }
        if (gamepad1.a && hslidepos > 0) {
            hslidepos += -0.01;
        }
        hardware.horizontalSlide.setPosition(hslidepos);
        telemetry.addData("Horizontal Position", hardware.horizontalSlide.getPosition());
    }

    public void specimenWallPick() {
        // todo: move these into Hardware.java
        double clawopen = 0.55;
        double clawclose = 0.02;
        double wristup = 0.46;
        double wristback = 0.30;
        double slideup = 200;
        double armout = 23.94;

        scheduler.add(
                groupOf(it -> it.add(run(() -> hardware.claw.setPosition(clawopen)))
                        .then(await(1000))
                        .then(run(() -> hardware.wrist.setPosition(wristup)))
                        .then(await(1000))
                        .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(45))))
                        // FIXME: get that global variable out of here
                        .then(await(500))
                        .then(run(() -> hardware.claw.setPosition(clawclose)))
                        .then(await(1000))
                        .then(liftProxy.moveTo(300, 5, 1.0))
                        .then(run(() -> hardware.wrist.setPosition(wristback)))
                        .then(await(1000))
                        // TODO: investigate if 10 ticks or 10 degrees is the right number
                        .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
                        .then(await(1000))
                        .then(liftProxy.moveTo(0, 5, 0))
                ).extraDepends(
                        liftProxy.CONTROL,
                        Locks.ArmAssembly
                )
        );
    }

    private void score() {
        double clawclose = 0.02;

        scheduler.add(
                groupOf(it -> it.add(run(() -> hardware.claw.setPosition(clawclose)))
                        .then(liftProxy.moveTo(710, 5, 1.0))
                        .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(-99))))
                        .then(await(1000))
                        .then(run(() -> hardware.wrist.setPosition(1)))
//                        .then(await(1000))
//                        .then(moveRel(new Pose(-3.5, 0, 0)))
//                        .then(run(() -> hardware.claw.setPosition(clawopen)))
//                        .then(await(500))
//                        // Maybe let the rest of this be async
//                        .then(run(() -> {
//                            hardware.wrist.setPosition(0.28);
//                            hardware.arm.setTargetPosition(Hardware.deg2arm(0));
//                        }))
//                        .then(await(1000))
//                        .then(run(() -> hardware.verticalSlide.setTargetPosition(0)))
//                        .then(await(250))

                ).extraDepends(
                        liftProxy.CONTROL,
                        Locks.ArmAssembly
                )
        );
    }

    public void Horizontalpick() {
        double hslideout = 0.35;
        double flipdown = 0.04;
        double frontopen = 0.33;
        double frontclose = 0.07;
        double flipup = 0.98;
        double hslidein = 0.1;
        hardware.horizontalSlide.setPosition(hslideout);
        sleep(500);
        hardware.clawFlip.setPosition(flipdown);
        sleep(500);
        hardware.clawFront.setPosition(frontopen);
        sleep(500);
        hardware.clawFront.setPosition(frontclose);
        ClawFrontPos = frontclose;
        sleep(500);
        hardware.clawFlip.setPosition(flipup);
        sleep(500);
        hardware.horizontalSlide.setPosition(hslidein);
        sleep(500);
    }

    public void Flipout() {
        double hslideout = 0.35;
        double flipdown = 0.04;
        hardware.horizontalSlide.setPosition(hslideout);
        hslidepos = hslideout;
        sleep(500);
        hardware.clawFlip.setPosition(flipdown);
        ClawFlipPos = flipdown;
        sleep(500);
    }

    public void Flipin() {
        double flipup = 0.98;
        double hslidein = 0.1;
        hardware.clawFlip.setPosition(flipup);
        ClawFlipPos = flipup;
        sleep(500);
        hardware.horizontalSlide.setPosition(hslidein);
        hslidepos = hslidein;
        sleep(500);


    }

    public void trasfer() {
        if (gamepad2.x) {
            hardware.clawFront.setPosition(0.07);
            ClawFrontPos = 0.07;
            hardware.claw.setPosition(0.55);
            sleep(500);
            hardware.wrist.setPosition(0);
            sleep(500);
            hardware.arm.setTargetPosition(-28);
            armTargetPosDeg = -8;
            sleep(1000);
            hardware.claw.setPosition(0.02);
            sleep(500);
            hardware.clawFront.setPosition(0.33);
            sleep(500);
            hardware.arm.setTargetPosition(0);
            armTargetPosDeg = 0;
            sleep(500);
            hardware.wrist.setPosition(0.28);
            ClawFrontPos = 0.28;
        }
    }
}
