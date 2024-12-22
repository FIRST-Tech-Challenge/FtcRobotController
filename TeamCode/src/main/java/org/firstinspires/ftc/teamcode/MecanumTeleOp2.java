package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;
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
import org.firstinspires.ftc.teamcode.hardware.HClawProxy;
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy;
import org.firstinspires.ftc.teamcode.hardware.VLiftProxy;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveRelTask;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;

import java.util.function.Consumer;

import dev.aether.collaborative_multitasking.ITaskWithResult;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskGroup;
import dev.aether.collaborative_multitasking.ext.Pause;
import dev.aether.collaborative_multitasking.ext.While;
import kotlin.Unit;
import kotlin.jvm.functions.Function0;


@TeleOp
public class MecanumTeleOp2 extends LinearOpMode {
    /**
     * How much you need to push the joysticks to stop autonomous code.
     */
    public static final double PUSH_TO_BYPASS = 0.20;
    double wristPos = 0.28;
    int highBasketTicks = 2180;
    private Hardware hardware;
    private MultitaskScheduler scheduler;
    private NavxMicroNavigationSensor navxMicro;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;
    private EncoderTracking tracker;
    private Ramps ramps;
    private LoopStopwatch loopTimer;
    private Speed2Power speed2Power;
    private org.firstinspires.ftc.teamcode.hardware.VLiftProxy vLiftProxy;
    private double heading = 0.0;

    /**
     * Forces any tasks using this lock to be stopped immediately.
     *
     * @param theLockInQuestion which lock to forcibly acquire
     */
    private void abandonLock(SharedResource theLockInQuestion) {
        scheduler.filteredStop(it -> it.requirements().contains(theLockInQuestion), true, true);
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

    private MoveRelTask moveRel(Pose offset) {
        return new MoveRelTask(
                scheduler, hardware, offset, tracker, loopTimer, speed2Power, ramps, telemetry
        );
    }

    private void hardwareInit() {
        tracker = new EncoderTracking(hardware);
        tracker.setOrientationProvider(() -> heading);
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

        navxMicro = hardware.gyro;
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
        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, hardware.verticalSlide));
        hSlideProxy = scheduler.add(new HSlideProxy(scheduler, hardware));
        hClawProxy = scheduler.add(new HClawProxy(scheduler, hardware));

        telemetry.log().clear();
        telemetry.log().add("Set and ready to roll!");
        telemetry.clear();

        waitForStart();
        if (isStopRequested()) return;

        boolean isFlipIn = false;
        boolean isFlipOut = false;
        boolean isSpecimenPick = false;
        boolean isScoreHigh = false;
        boolean isScoreHigh2 = false;
        boolean isScoreSpecimen = false;
        boolean isTx = false;
        boolean isTxDump = false;
        boolean isClearArmPos = false;
        boolean isResetVL = false;

        double yaw_offset = 0.0;
        while (opModeIsActive()) {

            Orientation angles = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (gamepad1.back) {
                yaw_offset = angles.firstAngle;
            }
            double botheading = angles.firstAngle - yaw_offset;
            telemetry.addData("Heading", formatAngle(angles.angleUnit, botheading));
            heading = botheading;
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
            lamps();
            wrist();
            claw();
            stepper();
            lift();
            arm();

            boolean shouldScoreHigh = gamepad2.left_trigger > 0.5;
            boolean shouldScoreHigh2 = gamepad2.right_trigger > 0.5;
            if (shouldScoreHigh && !isScoreHigh) {
                ScoreHighBasket1();
            }
            if (shouldScoreHigh2 && !isScoreHigh2) {
                ScoreHighBasket2();
            }
            boolean shouldSpecimenPick = gamepad2.b;
            if (shouldSpecimenPick && !isSpecimenPick) {
                specimenWallPick();
            }
            boolean shouldScoreSpecimen = gamepad2.dpad_left;
            if (shouldScoreSpecimen && !isScoreSpecimen) {
                score();
            }
            boolean shouldTx = gamepad2.x;
            if (shouldTx && !isTx) {
                transfer();
            }
            boolean shouldTxDump = gamepad1.x;
            if (shouldTxDump && !isTxDump) {
                transferAndDrop();
            }
            boolean shouldClearArmPos = gamepad2.back;
            if (shouldClearArmPos && !isClearArmPos) {
                DcMotor.RunMode mode = hardware.arm.getMode();
                hardware.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hardware.arm.setTargetPosition(0);
                hardware.arm.setMode(mode);
            }

            boolean shouldFlipIn = gamepad1.right_trigger > 0.5;
            if (shouldFlipIn && !isFlipIn) Flipin();
            boolean shouldFlipOut = gamepad1.left_trigger > 0.5;
            if (shouldFlipOut && !isFlipOut) Flipout();

            boolean shouldResetVL = gamepad2.a;
            if (shouldResetVL && !isResetVL) {
                resetAll();
            }

            isSpecimenPick = shouldSpecimenPick;
            isFlipIn = shouldFlipIn;
            isFlipOut = shouldFlipOut;
            isScoreHigh = shouldScoreHigh;
            isScoreHigh2 = shouldScoreHigh2;
            isScoreSpecimen = shouldScoreSpecimen;
            isTx = shouldTx;
            isTxDump = shouldTxDump;
            isResetVL = shouldResetVL;

            int verticalPosition = hardware.encoderVerticalSlide.getCurrentPosition();

            scheduler.tick();
            tracker.step();
            telemetry.addData("Wrist Position", hardware.wrist.getPosition());
            telemetry.addData("Claw Position", hardware.claw.getPosition());
            telemetry.addData("Vertical position", verticalPosition);
            telemetry.addData("Est pose", tracker.getPose());
            scheduler.displayStatus(false, true, (str) -> {
                telemetry.addLine(str);
                return Unit.INSTANCE;
            });
            telemetry.update();
        }

    }

    /////////////////////////////////////////////

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    @SuppressLint("DefaultLocale")
    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // lifts the vertical slides to a target position in ticks

    private ITaskWithResult<Boolean> targetLift(int targetPosition) {
        abandonLock(vLiftProxy.CONTROL);
        return scheduler.add(vLiftProxy.moveTo(targetPosition, 5, 3.0));
    }

    private void lamps() {
//        @ColorInt int color = hardware.clawColor.argb();
        // extract the components from the packed number
//        int red = (color >> 16) & 0xff;
//        int green = (color >> 8) & 0xff;
//        int blue = color & 0xff;
        int red = hardware.clawColor.red();
        int green = hardware.clawColor.green();
        int blue = hardware.clawColor.blue();

        if (blue - green > 100 && blue - red > 100) {
            hardware.lightRight.setPosition(Hardware.LAMP_BLUE);
            hardware.lightLeft.setPosition(Hardware.LAMP_BLUE);
//            telemetry.addLine("blue");
        } else if (red - blue > 100 && red - green > 100) {
//            telemetry.addLine("red");
            hardware.lightRight.setPosition(Hardware.LAMP_RED);
            hardware.lightLeft.setPosition(Hardware.LAMP_RED);
        } else if (green - blue > 100 && green - red > 100 && red >= 350) {
//            telemetry.addLine("yellow");
            hardware.lightRight.setPosition(Hardware.LAMP_YELLOW);
            hardware.lightLeft.setPosition(Hardware.LAMP_YELLOW);
        } else {
            hardware.lightRight.setPosition(0);
            hardware.lightLeft.setPosition(0);
        }
    }

    private void lift() {
        vLiftProxy.controlManual(gamepad2.dpad_up, gamepad2.dpad_down);
    }

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
        double tRotations = arm.getTargetPosition() / Hardware.spinTickPerRev;
        double tDegrees = tRotations * 360.0; // 0 = straight down
        // Negative: towards front;
        // Positive: towards back.
        // Exclusion zone 0 to -25deg whe lift < 6in.
        // [removed if statement, disabled]
        // Full* clearance
        boolean shouldWrite = true;
        if (stick_pos > 0.7 && tDegrees <= 110) {
            tDegrees += 1;
        } else if (stick_pos < -0.7 && tDegrees >= -110) {
            tDegrees -= 1;
        } else {
            shouldWrite = false;
        }
        if (shouldWrite) {
            abandonLock(Locks.ArmAssembly);
            arm.setTargetPosition(Hardware.deg2arm(tDegrees));
        }
        telemetry.addData("arm deg", degrees);
        telemetry.addData("arm target deg", tDegrees);
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
        if (gamepad2.left_bumper) {
            abandonLock(Locks.ArmAssembly);
            hardware.claw.setPosition(Hardware.CLAW_OPEN);
        } else if (gamepad2.right_bumper) {
            abandonLock(Locks.ArmAssembly);
            hardware.claw.setPosition(Hardware.CLAW_CLOSE);
        }
    }

    public void resetAll() {
        abandonLock(Locks.ArmAssembly);
        abandonLock(vLiftProxy.CONTROL);
        scheduler.add(groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                .then(await(200))
                .then(run(() -> {
                    hardware.arm.setTargetPosition(0);
                    hardware.wrist.setPosition(Hardware.WRIST_UP);
                }))
                .then(await(200))
                .then(vLiftProxy.moveTo(0, 5, 1.0))
        ).extraDepends(Locks.ArmAssembly, vLiftProxy.CONTROL));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    public void ScoreHighBasket1() {
        abandonLock(Locks.ArmAssembly);
        abandonLock(vLiftProxy.CONTROL);
        scheduler.add(
                groupOf(inner -> inner.add(vLiftProxy.moveTo(highBasketTicks, 5, 2.0))
                                .then(run(() -> hardware.arm.setTargetPosition(222)))
//                        .then(run(() -> hardware.claw.setPosition(0.02)))
//                        .then(await(500))
                        // broken into two here
                ).extraDepends(
                        Locks.ArmAssembly,
                        vLiftProxy.CONTROL
                )
        );
    }

    public void ScoreHighBasket2() {
        // prevent doing this by accident
        if (hardware.verticalSlide.getCurrentPosition() < 1000) return;
        if (hardware.arm.getCurrentPosition() < 190) return;
        abandonLock(Locks.ArmAssembly);
        abandonLock(vLiftProxy.CONTROL);
        scheduler.add(groupOf(inner -> inner.add(run(() -> hardware.wrist.setPosition(0.94)))
                .then(await(200))
                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                .then(await(100))
                .then(run(() -> hardware.wrist.setPosition(0.28)))
                .then(await(500))
                .then(run(() -> hardware.arm.setTargetPosition(0)))
                .then(await(500))
                .then(vLiftProxy.moveTo(0, 5, 2.0)))
                .extraDepends(
                        Locks.ArmAssembly,
                        vLiftProxy.CONTROL
                ));
    }

    private void stepper() {
        // this deals with the horizontal slide stuff. we don't have locks (yet) so it's remaining
        // as-is for now
        if (gamepad1.dpad_left) {
            abandonLock(hClawProxy.CONTROL_CLAW);
            hClawProxy.setClaw(Hardware.FRONT_OPEN);
        }
        if (gamepad1.dpad_right) {
            abandonLock(hClawProxy.CONTROL_CLAW);
            hClawProxy.setClaw(Hardware.FRONT_CLOSE);
        }
        if (gamepad1.dpad_up) {
            abandonLock(hClawProxy.CONTROL_FLIP);
            hClawProxy.setFlip(hClawProxy.getFlipPosition() + 0.01);
        }
        if (gamepad1.dpad_down) {
            abandonLock(hClawProxy.CONTROL_FLIP);
            hClawProxy.setFlip(hClawProxy.getFlipPosition() - 0.01);
        }
//        hardware.clawFlip.setPosition(ClawFlipPos);
//        hardware.clawFront.setPosition(ClawFrontPos);
        // clawFront close is 0
        //clawFront open is 0.27

        telemetry.addData("FrontClawPos", hClawProxy.getClawPosition());
        telemetry.addData("FlipClawPos", hClawProxy.getFlipPosition());
    }

    public void specimenWallPick() {
        abandonLock(Locks.ArmAssembly);
        abandonLock(vLiftProxy.CONTROL);
        scheduler.add(
                groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                                .then(await(200))
                                .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_UP)))
                                .then(await(200))
                                .then(run(() -> hardware.arm.setTargetPosition(50)))
                                .then(await(500))
                                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                                .then(await(500))
                                .then(vLiftProxy.moveTo(225, 5, 0.4))
                                .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_BACK)))
//                        .then(await(200))
                                // TODO: investigate if 10 ticks or 10 degrees is the right number
                                .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
                                .then(await(200))
                                .then(vLiftProxy.moveTo(0, 5, 0))
                ).extraDepends(
                        vLiftProxy.CONTROL,
                        Locks.ArmAssembly
                )
        );
    }

    private void score() {
        double clawclose = 0.02;
        abandonLock(vLiftProxy.CONTROL);
        abandonLock(Locks.ArmAssembly);
        abandonLock(Locks.DriveMotors);


        // if something else takes the locks between these it's the driver's fault smh
        scheduler.add(
                groupOf(it -> it.add(run(() -> hardware.claw.setPosition(clawclose)))
                        .then(vLiftProxy.moveTo(710, 5, 1.0))
                        .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(-99))))
                        .then(await(1000))
                        .then(run(() -> hardware.wrist.setPosition(1)))
                ).extraDepends(
                        vLiftProxy.CONTROL,
                        Locks.ArmAssembly
                )
        );
    }

    /* // might need for Left Auto
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
        sleep(500);
        hardware.clawFlip.setPosition(flipup);
        sleep(500);
        hardware.horizontalSlide.setPosition(hslidein);
        sleep(500);
    }
    */

    // TODO: Reject while running
    public void transferAndDrop() {
        abandonLock(hClawProxy.CONTROL_CLAW);
        abandonLock(vLiftProxy.CONTROL);
        abandonLock(Locks.ArmAssembly);
        transferInternal()
                .then(groupOf(
                        it -> it.add(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
                                .then(await(100))
                                .then(run(() -> hardware.wrist.setPosition(0.75)))
                                .then(await(100))
                                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                                .then(await(300))
                                .then(run(() -> {
                                    hardware.wrist.setPosition(Hardware.WRIST_BACK);
                                    hardware.arm.setTargetPosition(0);
                                }))
                ).extraDepends(
                        Locks.ArmAssembly
                ));
    }

    public void Flipout() {
        abandonLock(hSlideProxy.CONTROL);
        abandonLock(hClawProxy.CONTROL_CLAW);
        abandonLock(hClawProxy.CONTROL_FLIP);
        scheduler.add(groupOf(
                it -> it.add(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                        .then(hSlideProxy.moveOut())
                        .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
        ).extraDepends(
                hSlideProxy.CONTROL,
                hClawProxy.CONTROL_FLIP,
                hClawProxy.CONTROL_CLAW
        ));
    }

    public void Flipin() {
        abandonLock(hSlideProxy.CONTROL);
        abandonLock(hClawProxy.CONTROL_FLIP);
        double flipThird = 0.66;
        scheduler.add(groupOf(
                it -> it.add(hClawProxy.aSetFlip(flipThird))
                        .then(hSlideProxy.moveIn())
                        .then(hClawProxy.aSetFlip(Hardware.FLIP_UP))
        ).extraDepends(
                hSlideProxy.CONTROL,
                hClawProxy.CONTROL_FLIP
        ));
    }

    private TaskGroup transferInternal() {
        return scheduler.add(groupOf(
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
                        .then(await(300))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(250))
                        .then(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                        .then(await(250))
                        .then(run(() -> {
                            hardware.arm.setTargetPosition(0);
                            hardware.wrist.setPosition(Hardware.WRIST_BACK);
                        }))

        ).extraDepends(
                hClawProxy.CONTROL_CLAW,
                vLiftProxy.CONTROL,
                Locks.ArmAssembly
        ));
    }

    public void transfer() {
        abandonLock(hClawProxy.CONTROL_CLAW);
        abandonLock(vLiftProxy.CONTROL);
        abandonLock(Locks.ArmAssembly);
        transferInternal();
    }

}
