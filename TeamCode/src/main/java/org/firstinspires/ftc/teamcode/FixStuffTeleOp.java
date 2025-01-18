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

import dev.aether.collaborative_multitasking.ITask;
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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class FixStuffTeleOp extends LinearOpMode {

    private Hardware hardware;
    private MultitaskScheduler scheduler;
    private VLiftProxy vLiftProxy;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;

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

    private void hardwareInit() {

        hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.clawFlip.setPosition(Hardware.FLIP_UP);
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);

        hardware.arm.setTargetPosition(0);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.3);
        hardware.wrist.setPosition(Hardware.WRIST_BACK);
        hardware.claw.setPosition(Hardware.CLAW_CLOSE);
        hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
        // we don't have the proxy object to handle this for us
        // so manually implement the inversion
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);

        hardware.lightLeft.setPosition(Hardware.LAMP_PURPLE);
        hardware.lightRight.setPosition(Hardware.LAMP_PURPLE);

    }

    public void runOpMode() {
        scheduler = new MultitaskScheduler();
        hardware = new Hardware(hardwareMap);
        hardwareInit();

        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, hardware.verticalLift));
        hSlideProxy = scheduler.add(new HSlideProxy(scheduler, hardware));
        hClawProxy = scheduler.add(new HClawProxy(scheduler, hardware));

        waitForStart();
        if (isStopRequested()) return;

        boolean wasX = false;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                SlideOut();
            }
            if (gamepad1.b) {
                SlideIn();
            }
            boolean isX = gamepad1.x;
            if (isX && !wasX) {
                scheduler.add(GetSpec());
            }
            wasX = isX;

            if (gamepad1.y) {
                transfer();
            }

            telemetry.addData("slidePos", hardware.horizontalLeft.getPosition());
            telemetry.addData("slidePos2", hardware.horizontalSlide.getPosition());
            telemetry.update();
            scheduler.tick();
        }
    }

    public ITask GetSpec() {
        return groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(await(200))
                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_UP)))
                        .then(await(200))
                        .then(run(() -> hardware.arm.setTargetPosition(65)))
                        .then(await(500))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(200))
                        .then(vLiftProxy.moveTo(50, 3, 0.4))
                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_BACK)))
//                        .then(await(200))
                        .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
                        .then(await(200))
                        .then(vLiftProxy.moveTo(0, 5, 0))
        );
    }

    public void SlideOut() {
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
        sleep(500);
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_OUT);
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_DOWN);

    }

    public void SlideIn() {
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_ONE_THIRD);
        sleep(500);
        hardware.horizontalSlide.setPosition(Hardware.SLIDE_OVERSHOOT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.SLIDE_OVERSHOOT);
        sleep(500);
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);
        hardware.clawFlip.setPosition(Hardware.FLIP_UP);
    }

    public void FourthSample() {
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
        sleep(500);
        double PartialFlip = 0.167;
        hardware.clawFlip.setPosition(PartialFlip);
        sleep(500);
        hardware.clawTwist.setPosition(0.26);
        sleep(500);
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_OUT);
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_DOWN);
        sleep(500);
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
        sleep(500);
        hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
        sleep(500);
        hardware.horizontalSlide.setPosition(Hardware.SLIDE_OVERSHOOT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.SLIDE_OVERSHOOT);
        sleep(500);
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_UP);
    }

    public void transfer() {
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
        hardware.claw.setPosition(Hardware.CLAW_OPEN);
        hardware.arm.setTargetPosition(Hardware.ARM_TRANSFER_POS);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.5);
        hardware.wrist.setPosition(0);
        sleep(500);
        hardware.claw.setPosition(Hardware.CLAW_CLOSE);
        sleep(500);
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
        sleep(500);

        hardware.arm.setTargetPosition(0);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.5);
        sleep(500);
        hardware.wrist.setPosition(Hardware.WRIST_BACK);
        sleep(500);
    }
}
