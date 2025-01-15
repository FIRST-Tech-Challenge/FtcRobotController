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

public class FixStuffTeleOp extends LinearOpMode{

    private Hardware hardware;


    private void hardwareInit() {

        hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void runOpMode() {

        hardware =new Hardware(hardwareMap);
        hardwareInit();


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a){
                SlideOut();
            }
            if (gamepad1.b){
                SlideIn();
            }

            telemetry.addData("slidePos", hardware.horizontalLeft.getPosition());
            telemetry.addData("slidePos2", hardware.horizontalSlide.getPosition());
            telemetry.update();

        }
    }
    public void SlideOut() {
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(1-Hardware.RIGHT_SLIDE_OUT);
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_DOWN);

    }
    public void SlideIn() {
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_ONE_THIRD);
        sleep(500);
        hardware.horizontalSlide.setPosition(Hardware.SLIDE_OVERSHOOT);
        hardware.horizontalLeft.setPosition(1 - Hardware.SLIDE_OVERSHOOT);
        sleep(500);
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1-Hardware.RIGHT_SLIDE_IN);
        hardware.clawFlip.setPosition(Hardware.FLIP_UP);
    }
}
