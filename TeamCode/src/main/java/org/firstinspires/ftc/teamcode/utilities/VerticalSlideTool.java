package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.hardware.VLiftProxy;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveToTask;

import java.util.function.Consumer;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.TaskGroup;
import dev.aether.collaborative_multitasking.TaskTemplate;
import dev.aether.collaborative_multitasking.ext.Pause;
import dev.aether.collaborative_multitasking.ext.While;
import kotlin.Unit;

@TeleOp(name = "util: Vertical Slide", group = "Utilities")
public class VerticalSlideTool extends LinearOpMode {

    MultitaskScheduler scheduler;

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
    private VLiftProxy vLiftProxy;
    private Hardware h;

    private ITask scoreHighBasket() {
        return groupOf(inner -> inner.add(groupOf(a -> {
                            a.add(vLiftProxy.moveTo(Hardware.VLIFT_SCORE_HIGH, 5, 2.0));
                            a.add(run(() -> h.arm.setTargetPosition(252)));
                        }))
                        .then(run(() -> h.wrist.setPosition(0.94)))
                        .then(await(200))
                        .then(run(() -> h.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(await(100))
                        .then(run(() -> h.wrist.setPosition(0.28)))
                        .then(run(() -> h.arm.setTargetPosition(0)))
                        .then(vLiftProxy.moveTo(0, 5, 2.0))
        );
    }
    private void hardwareInit() {
        h.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        h.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        h.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        h.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        h.clawFlip.setPosition(Hardware.FLIP_UP);
        h.clawFront.setPosition(Hardware.FRONT_OPEN);

        h.arm.setTargetPosition(0);
        h.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.arm.setPower(0.3);
        h.wrist.setPosition(0.28);
        h.claw.setPosition(Hardware.CLAW_CLOSE);

        // we don't have the proxy object to handle this for us
        // so manually implement the inversion
        h.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_IN);
        h.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);

        h.lightLeft.setPosition(Hardware.LAMP_PURPLE);
        h.lightRight.setPosition(Hardware.LAMP_PURPLE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        scheduler = new MultitaskScheduler();
        h = new Hardware(hardwareMap);
        h.verticalLift.disable();
//        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, h.verticalLift));
//        hardwareInit();
        waitForStart();
//        scheduler.add(scoreHighBasket());
//        h.verticalLift.setTargetPosition(500);
        while (opModeIsActive()) {
            telemetry.addData("position", h.verticalLift.getCurrentPosition());
//            telemetry.addData("directed power", h.verticalLift.getPower());
//            telemetry.addData("target", h.verticalLift.getTargetPosition());
//            telemetry.addData("eTotal", h.verticalLift.getETotal());
            telemetry.addLine();
//            if (gamepad1.y) h.verticalLift.setTargetPosition(Hardware.VLIFT_SCORE_HIGH);
//            else if (gamepad1.b) h.verticalLift.setTargetPosition(300);
//            else if (gamepad1.a) h.verticalLift.setTargetPosition(0);
//            else h.verticalLift.setPower(0);
            scheduler.tick();
            scheduler.displayStatus(true, true,
                    str -> {
                        telemetry.addLine(str);
                        return Unit.INSTANCE;
                    });
            telemetry.update();
        }
    }
}
