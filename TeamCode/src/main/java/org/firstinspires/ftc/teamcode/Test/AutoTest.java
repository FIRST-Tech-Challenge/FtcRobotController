package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

import java.util.Arrays;

@Autonomous(name="AutoTest", group="Test")
public class AutoTest extends LinearOpMode {
    ViperSlide viperSlide = new ViperSlide(this);

    @Override
    public void runOpMode() {
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Initialized", "");
            telemetry.update();
        }

        waitForStart();

//        Action

        Actions.runBlocking(
                new SequentialAction(
                        viperSlide.goToPositionAction(1000)

                )
        );

        if (isStopRequested() || gamepad1.b) return;
    }
}
