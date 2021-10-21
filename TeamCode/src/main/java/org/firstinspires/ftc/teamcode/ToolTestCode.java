package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.*;

import java.util.concurrent.TimeUnit;

@TeleOp
public class ToolTestCode extends LinearOpMode {
    @Override
    public void runOpMode() {
        hardwareMap.get(Blinker.class, "Control Hub");
        hardwareMap.get(Blinker.class, "Expansion Hub 2");
        final GamepadEx controller = new GamepadEx(gamepad1);
        final Lift lift = new Lift(hardwareMap, controller, telemetry);
        final Carousel carousel = new Carousel(hardwareMap,controller);
        final Intake intake = new Intake(hardwareMap,controller);
        final ElapsedTime clocktimer = new ElapsedTime();
        int clocks = 0;
        int clockOutput = 0;
        waitForStart();
        while (opModeIsActive()) {
            if (clocktimer.time(TimeUnit.SECONDS) <= 1) { clocks++; }
            else {
                clockOutput = clocks;
                clocks = 0;
                clocktimer.reset();
            }
            telemetry.addData("ClocksPerSecond", clockOutput);
            lift.update();
            carousel.update();
            intake.update();
            telemetry.update();
        }
    }
}
