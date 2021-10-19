package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.teamcode.tools.*;
@TeleOp
public class ToolTestCode extends LinearOpMode {
    @Override
    public void runOpMode() {
        hardwareMap.get(Blinker.class, "Control Hub");
        hardwareMap.get(Blinker.class, "Expansion Hub 2");
        final GamepadEx controller = new GamepadEx(gamepad1);
        final Lift lift = new Lift(hardwareMap, telemetry, controller);
        final Carousel carousel = new Carousel(hardwareMap,controller);
        final Intake intake = new Intake(hardwareMap,controller);
        waitForStart();
        while (opModeIsActive()) {
            lift.update();
            carousel.update();
            intake.update();
            telemetry.update();
        }
    }
}
