package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
public class TestingTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad DriverTwo = gamepad2;
        Gamepad DriverOne = gamepad1;

        waitForStart();
        Slides slides = new Slides(this);

        while (opModeIsActive()) {
            slides.teleOp(gamepad1);
        }
    }
}
