package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@TeleOp
public class TestingTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad DriverTwo = gamepad2;
        Gamepad DriverOne = gamepad1;

        waitForStart();
        Claw claw = new Claw(this);
        Wrist wrist = new Wrist(this);

        while (opModeIsActive()) {
            claw.teleOp();
            wrist.teleOp();

        }
    }
}
