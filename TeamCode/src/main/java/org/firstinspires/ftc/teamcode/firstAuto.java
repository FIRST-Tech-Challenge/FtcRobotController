package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Intake.Claw;

@TeleOp(name = "firstAuto", group = "Examples")
public class firstAuto extends LinearOpMode {
    private Claw claw;
    private double openPos = 0.232;
    private double closePos = 0.055;

    @Override
    public void runOpMode() {
        claw = new Claw(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            claw.openOrCloseClaw(gamepad1.a, gamepad1.y, closePos, openPos);

            sleep(200);
        }
    }
}