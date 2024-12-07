package org.firstinspires.ftc.teamcode.Opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
@Disabled

public class Intaketest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        waitForStart();
        while (opModeIsActive()) {
            intakeMotor.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}
