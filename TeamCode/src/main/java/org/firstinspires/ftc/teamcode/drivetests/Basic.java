package org.firstinspires.ftc.teamcode.drivetests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Basic", group="Robot")
public class Basic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor eleL = hardwareMap.get(DcMotor.class, "eleL");
        DcMotor eleR = hardwareMap.get(DcMotor.class, "eleR");

        waitForStart();

        while (opModeIsActive()) {
            eleL.setPower(1);
        }
    }
}
