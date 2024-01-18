package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class intake extends LinearOpMode {
    private DcMotor intake, lSlide, rSlide;
    private Servo V4BL, V4BR;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.dcMotor.get("i");
        lSlide = hardwareMap.dcMotor.get("ls");
        rSlide = hardwareMap.dcMotor.get("rs");
        V4BL = hardwareMap.servo.get("V4BL");
        V4BR = hardwareMap.servo.get("V4BR");

        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double slides = gamepad1.left_stick_y;
            lSlide.setPower(-slides);
            rSlide.setPower(slides);

            double power = gamepad1.left_trigger - gamepad1.right_trigger;
            intake.setPower(power);

            telemetry.addLine("V4BR: " + V4BR.getPosition());
            telemetry.addLine("V4BL: " + V4BL.getPosition());
            telemetry.addLine("SlidePos: " + rSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}
