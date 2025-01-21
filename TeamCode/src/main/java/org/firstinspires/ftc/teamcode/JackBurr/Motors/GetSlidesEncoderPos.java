package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class GetSlidesEncoderPos extends OpMode {
    public DcMotor leftSlides;
    public DcMotor rightSlides;
    @Override
    public void init() {
        leftSlides = hardwareMap.get(DcMotor.class, "deliverySlideL");
        rightSlides = hardwareMap.get(DcMotor.class, "deliverySlideR");
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addLine("Left: " + leftSlides.getCurrentPosition());
        telemetry.addLine("Right: " + rightSlides.getCurrentPosition());
    }
}
