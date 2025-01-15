package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DeliverySlidesController extends OpMode {
    public int TARGET = -10;
    public DcMotor deliverySlideL;
    public DcMotor deliverySlideR;
    public ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        deliverySlideL = hardwareMap.get(DcMotor.class,"deliverySlideL");
        deliverySlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deliverySlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deliverySlideR = hardwareMap.get(DcMotor.class,"deliverySlideR");
        deliverySlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deliverySlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper && timer.seconds() > 0.3){
            TARGET = TARGET + 20;
            timer.reset();
        }
        else if(gamepad1.left_bumper && timer.seconds() > 0.3){
            TARGET = TARGET - 20;
            timer.reset();
        }
        deliverySlideL.setTargetPosition(-TARGET);
        if (deliverySlideL.getCurrentPosition() != -TARGET) {
            deliverySlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliverySlideL.setPower(-1);
        }
        else {
            deliverySlideL.setPower(0);
        }
        deliverySlideR.setTargetPosition(TARGET);
        if (deliverySlideR.getCurrentPosition() != TARGET) {
            deliverySlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliverySlideR.setPower(-1);
        }
        else {
            deliverySlideR.setPower(0);
        }
        telemetry.addData("Left Motor Power: ", deliverySlideL.getPower());
        telemetry.addData("Left Encoder Positin: ", deliverySlideL.getCurrentPosition());
        telemetry.addData("Left Target Encoder Position: ", deliverySlideL.getTargetPosition());
        telemetry.addData("Left ZeroPowerBehavior: ", deliverySlideL.getZeroPowerBehavior());
        telemetry.addData("Left Is Busy: ", deliverySlideL.isBusy());
        telemetry.addData("Right Motor Power: ", deliverySlideR.getPower());
        telemetry.addData("Right Encoder Position: ", deliverySlideR.getCurrentPosition());
        telemetry.addData("Left Target Encoder Position: ", deliverySlideR.getTargetPosition());
        telemetry.addData("Right ZeroPowerBehavior: ", deliverySlideR.getZeroPowerBehavior());
        telemetry.addData("Right Is Busy: ", deliverySlideR.isBusy());
        telemetry.update();
    }
}
