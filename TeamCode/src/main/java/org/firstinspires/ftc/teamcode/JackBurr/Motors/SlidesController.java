package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp
public class SlidesController extends OpMode {
    public int TARGET = 10;
    public DcMotor intakeSlides;
    public ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        intakeSlides = hardwareMap.get(DcMotor.class,"intakeSlides");
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        intakeSlides.setTargetPosition(TARGET);
        if (intakeSlides.getCurrentPosition() != TARGET) {
            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeSlides.setPower(1);
        }
        else {
            intakeSlides.setPower(0);
        }
        telemetry.addData("Motor Power: ", intakeSlides.getPower());
        telemetry.addData("Encoder Position: ", intakeSlides.getCurrentPosition());
        telemetry.addData("ZeroPowerBehavior: ", intakeSlides.getZeroPowerBehavior());
        telemetry.addData("Is Busy: ", intakeSlides.isBusy());
        telemetry.update();
    }
}
