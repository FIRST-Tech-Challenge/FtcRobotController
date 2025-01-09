package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class IntakeSlidesBrake extends OpMode {
    public DcMotor intakeSlides;
    public double power = 0;

    @Override
    public void init(){
        intakeSlides = hardwareMap.get(DcMotor.class,"intakeSlides");
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    @Override
    public void loop() {
        intakeSlides.setPower(1);
        intakeSlides.setTargetPosition(75);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Motor Power: ", intakeSlides.getPower());
        telemetry.addData("Encoder Position: ", intakeSlides.getCurrentPosition());
        telemetry.addData("ZeroPowerBehavior: ", intakeSlides.getZeroPowerBehavior());
        telemetry.update();
    }
}
