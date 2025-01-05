package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class IntakeSlidesBrake2 extends OpMode {
    public DcMotor intakeSlides;
    public double power = 0;

    @Override
    public void init(){
        intakeSlides = hardwareMap.get(DcMotor.class,"intakeSlides");
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void loop() {
        intakeSlides.setTargetPosition(20);
        if (intakeSlides.getCurrentPosition() != 20) {
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
