package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class IntakeMotorTest extends OpMode {
    public DcMotor intakeSlides;
    @Override
    public void init(){
        this.intakeSlides = hardwareMap.get(DcMotor.class,"intakeSlides");
        intakeSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlides.setPower(0);
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runToPosition(int position, double power){
        intakeSlides.setPower(power);
        intakeSlides.setTargetPosition(position);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        telemetry.addLine(String.valueOf(intakeSlides.getCurrentPosition()));
    }
}