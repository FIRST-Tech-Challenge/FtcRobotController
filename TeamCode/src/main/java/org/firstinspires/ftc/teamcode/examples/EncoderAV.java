package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Encoder AV Test ")
public class EncoderAV extends OpMode {
    DcMotor armmotor;
    double ticks = 2786.2;
    double newTarget;
    @Override
    public void init() {
        armmotor = hardwareMap.get(DcMotor.class, "motor");
        telemetry.addData("Hardware: ", "Initialized");
        armmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            encoder(2);
        }
        telemetry.addData("Motor Ticks: ", armmotor.getCurrentPosition());
        if(gamepad1.b){
            tracker();
        }

    }
    public void encoder(int turnage){
        newTarget = ticks/turnage;
        armmotor.setTargetPosition((int)newTarget);
        armmotor.setPower(0.3);
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void tracker(){
        armmotor.setTargetPosition(0);
        armmotor.setPower(0.8);
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}