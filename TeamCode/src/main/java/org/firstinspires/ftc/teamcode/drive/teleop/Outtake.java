package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
public class Outtake extends OpMode {
    DcMotor poliaright;
    DcMotor polialeft;
    Servo Bright;
    Servo Bleft;
    Servo garrinha;
    double ticks = 2800.5;
    double newTarget;
    private static int MIN_POSITION = 0;

    @Override
    public void init(){
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        telemetry.addData("Hardware: ", "Initialized");
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Bright = hardwareMap.get(Servo.class,"bright");
        Bleft = hardwareMap.get(Servo.class,"bleft");
        garrinha = hardwareMap.get(Servo.class,"garrinha");
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            viperslide1Up(-1);
            viperslide2Up(1);
        }
        if (gamepad1.dpad_left){
            viperslide1Up(-3);
            viperslide2Up(3);
        }
        if (gamepad1.dpad_right){
            viperslide1Down();
            viperslide2Down();
        }
        int currentPosition1 = poliaright.getCurrentPosition();
        int currentPosition2 = polialeft.getCurrentPosition();
        // Se o motor atingir a posição mínima, ele é desligado
        if (currentPosition1 <= MIN_POSITION) {
            poliaright.setPower(0);
        }
        if (currentPosition2 <= MIN_POSITION){
            polialeft.setPower(0);
        }
        if (gamepad1.a) {
            Bright.setPosition(1);
            Bleft.setPosition(0);
        }
        if (gamepad1.b){
            Bright.setPosition(0.2);
            Bleft.setPosition(0.8);
        }
        if (gamepad1.x){
            garrinha.setPosition(0.4);
        }
        if (gamepad1.y){
            garrinha.setPosition(0);
        }

    }
    public void viperslide1Up(int turnage) {
        newTarget = ticks / turnage;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Up(int turnage) {
        newTarget = ticks / turnage;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide1Down() {
        poliaright.setTargetPosition(0);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Down() {
        polialeft.setTargetPosition(0);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
