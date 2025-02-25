package org.firstinspires.ftc.teamcode.drive.teleop;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Outtake extends OpMode {
    DcMotor polia;
    Servo turn;
    Servo garrinha;
    double ticks = 2800.5;
    double ticks2 = 1000;
    double ticks3 = 2000;
    double newTarget;

    public void init(){
        polia = hardwareMap.get(DcMotor.class, "polia");
        turn = hardwareMap.get(Servo.class, "turn");
        garrinha = hardwareMap.get(Servo.class, "garrinha");

        polia.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polia.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop(){
        if (gamepad1.dpad_up) {
            newTarget = ticks / 1;
            polia.setTargetPosition((int) newTarget);
            polia.setPower(1);
            polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(200);
            turn.setPosition(0.85);
        }
        if (gamepad1.dpad_left){
            newTarget = ticks2 / 1;
            polia.setTargetPosition((int) newTarget);
            polia.setPower(1);
            polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.dpad_down){
            turn.setPosition(0.3);
            polia.setTargetPosition(0);
            polia.setPower(1);
            polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(200);
            garrinha.setPosition(0.6);
            sleep(2000);
            polia.setPower(0);
        }
        if (gamepad1.dpad_right){
            newTarget = ticks3 / 1;
            polia.setTargetPosition((int) newTarget);
            polia.setPower(1);
            polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.b){
            garrinha.setPosition(1); // fecha
        }
        if (gamepad1.a) {
            garrinha.setPosition(0.6); // abre
        }
        if (gamepad1.x){
            turn.setPosition(0.25); // volta o bracinho
        }
        if (gamepad1.y){
            turn.setPosition(1);
        }
    }
    public void poliaBasket(int turnage) {
        newTarget = ticks / turnage;
        polia.setTargetPosition((int) newTarget);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(200);
        turn.setPosition(0.85);
    }
    public void poliaClip(int turnage) {
        newTarget = ticks2 / turnage;
        polia.setTargetPosition((int) newTarget);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void poliaClipar(int turnage){
        newTarget = ticks3 / turnage;
        polia.setTargetPosition((int) newTarget);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void poliaDown() {
        polia.setTargetPosition(0);
        polia.setPower(1);
        polia.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(200);
        garrinha.setPosition(0.6);
        sleep(2000);
        polia.setPower(0);
    }
}