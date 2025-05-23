package org.firstinspires.ftc.teamcode.drive.actuators;

import static com.qualcomm.robotcore.hardware.Servo.MIN_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ViperSlide extends OpMode{
    DcMotor poliaright;
    DcMotor polialeft;

    @Override
    public void init(){
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        telemetry.addData("Hardware: ", "Initialized");
        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        polialeft.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.update();
    }

    @Override
    public void loop(){
        telemetry.addData("Posição da Polia: ", poliaright.getCurrentPosition());
        double potencia = gamepad1.left_stick_y;
        polialeft.setPower(potencia);
        poliaright.setPower(potencia);
        }
    }