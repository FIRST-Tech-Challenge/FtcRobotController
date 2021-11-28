package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueBlockPlace", group = "Autonomous")
public class BlueParkBlockPLace extends MasterOpMode{
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorDuck;
    DcMotor motorArm;
    Servo servoGrabber;
    Servo servoArm;
    @Override
    public void runOpMode() {
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorDuck = hardwareMap.dcMotor.get("motorDuck");
        servoGrabber = hardwareMap.servo.get("servoGrabber");
        servoArm = hardwareMap.servo.get("servoArm");
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set run mode of arm motor (encoders --> run to position)
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoGrabber.setPosition(0.34);
        pauseMillis(500);
        servoArm.setPosition(0.01);
        waitForStart();
        servoArm.setPosition(1);
        motorArm.setTargetPosition(-720);
        motorArm.setPower(0.9);
        pauseMillis(500);
        Forward(24,0.6);
        pauseMillis(1000);
        TurnAngle(50);
        pauseMillis(1000);
        Forward(10, 0.5);
        pauseMillis(1000);
        servoGrabber.setPosition(0.7);
        pauseMillis(700);
        Forward(-10 , 0.3);
        pauseMillis(500);
        TurnAngle(-142);
        pauseMillis(1500);
        Forward(40,0.8);
        pauseMillis(1500);
        servoGrabber.setPosition(0.34);
        pauseMillis(100);
        servoArm.setPosition(0.1);
        motorArm.setTargetPosition(10);
        motorArm.setPower(0.9);
        pauseMillis(700);
    }
}