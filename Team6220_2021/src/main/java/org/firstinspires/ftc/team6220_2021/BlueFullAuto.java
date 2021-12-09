package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueFullAuto", group = "Autonomous")
public class BlueFullAuto extends MasterOpMode{
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorLeftDuck;
    DcMotor motorArm;
    Servo servoGrabber;
    Servo servoArm;
    int Detection = 3;
    int ArmPosition;
    int DriveAdjust;
    double ServoPosition;
    @Override
    public void runOpMode() {
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorLeftDuck = hardwareMap.dcMotor.get("motorLeftDuck");
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
        motorLeftDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set run mode of arm motor (encoders --> run to position)
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoGrabber.setPosition(0.0);
        pauseMillis(500);
        servoArm.setPosition(0.81);
        waitForStart();
        if (Detection == 1){
            ArmPosition = -220;
            ServoPosition = 0.6;
            DriveAdjust = 23;
        } else if (Detection == 2){
            ArmPosition = -470;
            ServoPosition = 0.8;
            DriveAdjust = 21;
        } else if (Detection == 3){
            ArmPosition = -720;
            ServoPosition = 1;
            DriveAdjust = 25;
        }
        Forward(10,0.3);
        TurnAngle(90);
        Forward(15,0.5);
        stopbase();
        BlueDuck();
        pauseMillis(2000);
        Forward(-5,0.5);
        TurnAngle(-125);
        servoArm.setPosition(ServoPosition);
        motorArm.setTargetPosition(ArmPosition);
        motorArm.setPower(0.9);
        pauseMillis(500);
        Forward(DriveAdjust,0.5);
        stopbase();
        servoGrabber.setPosition(0.7);
        pauseMillis(750);
        Forward(-20,0.5);
        TurnAngle(130);
        stopbase();
        servoGrabber.setPosition(0.34);
        pauseMillis(500);
        servoArm.setPosition(0.01);
        motorArm.setTargetPosition(-220);
        motorArm.setPower(0.9);
        pauseMillis(500);
        Forward(-84, 0.8);
        motorArm.setTargetPosition(10);
        motorArm.setPower(0.9);
        pauseMillis(500);
    }
}