package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Autonomous Comp", group = "Autonomous")
@Disabled
public class AutonomousComp extends LinearOpMode {
    // Declaring motors and servos
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorDuck;
    DcMotor motorArm;
    Servo servoGrabber;
    Servo servoArm;
    double x = 0.7;

    //Other Devices
    BNO055IMU imu;
    @Override
    public void runOpMode() {
        //Initialize the motors and servos
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorDuck = hardwareMap.dcMotor.get("motorDuck");
        servoGrabber = hardwareMap.servo.get("servoGrabber");
        servoArm = hardwareMap.servo.get("servoArm");

        //Set direction of the motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDuck.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set position of servos
        servoGrabber.setPosition(.34);
        servoArm.setPosition(0.00000000000);
        telemetry.addData("Auto version: ", "yes");
        telemetry.update();

        //Declare variables
        int position = 0;
        boolean isPressed = false;
        double motorPower = 0.9;
        double increase = 1;
        double speed =1;

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

        String autoAlience = "Blue";
        String autoState = "Blue";

        /*while(!gamepad1.a && !gamepad2.a){

            if(gamepad1.right_trigger >= 0.5){
                if(autoAlience.equals("Blue")){
                    autoAlience = "Blue";
                } else{
                    autoAlience = "Red";
                }
            }
            if(gamepad1.right_trigger >= 0.5){
                if(autoAlience.equals("Blue")){
                    autoAlience = "Blue";
                } else{
                    autoAlience = "Red";
                }
            }
            telemetry.addData("Auto version: ", autoAlience);
            telemetry.update();
        }*/

        waitForStart();
    }

}