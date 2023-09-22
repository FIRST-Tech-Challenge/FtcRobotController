package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class XDrive extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor northWheel;
    private DcMotor southWheel;
    private DcMotor westWheel;
    private DcMotor eastWheel;
    private DcMotor neck;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo leftGrip;
    private Servo rightGrip;

    @Override
    public void RunOpMode(){

        imu = hardwareMap.get(Gyroscope.class, "imu");
        northWheel = hardwareMap.get(DcMotor.class, "northWheel");
        southWheel = hardwareMap.get(DcMotor.class, "southWheel");
        westWheel = hardwareMap.get(DcMotor.class, "westWheel");
        eastWheel = hardwareMap.get(DcMotor.class, "eastWheel");
        neck = hardwareMap.get(DcMotor.class, "neck");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        //variables for drivetrain
        double forwardPower;
        double strafePower;

        //variables for arm
        double neckInput;
        double shoulderInput;

        //initializes positions for claw grips
        leftGrip.setPosition(0);
        rightGrip.setPosition(0);

        //set the directions of motors
        northWheel.setDirection(DcMotor.Direction.FORWARD);
        southWheel.setDirection(DcMotor.Direction.REVERSE);
        westWheel.setDirection(DcMotor.Direction.FORWARD);
        eastWheel.setDirection(DcMotor.Direction.REVERSE);

        while(opModeIsActive()){

            //Sets variables for driving on gamepad1
            forwardPower=gamepad1.left_stick_y;
            strafePower=gamepad1.right_stick_x;

            //Sets power of wheels based on double variables
            northWheel.setPower(strafePower);
            southWheel.setPower(strafePower);
            westWheel.setPower(forwardPower);
            eastWheel.setPower(forwardPower);

            //Sets variables for inputs received from gamepad2
            neckInput=gamepad2.left_stick_x;
            shoulderInput=gamepad2.right_stick_y;

            //Sets power of the neck and shoulder based on double variables
            neck.setPower(neckInput);
            shoulder.setPower(shoulderInput);

            //changes the current wrist position based on inputs received from the dpad on gamepad2
            while(gamepad2.dpad_up){
                wrist.setPosition(wrist.getPosition()+0.1);
            } while(gamepad2.dpad_down){
                wrist.setPosition(wrist.getPosition()-0.1);
            }

            //changes current grip position based on inputs recieved from A and B buttons on gamepad2
            if(gamepad2.a) {
                //returns grip to retracted position
                leftGrip.setPosition(0);
                rightGrip.setPosition(0);
            }
            if (gamepad2.b) {
                //sets grip to grab position
                leftGrip.setPosition(0.5);
                rightGrip.setPosition(0.5);
            }

            telemetry.addData("status", "running");
            telemetry.update();
        }
    }

}
