//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp Comp", group = "TeleOp")
public class TeleOpComp extends LinearOpMode{
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
        servoGrabber.setPosition(0.34);
        servoArm.setPosition(0.3);

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

        waitForStart();

        //Set power of motors
        while (opModeIsActive()) {
            motorBackLeft.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * speed);
            motorFrontLeft.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * speed);
            motorBackRight.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * speed);
            motorFrontRight.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * speed);

            //Use switch to declare values for each arm position
            switch (position) {
                case 0:
                    motorArm.setTargetPosition(-55);
                    servoArm.setPosition(0.3);
                    motorArm.setPower(motorPower);
                    break;
                case 1:
                    motorArm.setTargetPosition(-195);
                    servoArm.setPosition(0.4);
                    motorArm.setPower(motorPower);
                    break;
                case 2:
                    motorArm.setTargetPosition(-430);
                    servoArm.setPosition(0.6);
                    motorArm.setPower(motorPower);
                    break;
                case 3:
                    motorArm.setTargetPosition(-690);
                    servoArm.setPosition(0.8);
                    motorArm.setPower(motorPower);
                    break;
                case 4:
                    motorArm.setTargetPosition(-1260);
                    servoArm.setPosition(0.4);
                    motorArm.setPower(motorPower);
                    break;
                case 5:
                    motorArm.setTargetPosition(-1540);
                    servoArm.setPosition(0.7);
                    motorArm.setPower(motorPower);
                    break;
                case 6:
                    motorArm.setTargetPosition(-1620);
                    servoArm.setPosition(0.6);
                    motorArm.setPower(motorPower);
                    break;
            }
            if (gamepad1.left_trigger>0){
                speed = 0.3;

            }
            else {
                speed = 1;
            }
            telemetry.addData("Motor Ticks: ", motorArm.getCurrentPosition());
            telemetry.update();

            // checks old position of arm, right when it goes over top of robot from front to back, it reduces speed
            if (gamepad2.dpad_up){
                if (!isPressed) {
                    position += increase;
                }
                isPressed = true;
            }

            // checks old arm position, when it goes over the top of the robot from back to front, it reduces speed
            if (!gamepad2.dpad_up && !gamepad2.dpad_down){
                isPressed = false;
            }
            else if (gamepad2.dpad_down){
                if (!isPressed) {
                    position -= increase;
                }
                isPressed = true;
            }

            else {
                motorPower = 0.5;
            }

            while (gamepad2.left_trigger >= 0.5) {
                motorArm.setTargetPosition(motorArm.getCurrentPosition()+2);
            }
            while (gamepad2.right_trigger >= 0.5) {
                motorArm.setTargetPosition(motorArm.getCurrentPosition()-2);
            }


            if (position < 0){
                position = 0;
            }
            else if (position > 6){
                position = 6;
            }

            if(gamepad2.x) {
                servoGrabber.setPosition(0.34);
            }
            else if (gamepad2.a) {
                servoGrabber.setPosition(1);
            }

            if (gamepad1.right_bumper) {
                x = 0.7;
                while (true) {
                    motorDuck.setPower(x);
                    pauseMillis(150);
                    x += 0.05;
                    telemetry.addData("duckPower", motorDuck.getPower());
                    telemetry.update();
                    if (x >= 0.85){
                        pauseMillis(1500);
                        motorDuck.setPower(-.1);
                        pauseMillis(30);
                        motorDuck.setPower(0);
                        x=0.7;
                        break;
                    }
                }
            }
            else if (gamepad1.left_bumper) {
                x = -0.7;
                while (true) {
                    motorDuck.setPower(x);
                    pauseMillis(150);
                    x -= 0.05;
                    telemetry.addData("duckPower", motorDuck.getPower());
                    telemetry.update();
                    if (x <= -0.85){
                        pauseMillis(1500);
                        motorDuck.setPower(.1);
                        pauseMillis(30);
                        motorDuck.setPower(0);
                        x=0.7;
                        break;
                    }
                }
            }
        }
    }
    public void pauseMillis(double time) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            idle();
        }
    }
}