//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp Comp", group = "TeleOp")
public class TeleOpComp extends MasterOpMode{

    int tickvalue = 97;
    double x = 0.7;

    //for run to position or manual control
    boolean toPosition = true;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        Initialize();

        //Set position of servos
        servoGrabber.setPosition(0.34);
        //servoArm.setPosition(0.3);

        //Declare variables
        int position = 0;
        int addingticks = 0;
        boolean isPressed = false;
        double motorPower = 0.9;
        double increase = 1;
        double speed =1;
        int motorBeltTargetPosition = 0;

        waitForStart();

        //Set power of motors
        while (opModeIsActive()) {
            motorBL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * speed);
            motorFL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * speed);
            motorBR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * speed);
            motorFR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * speed);

            //Use switch to declare values for each arm position
            switch (position) {
                case -2:
                    servoArm.setPosition(0.8);
                    motorArm.setPower(motorPower);
                    tickvalue = 555;
                    break;
                case -1:
                    servoArm.setPosition(0.1);
                    motorArm.setPower(motorPower);
                    tickvalue = 730;
                    break;
                case 0:
                    servoArm.setPosition(0.6);
                    motorArm.setPower(motorPower);
                    tickvalue = 100;
                    break;
                case 1:
                    servoArm.setPosition(0.55);
                    motorArm.setPower(motorPower);
                    tickvalue = 300;
                    break;
                case 2:
                    servoArm.setPosition(0.45);
                    motorArm.setPower(motorPower);
                    tickvalue = 555;
                    break;
                case 3:
                    servoArm.setPosition(0.3);
                    motorArm.setPower(motorPower);
                    tickvalue = 900;
                    break;
                case 4:
                    servoArm.setPosition(0.25);
                    motorArm.setPower(motorPower);
                    tickvalue = 1000;
                    break;
                case 5:
                    servoArm.setPosition(0.55);
                    motorArm.setPower(motorPower);
                    tickvalue = 1980;
                    break;
                case 6:
                    servoArm.setPosition(0.4);
                    motorArm.setPower(motorPower);
                    tickvalue = 2310;
                    break;
                case 7:
                    servoArm.setPosition(0.55);
                    motorArm.setPower(motorPower);
                    tickvalue = 2330;
                    break;
            }

            motorBelt.setPower(-(gamepad2.left_stick_y));
            if (gamepad1.left_trigger>0){
                speed = 0.25;
            } else {
                speed = 1;
            }
            telemetry.addData("Motor Ticks: ", motorArm.getCurrentPosition());
            telemetry.addData("ServoArmPosition",servoArm.getPosition());
            telemetry.update();

            // checks old position of arm, right when it goes over top of robot from front to back, it reduces speed
            if (gamepad2.dpad_up){
                if (!isPressed) {
                    if (position == -2){
                        increase = 4;
                    }
                    else {
                        increase = 1;
                    }
                    position += increase;
                    if (position > 7){
                        position = 7;
                    }
                }
                addingticks = 0;
                toPosition = true;
                isPressed = true;
            }

            // checks old arm position, when it goes over the top of the robot from back to front, it reduces speed
            if (!gamepad2.dpad_up && !gamepad2.dpad_down){
                isPressed = false;
            } else if (gamepad2.dpad_down){
                if (!isPressed) {
                    position -= increase;
                    if (position < 0){
                        position = 0;
                    }
                }
                addingticks = 0;
                toPosition = true;
                isPressed = true;
            } else {
                motorPower = 0.5;
            }

            if (gamepad2.right_stick_y > 0.5) {
                addingticks -= 2;
            } else if (gamepad2.right_stick_y < -0.5) {
                addingticks += 2;
            }

            motorArm.setTargetPosition(tickvalue + addingticks);
            motorArm.setPower(0.9);

            if (gamepad2.x) {
                if (position == 0){
                    servoGrabber.setPosition(0.6);
                } else {
                    servoGrabber.setPosition(0.5);
                }
            } else if (gamepad2.a) {
                servoGrabber.setPosition(0.0);
            }

            if (gamepad2.y){
                position = -1;
            }
            else if (gamepad2.b){
                position = -2;
            }

            if (gamepad2.right_bumper) {
                motorBR.setPower(0);
                motorBL.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
                x = 0.7;
                while (true) {
                    motorRightDuck.setPower(x);
                    motorLeftDuck.setPower(x);
                    pauseMillis(150);
                    x += 0.05;
                    telemetry.addData("duckPower", motorRightDuck.getPower());
                    telemetry.update();
                    if (x >= 0.85){
                        pauseMillis(1500);
                        motorRightDuck.setPower(-.1);
                        motorLeftDuck.setPower(-.1);
                        pauseMillis(30);
                        motorRightDuck.setPower(0);
                        motorLeftDuck.setPower(0);
                        x=0.7;
                        break;
                    }
                }
            } else if (gamepad2.left_bumper) {
                motorBR.setPower(0);
                motorBL.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
                x = -0.7;
                while (true) {
                    motorRightDuck.setPower(x);
                    motorLeftDuck.setPower(x);
                    pauseMillis(150);
                    x -= 0.05;
                    telemetry.addData("duckPower", motorRightDuck.getPower());
                    telemetry.update();
                    if (x <= -0.85){
                        pauseMillis(1500);
                        motorRightDuck.setPower(.1);
                        motorLeftDuck.setPower(.1);
                        pauseMillis(30);
                        motorRightDuck.setPower(0);
                        motorLeftDuck.setPower(0);
                        x=0.7;
                        break;
                    }
                }
            }
            if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5){
                servoGrabber.setPosition(0.34);
                servoArm.setPosition(0.8);
                motorArm.setTargetPosition(-930);
                pauseMillis(700);
                motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pauseMillis(1000);
                motorArm.setTargetPosition(0);
                motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                position = 0;
            }
        }
    }
}