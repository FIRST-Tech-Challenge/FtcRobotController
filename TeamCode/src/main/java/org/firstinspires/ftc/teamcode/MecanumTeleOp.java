package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MecanumTeleOp", group="FreightFrenzy")
public class MecanumTeleOp extends LinearOpMode {

    FrenzyHardwareMap robot = new FrenzyHardwareMap();

    @Override
    public void runOpMode() throws InterruptedException {

        //import the hardware map
        robot.init(hardwareMap, telemetry);

        //set the slowdown
        double slowdown = 1.0;

        //set the controller stick values
        double controller1lstickx = 0.0;
        double controller1lsticky = 0.0;
        double controller1rstickx = 0.0;

        // init motor and add intake for arm
        robot.motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int armCurrentPos = 0;
        int armSetPos = 0;
        int armMaxPos = 1200;


        //set arm levels
        int armLevel0 = 0;   // (a)
        int armLevel1 = 250; // (x)
        int armLevel2 = 600; // (y)
        int armLevel3 = 900; // (b)

        //intake power
        double intakePower = 0.5;

        //carousel power
        double carouselPower = 0;

        //booleans for slow mode
        boolean bpressed = false;
        boolean slowdownflag = false;

        //init loop
         while (! isStarted()) {
             telemetry.addData("Arm Encoder", robot.motorArm.getCurrentPosition());
             telemetry.addData("Say GEX GEX GEX", "Hello Driver");
             telemetry.addData("arm limit switch", robot.armLimitSwitch.isPressed());
             telemetry.update();
         }


        waitForStart();

        if (isStopRequested()) return;

        //Code to test if the motors are functioning as intended. Leave commented out if not testing.
        /*

       robot.motorFrontLeft.setPower(0.5);
       telemetry.addData("motor", "front left");
       telemetry.addData("encoder", robot.motorFrontLeft.getCurrentPosition());
       telemetry.update();
       sleep(5000);
       robot.motorFrontLeft.setPower(0);

       robot.motorFrontRight.setPower(0.5);
       telemetry.addData("motor", "front right");
       telemetry.addData("encoder", robot.motorFrontRight.getCurrentPosition());
       telemetry.update();
       sleep(5000);
        robot.motorFrontRight.setPower(0);

        robot.motorBackRight.setPower(0.5);
        telemetry.addData("motor","back right");
        telemetry.addData("encoder", robot.motorBackRight.getCurrentPosition());
        telemetry.update();
        sleep(5000);
        robot.motorBackRight.setPower(0);

        robot.motorBackLeft.setPower(0.5);
        telemetry.addData("motor","back left");
        telemetry.addData("encoder", robot.motorBackLeft.getCurrentPosition());
        telemetry.update();
        sleep(5000);
        robot.motorBackLeft.setPower(0);

        robot.motorFrontLeft.setPower(0.5);
        robot.motorFrontRight.setPower(0.5);
        telemetry.addData("motor","front Right and Left");
        telemetry.update();
        sleep(5000);
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);

        robot.motorBackLeft.setPower(0.5);
        robot.motorBackRight.setPower(0.5);
        telemetry.addData("motor","back Right and Left");
        telemetry.update();
        sleep(5000);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);
        */

        robot.motorArm.setTargetPosition(0);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            //set joystick values
            controller1lstickx = gamepad1.left_stick_x;
            controller1lsticky = gamepad1.left_stick_y;
            controller1rstickx = gamepad1.right_stick_x;
            if(controller1lstickx < 0.3 && controller1lstickx > -0.3)
                controller1lstickx = 0.0;
            if(controller1lsticky < 0.3 && controller1lsticky > -0.3)
                controller1lsticky = 0.0;
            if(controller1rstickx < 0.3 && controller1rstickx > -0.3)
                controller1rstickx = 0.0;

            //set slowdown
            if(gamepad1.b == true && bpressed == false){
                if(slowdownflag){
                    slowdown = 1;
                    slowdownflag = false;
                }
                else if(slowdownflag == false){
                    slowdown = 0.5;
                    slowdownflag = true;
                }
            }
            bpressed = gamepad1.b;
            //Add slowdown
            controller1lstickx = controller1lstickx * slowdown;
            controller1lsticky = controller1lsticky * slowdown;
            controller1rstickx = controller1rstickx * slowdown;

            // Track Arm Current Pos
            armCurrentPos = robot.motorArm.getCurrentPosition();
            // Arm Up and Arm Down in small increments
            if (gamepad2.right_stick_y != 0.25) {
                //armSetPos += gamepad2.right_stick_y*10;
                if (gamepad2.right_stick_y == -1 ){
                    armSetPos = armSetPos + 5;
                } else if (gamepad2.right_stick_y == 1 ) {
                    armSetPos = armSetPos - 5;
                }
            }
            //set arm positions to gamepad.2 buttons
            if (gamepad2.a) {
                armSetPos = armLevel0;
            }else if (gamepad2.x) {
                armSetPos = armLevel1;
            }else if (gamepad2.y) {
                armSetPos = armLevel2;
            }else if (gamepad2.b) {
                armSetPos = armLevel3;
            }

            //intake
            if (gamepad2.right_trigger > 0.5){
                intakePower = -0.75;
            }
            else if (gamepad2.left_trigger > 0.5){
                intakePower = 0.6;
            }
            else if (gamepad2.left_trigger <= 0.5 && gamepad2.right_trigger <= 0.5){
                intakePower = 0;
            }
            //set power intake
            robot.motorIntake.setPower(intakePower);

            //carousel code
            if (gamepad2.right_bumper == true && !gamepad2.left_bumper) {
                carouselPower = 0.5;
            }
            else if (gamepad2.left_bumper == true && !gamepad2.right_bumper) {
                carouselPower = -0.5;
            }
            else{
                carouselPower = 0;
            }

            //set power carousel
            robot.motorCarousel.setPower(carouselPower);

            //drivetrain mecanum
            double y = controller1lsticky;
            double x = -controller1lstickx * 1.1; // * 1.1 Counteract imperfect strafing
            double rx = -controller1rstickx;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator)*slowdown;
            double backLeftPower = ((y - x + rx) / denominator)*slowdown;
            double frontRightPower = ((y - x - rx) / denominator)*slowdown;
            double backRightPower = ((y + x - rx) / denominator)*slowdown;

            robot.motorFrontLeft.setPower(frontLeftPower);
            robot.motorBackLeft.setPower(backLeftPower);
            robot.motorFrontRight.setPower(frontRightPower);
            robot.motorBackRight.setPower(backRightPower);

            //set arm power
            robot.motorArm.setTargetPosition(armSetPos);
            robot.motorArm.setPower(0.5);

            telemetry.addData("Left Stick X", controller1lstickx);
            telemetry.addData("Left Stick Y", controller1lsticky);
            telemetry.addData("Left Stick X input", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y input", gamepad1.left_stick_y);
            telemetry.addData("Left Stick Y", controller1lsticky);
            telemetry.addData("Arm pos", armCurrentPos);
            telemetry.addData("Arm Set pos", armSetPos);
            telemetry.addData("Right Y Stick",gamepad1.right_stick_y);
            telemetry.addData("Left Y Stick",gamepad1.left_stick_y);
            telemetry.addData("Slowdown:",slowdownflag);
            telemetry.update();
        }
    }
}