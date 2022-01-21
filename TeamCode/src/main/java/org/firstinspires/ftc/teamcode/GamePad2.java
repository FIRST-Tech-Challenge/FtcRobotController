 package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;


 @TeleOp(name="Gamepad27", group="Linear Opmode")
 public class GamePad2 extends OpMode {
     HardwarePushbot robot = new HardwarePushbot();

     double robotAngle;
     double rightX;
     double h;

     double frontLeftPower;
     double backLeftPower;
     double frontRightPower;
     double backRightPower;


     int armStart;
     int armTarget;
     int armTop;
     int armIntake;

     int elbowStart;
     int elbowTarget;
     int elbowTop;
     int elbowIntake;

     long lastPressed = 0;
     boolean motorOn = false;
     boolean squarePressed;
     int squareCount;
     int arm;
     int elbow;
     double pmodify = .25;

     @Override
     public void init() {
         robot.init(hardwareMap);
         armStart = robot.arm.getCurrentPosition();
         elbowStart = robot.elbow.getCurrentPosition();

     }


     @Override
     public void loop() {
         telemetry.addData("Arm", "Current Position : %7d", robot.arm.getCurrentPosition());
         telemetry.addData("Elbow", "Current Position : %7d", robot.elbow.getCurrentPosition());
         telemetry.update();

         /////Drive System//////
         h = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
         robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
         rightX = gamepad1.right_stick_x;

         frontLeftPower = h * Math.sin(robotAngle) - rightX;
         frontRightPower = h * Math.cos(robotAngle) + rightX;
         backLeftPower = h * Math.cos(robotAngle) - rightX;
         backRightPower = h * Math.sin(robotAngle) + rightX;


         if (gamepad1.triangle) {
             robot.frontRight.setPower(frontRightPower * pmodify);
             robot.frontLeft.setPower(frontLeftPower * pmodify);
             robot.backRight.setPower(backRightPower * pmodify);
             robot.backLeft.setPower(backLeftPower * pmodify);

         } else
             {
             robot.frontRight.setPower(frontRightPower);
             robot.frontLeft.setPower(frontLeftPower);
             robot.backRight.setPower(backRightPower);
             robot.backLeft.setPower(backLeftPower);
             // robot.cannon.setPower(cannonPower);
         }




//////////////////////////////////arm//////////////////////////////

         if (gamepad1.left_bumper && !robot.armstop.isPressed()) {//goes
             robot.arm.setPower(0.5);


         } else if (gamepad1.left_trigger > .5  ) {// goes down
             robot.arm.setPower(-0.5);


         } else {
             robot.arm.setPower(0);
         }
 ///////////////////////elbow//////////////////////////////
         if (gamepad1.right_bumper ) {// goes up
             robot.elbow.setPower(0.5);


         } else if (gamepad1.right_trigger > .5 && !robot.elbowstop.isPressed()) {// goes down
             robot.elbow.setPower(-0.5);



         } else {
             robot.elbow.setPower(0);
         }


 ///////////////////////////intake//////////////////

         if (gamepad1.dpad_down && System.currentTimeMillis() - lastPressed > 500) {
             lastPressed = System.currentTimeMillis();
             motorOn = !motorOn;
             if (robot.intake.getPower() == 0) {
                 robot.intake.setPower(1);
                 // robot.topIntake.setPower(1);
                 //robot.windmill.setPower(1);

             } else {
                 robot.intake.setPower(0);
                 //robot.topIntake.setPower(0);
                 //  robot.windmill.setPower(0);

             }}

 //////////////////turntable////////////////////
         if(gamepad1.cross && System.currentTimeMillis() - lastPressed > 500) {
             lastPressed = System.currentTimeMillis();
             motorOn = !motorOn;
             if(robot.turntable.getPower() == 0)
             {
                 robot.turntable.setPower(1);

             } else
                 {
                 robot.turntable.setPower(0);


             }}
/////////////////////////////




/////////////////Controlled Arm and Elbow////////////

         if (gamepad1.dpad_left) {
             armTarget = armTop;
             elbowTarget = elbowTop;
         }

         if (gamepad1.dpad_right){
             armTarget= armIntake;
             elbowTarget = elbowIntake;
         }
         if (gamepad1.circle) {
             robot.arm.setTargetPosition(armTarget);
             robot.elbow.setTargetPosition(elbowTarget);

             robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             while (robot.arm.isBusy() && robot.elbow.isBusy()) {
                 telemetry.addData("ARM", "Running to %7d : %7d",
                         armTarget,
                         robot.arm.getCurrentPosition());
                 telemetry.addData("ELBOW", "Running to %7d : %7d",
                         elbowTarget,
                         robot.elbow.getCurrentPosition());
                 telemetry.update();


             // Stop all motion;
             robot.backLeft.setPower(0);
             robot.backRight.setPower(0);

             // Turn off RUN_TO_POSITION
             robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


         }

/////////////////// DriverControl Functions/////////////////////
 /*
     public void shootring3() {
         robot.ringFeeder.setPosition(.48);
         for (int i = 0; i < 3; i++) {
             timer.reset();
             while (timer.time(TimeUnit.MILLISECONDS) < 400) {
                 robot.ringFeeder.setPosition(.48);
             }
             robot.ringFeeder.setPosition(.78);
             timer.reset();
             while (timer.time(TimeUnit.MILLISECONDS) < 400) {
                 robot.ringFeeder.setPosition(.78);
             }
         }
         robot.ringFeeder.setPosition(.48);
     }
     */
             }
         }


