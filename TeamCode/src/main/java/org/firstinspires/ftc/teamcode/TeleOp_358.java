/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * 
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//  DRIVING MECANUM WHEELS SIMPLIFIED         
//             @TeleOp
//             public class MecanumTeleOp extends LinearOpMode {
//                 @Override
//                 public void runOpMode() throws InterruptedException {
//                     // Declare our motors
//                     // Make sure your ID's match your configuration
//                     // call hardware class here


//                     if (isStopRequested()) return;

//                     while (opModeIsActive()) {
//                         double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//                         double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//                         double rx = gamepad1.right_stick_x;

//                         // nominator is the largest motor power (absolute value) or 1
//                         // This ensures all the powers maintain the same ratio, but only when
//                         // at least one is out of the range [-1, 1]
//                         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                         double frontLeftPower = (y + x + rx) / denominator;
//                         double backLeftPower = (y - x + rx) / denominator;
//                         double frontRightPower = (y - x - rx) / denominator;
//                         double backRightPower = (y + x - rx) / denominator;

//                         motorFrontLeft.setPower(frontLeftPower);
//                         motorBackLeft.setPower(backLeftPower);
//                         motorFrontRight.setPower(frontRightPower);
//                         motorBackRight.setPower(backRightPower);
//                     }
//                 }
//             }

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 */

@TeleOp(name="oldeTeleop", group="reborn") // i will come soon. when the 3d print is done, make sure u watch ur back
//@Disabled
public class TeleOp_358 extends OpMode {

    Hardware358 robot = new Hardware358();
    // Declare OpMode members.

    //methods to control the speed of the robot.
    private float speedModifier = 2f;
    private float reductionModifier = .3f;//the amount that the speed will be decreased in precision mode. Should be < 1
    private float turboModifier = 1.9f;// the amount that the speed will be increased in turbo mode. Must be <2. No increase is 1.
    private float precisionActive = 1f;
    private float turnReduction = .5f;//reduces the speed of turning. <1 to reduce. 1 if to leave as normal> yuh
    //private float BRDrive = 1f;

    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
    }

    @Override
    public void loop()

    {
        mecanumMove();
    }

    public void mecanumMove()
    {
//        double clawDistanceMeasure = robot.clawDist.getDistance(DistanceUnit.MM);
//        telemetry.addData("Claw Distance", clawDistanceMeasure);
//        telemetry.addData("Distance from Back", robot.backDist.getDistance(DistanceUnit.CM));
//        telemetry.addData("Touch Sensor Pressed", robot.magStopBottom.getValue());//should indicate whether the touch sensor is pressed.


        //======================================
        //------------WHEEL CODE----------------
        //======================================
        {
            double stickX = 0;
            double stickY = 0;
            boolean form = false;
            if (Math.abs(gamepad1.right_stick_x) > 0){
                stickX = gamepad1.right_stick_x;
            }
            if(Math.abs(gamepad1.left_stick_y) > 0){
                stickY = gamepad1.left_stick_y;
            }
            if(Math.abs(gamepad1.left_stick_y) > 0){
                form = true;
                stickX = gamepad1.right_stick_x;
            }

            //variables
            double r = Math.hypot(-stickX, stickY);
            double robotAngle = Math.atan2(stickY, -stickX) - Math.PI / 4;
            final double v1 = r * Math.cos(robotAngle);
            final double v2 = r * Math.sin(robotAngle);
            final double v3 = r * Math.sin(robotAngle);
            final double v4 = r * Math.cos(robotAngle);


            if (gamepad1.left_bumper || gamepad2.left_bumper) {//if the left bumper is pressed, it multiplies the total power by the precision driving modifer
                precisionActive = reductionModifier;
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                precisionActive = turboModifier;//right bumper = turbo mode (for crossing the barriers)
            } else {
                precisionActive = 1f; //no modifier
            }

            if(form){
                robot.lf.setPower(-stickX);
                robot.rf.setPower(stickX);
                robot.lb.setPower(-stickX);
                robot.rb.setPower(stickX);
            }
            else {
                robot.lf.setPower(-speedModifier * v1 * precisionActive);
                robot.rf.setPower(-speedModifier * v2 * precisionActive);
                robot.lb.setPower(-speedModifier * v3 * precisionActive);
                robot.rb.setPower(-speedModifier * v4 * precisionActive);
            }
            telemetry.addData("fLPower", -speedModifier * v1 * precisionActive);
            telemetry.addData("fRPower", -speedModifier * v2 * precisionActive);
            telemetry.addData("bLPower", -speedModifier * v3 * precisionActive);
            telemetry.addData("bRPower", -speedModifier * v4 * precisionActive);

            telemetry.addData("Encoder port 1 back left", robot.lb.getCurrentPosition());
            telemetry.addData("Encoder port 2 front right", robot.rf.getCurrentPosition());
            telemetry.addData("Encoder port 3 back right", robot.rb.getCurrentPosition());
            telemetry.addData("Encoder port 4 back left", robot.lb.getCurrentPosition());

        }

        telemetry.addLine();

        //======================================
        //----------CLAW GRABBER----------------
        //======================================
        if(gamepad1.right_trigger > 0.5){
            robot.leftServo.setPosition(1);
            robot.rightServo.setPosition(0);
            telemetry.addData("Button X", gamepad1.x);
        }
        else if(gamepad1.b){
            robot.leftServo.setPosition(0);
            robot.rightServo.setPosition(0);
            telemetry.addData("Button B", gamepad1.b);
        }
        else {
            robot.leftServo.setPosition(0);
            robot.rightServo.setPosition(1);
            telemetry.addData("Neither", gamepad1.b);
        }

        //======================================
        //----------CLAW ROTATOR----------------
        //======================================


//        if (gamepad1.dpad_up || gamepad2.dpad_up) {
//            robot.rotateRight.setPower(-.2 * precisionActive);
//            robot.rotateLeft.setPower(-.2 * precisionActive);
//            telemetry.addData("Rotator State", "Up"); //be careful wherever you go.
//        } else if ((gamepad1.dpad_down||gamepad2.dpad_down) && (robot.magStopBottom.getValue() == 0.0 || (gamepad1.left_bumper||gamepad2.left_bumper))) {
//            robot.rotateRight.setPower(.05 * precisionActive);
//            robot.rotateLeft.setPower(.05 * precisionActive);
//            telemetry.addData("Rotator State", "Down");
//
//        } else {
//            robot.rotateRight.setPower(0);
//            robot.rotateLeft.setPower(0);
//            telemetry.addData("Rotator State", "Off"); // r u watching behind you
//        }
//
//        if(gamepad1.a || gamepad2.a){
//            robot.chuteServo.setPosition(1);
//        }
//        else {
//            robot.chuteServo.setPosition(0.5);
//        }

        telemetry.addLine();
//        telemetry.addData("In the DANGER ZONE", robot.magStopBottom.getValue());
//        telemetry.addData("Claw Rotator Position:", robot.rotateRight.getCurrentPosition());
//        telemetry.addData("Claw Rotator Position:", robot.frontLeft.getCurrentPosition());


        //======================================
        //----------CLAW ACTIVATION-------------
        //======================================

//        if (Math.max(gamepad1.right_trigger, gamepad2.right_trigger)>.1){
//            robot.clawServo.setPosition(.15);
//            telemetry.addData("Claw State", "Squeeeze");//squeeze is close
//        }
//        else {
//            robot.clawServo.setPosition(.45);
//            telemetry.addData("Claw State", "Sigh");//sigh is release
//        }


        telemetry.update();
    }
}


