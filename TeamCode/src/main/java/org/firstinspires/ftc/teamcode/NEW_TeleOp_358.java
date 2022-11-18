/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * 
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 */

@TeleOp(name="358_NEWTeleop", group="reborn") // i will come soon. when the 3d print is done, make sure u watch ur back
//@Disabled
public class NEW_TeleOp_358 extends OpMode {

    Hardware358 robot = new Hardware358();
    // Declare OpMode members.

    //methods to control the speed of the robot.
    private float speedModifier = 2f;
    private float reductionModifier = .3f;//the amount that the speed will be decreased in precision mode. Should be < 1
    private float turboModifier = 3f;// the amount that the speed will be increased in turbo mode. Must be <2. No increase is 1.
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
        Move();
    }

    public void Move()
    {
//        double clawDistanceMeasure = robot.clawDist.getDistance(DistanceUnit.MM);
//        telemetry.addData("Claw Distance", clawDistanceMeasure);
//        telemetry.addData("Distance from Back", robot.backDist.getDistance(DistanceUnit.CM));
//        telemetry.addData("Touch Sensor Pressed", robot.magStopBottom.getValue());//should indicate whether the touch sensor is pressed.


        //======================================
        //------------WHEEL CODE----------------
        //======================================

            double stickX = 0;
            double stickY = 0;
            double v1 = 0;
            double v2 = 0;
            double vm = 0;
            if (Math.abs(gamepad1.left_stick_x) > 0.2)
            {
                stickX = gamepad1.left_stick_x;
                v1 = stickX;
                v2 = -stickX;
                //stickY = 0;
            }
            if (Math.abs(gamepad1.left_stick_y) > 0.2){
                stickY = gamepad1.left_stick_y;
                v1 = -stickY;
                v2 = -stickY;
                //stickX = 0;
            }
            if (Math.abs(gamepad1.right_stick_y) > 0.2)
            {
                vm = gamepad1.right_stick_y;
            }
//
//            //variables
//           if(Math.abs(stickY) > stickX)
//           {
//               v1 = stickX;
//               v2 = -stickX;
//           }
//           else if(Math.abs(stickX) >= stickY)
//           {
//               v1 = stickX;
//               v2 = -stickX;
//           }




            if (gamepad1.left_bumper || gamepad2.left_bumper) {//if the left bumper is pressed, it multiplies the total power by the precision driving modifer
                precisionActive = reductionModifier;
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                precisionActive = turboModifier;//right bumper = turbo mode (for crossing the barriers)
            } else {
                precisionActive = 1f; //no modifier
            }


            robot.lf.setPower(-speedModifier * v1 * precisionActive);
            robot.rf.setPower(-speedModifier * v2 * precisionActive);
            robot.lb.setPower(-speedModifier * v1 * precisionActive);
            robot.rb.setPower(-speedModifier * v2 * precisionActive);
            robot.lift.setPower(-speedModifier * vm * precisionActive);

            telemetry.addData("fLPower", -speedModifier * v1 * precisionActive);
            telemetry.addData("fRPower", -speedModifier * v2 * precisionActive);
            telemetry.addData("bLPower", -speedModifier * v1 * precisionActive);
            telemetry.addData("bRPower", -speedModifier * v2 * precisionActive);
            telemetry.addData("MPower", -speedModifier * vm * precisionActive);
            telemetry.addData("LEFTPower", gamepad1.left_stick_x);




            telemetry.addData("Encoder port 1 back left", robot.lb.getCurrentPosition());
            telemetry.addData("Encoder port 2 front right", robot.rf.getCurrentPosition());
            telemetry.addData("Encoder port 3 back right", robot.rb.getCurrentPosition());
            telemetry.addData("Encoder port 4 back left", robot.lb.getCurrentPosition());
            telemetry.addData("Encoder port 0 back left", robot.m.getCurrentPosition());



        telemetry.addLine();

        //======================================
        //----------CLAW ROTATOR----------------
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

//        telemetry.addLine();
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


