/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//  DRIVING MECANUM WHEELS SIMPLIFIED
//             @TeleOp
//             public class MecanumTeleOp extends LinearOpMode {
//                 @Override
//                 public void runOpMode() throws InterruptedException {
//                     // Declare our motors
//                     // Make sure your ID's match your configuration
//                     // call hardware class here


//                     waitForStart();

//                     if (isStopRequested()) return;

//                     while (opModeIsActive()) {
//                         double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//                         double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//                         double rx = gamepad1.right_stick_x;

//                         // Denominator is the largest motor power (absolute value) or 1
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

@TeleOp(name="TELEOP") // i will come soon. when the 3d print is done, make sure u watch ur back
//@Disabled
public class    TeleOp_358 extends OpMode {

    Hardware358 robot = new Hardware358();
    // Declare OpMode members.

    //methods to control the speed of the robot.
    private float speedModifier = .65f;
    private float reductionModifier = .4f;//the amount that the speed will be decreased in precision mode. Should be < 1
    private float turboModifier = 1.9f;// the amount that the speed will be increased in turbo mode. Must be <2. No increase is 1.
    private float precisionActive = 1f;
    private float turnReduction = .5f;//reduces the speed of turning. <1 to reduce. 1 if to leave as normal> yuh
    private boolean abuttonPressed = false;
    private boolean bbuttonPressed = false;
    private boolean ybuttonPressed = false;
    private boolean clawpressed = false;

    private boolean previousXButton = false;
    //private float BRDrive = 1f;

    @Override
    public void init() {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        mecanumMove();
    }

    public void mecanumMove() {
//        double clawDistanceMeasure = robot.clawDist.getDistance(DistanceUnit.MM);
//        telemetry.addData("Claw Distance", clawDistanceMeasure);
//        telemetry.addData("Distance from Back", robot.backDist.getDistance(DistanceUnit.CM));
//        telemetry.addData("Touch Sensor Pressed", robot.magStopBottom.getValue());//should indicate whether the touch sensor is pressed.

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //======================================
        //------------WHEEL CODE----------------
        //======================================

        double stickX = 0;
        double stickY = 0;
        double stickR = 0;
        double vm = 0;
        if (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) >.2) {
            stickX = gamepad1.left_stick_x;
            stickY = gamepad1.left_stick_y;
        }
//            else {
//                stickX = gamepad2.left_stick_x;
//                stickY = gamepad2.left_stick_y;
//            }

        if (Math.abs(gamepad1.right_stick_x) >.2) {
            stickR = gamepad1.right_stick_x;
        }
//            else {
//                stickR = gamepad2.right_stick_x;
//            }
        //======================================
        //----------Lift--------------capyright 19888
        //======================================


        if (Math.abs(gamepad1.right_stick_y) >.2 ) {

            vm = gamepad1.right_stick_y;
        }
        //else if (robot.touch.isPressed()) vm = 0;
        else if((Math.abs(gamepad1.right_stick_y) <.2 && Math.abs(gamepad1.right_stick_y) >0))
            vm=0.1;

        if (Math.abs(gamepad2.right_stick_y) >.2 ) {

            vm = gamepad2.right_stick_y;
        }
        //else if (robot.touch.isPressed()) vm = 0;
//        else if (Math.abs(gamepad2.right_stick_y) <.2 && Math.abs(gamepad2.right_stick_y) >0)
//            vm=0.15;

        //FOR controlling slider
        /*
        if (Math.abs(gamepad2.left_stick_y)>.2){
            robot.slideServo.setPower(gamepad2.left_stick_y);
        }
        else {
            robot.slideServo.setPower(0);
        }

         */

        //int ticks = tickConversion * cmMove * direction;

        int tickConversion = (int)(Driving358.COUNTS_PER_MOTOR_REV/(3.14));


        int cmMove = 0;
        boolean enco=false;
        //low level
        if (gamepad2.a && !this.abuttonPressed) {
            this.abuttonPressed = true;
            cmMove = 5;
            //cmMove  = 14;
            enco = true;
            telemetry.addData("a button prssee",1);

        }

        if (gamepad2.b && !this.bbuttonPressed) {
            this.bbuttonPressed = true;
            cmMove  = 24;
            enco = true;
            telemetry.addData("b button prssee",2);
        }
        if (gamepad2.y && !this.ybuttonPressed) {
            this.ybuttonPressed = true;
            cmMove  = 34;
            enco = true;
            telemetry.addData("y button pressed",3);
        }
        this.abuttonPressed = gamepad2.a;
        this.bbuttonPressed = gamepad2.b;
        this.ybuttonPressed = gamepad2.y;


        if (enco) {
            int ticks = tickConversion * cmMove;
//                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lift.setTargetPosition((ticks));
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(.2);
            while (robot.lift.isBusy()) {

            }
            robot.lift.setPower(0);
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            enco = false;
            cmMove = 0;
        }
//change right to left bumper
        if (gamepad2.left_bumper){
            boolean test = true;
            robot.lift.setPower(-.2);


            //og code
            while (robot.lift.isBusy() || test ) {
                if(robot.touch.isPressed()){
                    robot.lift.setPower(0);
                    telemetry.addData("touch pressed",1);
                    test=false;
                    break;
                }

            }

        }

        //variables
        double r = Math.hypot(-stickX, stickY); //ur mom is watching you from the ceiling. dont look up...
        double robotAngle = Math.atan2(stickY, -stickX) - Math.PI / 4;
        double rightX = -stickR * turnReduction;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX; //the swedes are coming 4 u soon
        final double v4 = r * Math.cos(robotAngle) - rightX;

//            if (gamepad1.left_bumper || gamepad2.left_bumper) {//if the left bumper is pressed, it multiplies the total power by the precision driving modifer
//                precisionActive = reductionModifier;
//            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
//                precisionActive = turboModifier;//right bumper = turbo mode (for crossing the barriers)
//            } else {
//                precisionActive = 1f; //no modifier
//            }
        if (gamepad1.left_bumper) {//if the left bumper is pressed, it multiplies the total power by the precision driving modifer
            precisionActive = reductionModifier;
        } else if (gamepad1.right_bumper) {
            precisionActive = turboModifier;//right bumper = turbo mode (for crossing the barriers)
        } else {
            precisionActive = 1f; //no modifier
        }

        telemetry.addData("Lift mode:", robot.lift.getMode());

        robot.lf.setPower(-speedModifier * v1 * precisionActive);
        robot.rf.setPower(-speedModifier * v2 * precisionActive);
        robot.lb.setPower(-speedModifier * v3 * precisionActive);
        robot.rb.setPower(-speedModifier * v4 * precisionActive);
        robot.lift.setPower(-speedModifier * vm * precisionActive);

        telemetry.addData("fLPower", -speedModifier * v1 * precisionActive);
        telemetry.addData("fRPower", -speedModifier * v2 * precisionActive);
        telemetry.addData("bLPower", -speedModifier * v3 * precisionActive);
        telemetry.addData("bRPower", -speedModifier * v4 * precisionActive);

        telemetry.addData("Encoder port 1 back left", robot.lb.getCurrentPosition());
        telemetry.addData("Encoder port 2 front right", robot.rf.getCurrentPosition());
        telemetry.addData("Encoder port 3 back right", robot.rb.getCurrentPosition());
        telemetry.addData("Encoder port 4 back left", robot.lb.getCurrentPosition());



        telemetry.addLine();

        //======================================
        //----------CLAW--------------
        //======================================
        this.clawpressed = (gamepad2.right_trigger > 0.5);
        if (this.clawpressed) robot.clawServo.setPosition(0.0);
        else  robot.clawServo.setPosition(0.8);
        /*
        if (gamepad2.right_trigger > 0.5&& !this.clawpressed) {
            this.clawpressed = true;
            //robot.leftServo.setPosition(0.95);
            //robot.rightServo.setPosition(0.95);
            robot.clawServo.setPosition(0.0);

            telemetry.addData("Right Trigger", gamepad2.x);
        }
        else if (gamepad1.right_trigger > 0.5) {
            //robot.leftServo.setPosition(0.95);
            //robot.rightServo.setPosition(0.95);
            robot.clawServo.setPosition(0.0);
            telemetry.addData("Right Trigger", gamepad1.x);




//        if (gamepad2.right_trigger>0.5&&!this.previousXButton) {
//            robot.leftServo.setPosition(0.95);
//            robot.rightServo.setPosition(0.6);
//            telemetry.addData("Right Trigger", gamepad2.x);
//
//        }
//
        else{
            //robot.leftServo.setPosition(0.6);
            //robot.rightServo.setPosition(0.34);
            robot.clawServo.setPosition(0.8);
            telemetry.addData("Neither", gamepad2.b);
            //======================================
            //----------CLAW ROTATOR----------------
            //======================================
        }

//        this.previousXButton = gamepad2.right_trigger>0.5;

         */
    }
}

