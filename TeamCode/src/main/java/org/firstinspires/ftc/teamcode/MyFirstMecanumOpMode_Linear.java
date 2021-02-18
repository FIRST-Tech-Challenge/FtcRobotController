package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Mecanum Test?")
public class MyFirstMecanumOpMode_Linear extends LinearOpMode {
    //    private Gyroscope imu;
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    DcMotor wobbleGoalRaiseMotor = null;
    DcMotor shooterMotor = null;
    Servo wobbleGoalGrippyThing = null;
    CRServo intakeOne = null;
  //  CRServo intakeTwo = null;
    CRServo shooterServo1 = null;
    CRServo shooterServo2 = null;


    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
//        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");


        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");
        wobbleGoalRaiseMotor = hardwareMap.dcMotor.get("wobbleLift");
        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        wobbleGoalGrippyThing = hardwareMap.servo.get("wobbleGrip");
        intakeOne = hardwareMap.crservo.get("intakeServoOne");
      //  intakeTwo = hardwareMap.crservo.get("intakeServoTwo");
        shooterServo1 = hardwareMap.crservo.get("shooterServo1");
        shooterServo2 = hardwareMap.crservo.get("shooterServo2");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
     //   intakeTwo.setDirection(CRServo.Direction.REVERSE);
        shooterServo2.setDirection(CRServo.Direction.REVERSE);

        boolean bPressed = false;
        boolean yPressed = false;
        boolean yOpen = true;
        boolean intake = false;
        boolean bRunning = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.3;
            double rx = gamepad1.right_stick_x;

            double leftFrontPower = y + x - rx;
            double leftRearPower = y - x - rx;
            double rightFrontPower = y - x + rx;
            double rightRearPower = y + x + rx;

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                double max;
                max = Math.max(leftFrontPower, leftRearPower);
                max = Math.max(max, rightFrontPower);
                max = Math.max(max, rightRearPower);

                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            leftFrontMotor.setPower(leftFrontPower);
            leftRearMotor.setPower(leftRearPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightRearMotor.setPower(rightRearPower);
//
//           double shooterPower = gamepad2.right_stick_y;
//            //double shooterPower= -0.85;
//            shooterMotor.setPower(shooterPower);

            if (gamepad2.right_trigger >= .87) {
                shooterMotor.setPower(.85);
            }

            if (gamepad2.right_bumper == true) {
                shooterMotor.setPower(0);
            }

            if (gamepad2.dpad_down == true) {
                shooterServo1.setPower(0.8);
                shooterServo2.setPower(0.8);
            }

            if (gamepad2.left_trigger >= .87) {
                wobbleGoalRaiseMotor.setPower(.6);
            } else if (gamepad2.left_bumper == true) {
                wobbleGoalRaiseMotor.setPower(-.4);
            } else {
                wobbleGoalRaiseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobbleGoalRaiseMotor.setPower(0);
            }

            if (gamepad2.a) {
                intakeOne.setPower(0.9);
            }  

//            if (gamepad2.a) {
//                intake = true;
//                if (gamepad2.a) {
//                    intake = false;
//                }
//                while (intake) {
//                    intakeOne.setPower(0.9);
//                    intakeTwo.setPower(0.9);
//                }
//            }
//            this won't work, HA HA HA

            if (gamepad2.b) {
                intakeOne.setPower(-0.9);
            }

            if (gamepad2.b) {
                if (!bPressed) {
                    bPressed = true;
                    if (bRunning) {
                        intakeOne.setPower(0);
                        bRunning = false;
                    } else {
                        intakeOne.setPower(.9);
                        bRunning = true;
                    }
                }
            } else {
                if (bPressed) {
                    bPressed = false;
                }
            }
//            if (gamepad2.y) {
//                if (!yPressed) {
//                    yPressed = true;
//                    if (yOpen) {
//                        wobbleGoalGrippyThing.setPosition(0.2);
//
//                        yOpen = false;
//                    } else {
//                        wobbleGoalGrippyThing.setPosition(.9);
//                        yOpen = true;
//                    }
//                }
//            } else {
//                if (yPressed) {
//                    yPressed = false;
//                }
//            }

            if (gamepad2.y){
                wobbleGoalGrippyThing.setPosition(0.2);
            }
            if (gamepad2.x){
                wobbleGoalGrippyThing.setPosition(0.9);
            }



        }
    }
}