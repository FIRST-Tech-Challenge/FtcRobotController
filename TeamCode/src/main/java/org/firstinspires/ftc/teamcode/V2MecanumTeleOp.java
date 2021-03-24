package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum TeleOp Version 2")
public class V2MecanumTeleOp extends LinearOpMode {
    //    private Gyroscope imu;
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
////    private Servo servoTest;
//    DcMotor leftFrontMotor = null;
//    DcMotor rightFrontMotor = null;
//    DcMotor leftRearMotor = null;
////    DcMotor rightRearMotor = null;
//    DcMotor wobbleGoalExtendMotor = null;
//    DcMotor wobbleGoalRaiseMotor = null;
    DcMotor elevator1 = null;
    DcMotor elevator2 = null;
//    DcMotorImplEx shooterMotor = null;
////    Servo wobbleGoalGrippyThing = null;
//    CRServo intakeOne = null;
//    //  CRServo intakeTwo = null;
//    CRServo shooterServo1 = null;
//    CRServo shooterServo2 = null;
//    RobotClass robot;
    private double ticks = 537;//537


    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
//        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");
//        robot= new RobotClass(hardwareMap, telemetry, this);

//        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
//        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
//        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
//        rightRearMotor = hardwareMap.dcMotor.get("backRight");
//        wobbleGoalExtendMotor = hardwareMap.dcMotor.get("wobbleExtendo");
//        wobbleGoalRaiseMotor = hardwareMap.dcMotor.get("wobbleLift");
        elevator1 = hardwareMap.dcMotor.get("elevator1");
        elevator2 = hardwareMap.dcMotor.get("elevator2");
//        shooterMotor = (DcMotorImplEx) hardwareMap.dcMotor.get("shooterMotor");
//        wobbleGoalGrippyThing = hardwareMap.servo.get("wobbleGrip");
//        intakeOne = hardwareMap.crservo.get("intakeServoOne");
//        //  intakeTwo = hardwareMap.crservo.get("intakeServoTwo");
//        shooterServo1 = hardwareMap.crservo.get("shooterServo1");
//        shooterServo2 = hardwareMap.crservo.get("shooterServo2");
//
//        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
//        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
//        //   intakeTwo.setDirection(CRServo.Direction.REVERSE);
//
//        boolean yPressed = false;
//        boolean yOpen = true;
//        boolean shooterServoPressed = false;
//        boolean shooterServoOn= false;
//        boolean intake = false;
//        wobbleGoalGrippyThing.setPosition(0.9);
        int elevatorGoal = 0;
        int elevatorPosition = 0;

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
//
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            double leftFrontPower = y + x - rx;
//            double leftRearPower = y - x - rx;
//            double rightFrontPower = y - x + rx;
//            double rightRearPower = y + x + rx;
//
//            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {
//
//                double max;
//                max = Math.max(leftFrontPower, leftRearPower);
//                max = Math.max(max, rightFrontPower);
//                max = Math.max(max, rightRearPower);
//
//                leftFrontPower /= max;
//                leftRearPower /= max;
//                rightFrontPower /= max;
//                rightRearPower /= max;
//            }
//
//            leftFrontMotor.setPower(leftFrontPower);
//            leftRearMotor.setPower(leftRearPower);
//            rightFrontMotor.setPower(rightFrontPower);
//            rightRearMotor.setPower(rightRearPower);
//
//            if (gamepad2.dpad_down) {
//                if (!shooterServoPressed) {
//                    if (shooterServoOn) {
//                        shooterServo1.setPower(0);
//                        shooterServo2.setPower(0);
//                        shooterServoOn = false;
//                    } else {
//                        shooterServo1.setPower(0.8);
//                        shooterServo2.setPower(0.8);
//                        shooterServoOn = true;
//                    }
//                }
//                shooterServoPressed=true;
//            } else {
//                shooterServoPressed = false;
//            }
//
//
//            if (gamepad2.left_trigger >= .87) {
//                wobbleGoalRaiseMotor.setPower(.6);
//            } else if (gamepad2.left_bumper == true) {
//                wobbleGoalRaiseMotor.setPower(-.4);
//            } else {
//                wobbleGoalRaiseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                wobbleGoalRaiseMotor.setPower(0);
//            }
//
//            if (gamepad2.right_trigger >= .87) {
//                shooterMotor.setVelocity(-5400*0.85*28/60);
//            } else if(gamepad2.right_bumper == true){
//                shooterMotor.setVelocity(0);
//            }
//
//
//            if (gamepad2.right_bumper == true) {
//                wobbleGoalExtendMotor.setPower(-.5);
//            } else {
//                wobbleGoalExtendMotor.setPower(0);
//            }
//
//            if (gamepad2.a) {
//                intakeOne.setPower(0.9);
//            } else if (gamepad2.b) {
//                intakeOne.setPower(-0.9);
//            } else if (gamepad2.x){
//                intakeOne.setPower(0);
//            }
//            /*beep boop
//             *
//             * robot thigd
//             *
//             *
//             * */
//            if (gamepad2.y) {
//                if (!yPressed) {
//                    if (yOpen) {
//                        wobbleGoalGrippyThing.setPosition(0.2);
//                        yOpen = false;
//                    } else {
//                        wobbleGoalGrippyThing.setPosition(.9);
//                        yOpen = true;
//                    }
//                }
//                yPressed=true;
//            } else {
//                yPressed = false;
//            }
//            if (gamepad1.a) {
//                robot.forwardToWhite(.9,.5,.3);
//                robot.forward(.5, -2.7);
//            }

            if (gamepad2.dpad_down && elevatorGoal != -1) {
                elevatorGoal = -1;

                //Numbers completely arbitrary for theoretical purposes.
                elevator1.setTargetPosition((int)2500);
                elevator2.setTargetPosition((int)2500);
                elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator1.setPower(Math.abs(.3));
                elevator2.setPower(Math.abs(.3));


            }
            if (gamepad2.dpad_up && elevatorGoal != 1) {
                elevatorGoal = 1;

                elevator1.setTargetPosition((int)0);
                elevator2.setTargetPosition((int)0);
                elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator1.setPower(Math.abs(.3));
                elevator2.setPower(Math.abs(.3));


            }
            if (elevatorGoal == 1) {
                if (elevator1.getCurrentPosition() <= 0 || elevator2.getCurrentPosition() <= 0) {
                    elevatorPosition = 1;
                    elevatorGoal = 0;
                    elevator1.setPower(0);
                    elevator2.setPower(0);

                }
            } else if (elevatorGoal == -1) {
                if (elevator1.getCurrentPosition() >= 2500  || elevator2.getCurrentPosition() >= 2500) {
                    elevatorPosition = -1;
                    elevatorGoal = 0;
                    elevator1.setPower(0);
                    elevator2.setPower(0);

                }
            }
            if (gamepad2.dpad_right) {
                elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator1.setPower(0);
                elevator2.setPower(0);
                elevatorGoal = 0;
            }

            telemetry.addData("Elevator motor 1 current position: ", elevator1.getCurrentPosition());
            telemetry.addData("Elevator motor 2 current position: ", elevator2.getCurrentPosition());
            telemetry.addData("Current goal: ", elevatorGoal);
            telemetry.addData("Current read position: ", elevatorPosition);
            telemetry.update();




//            while (gamepad2.dpad_left) {
//                /*
//                servo.setPosition(.7);
//                robot.pause();
//                servo.setPosition(.2);
//                 */
//            }

        }
    }
}
