package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
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
    DcMotor elevator = null;
    DcMotor wobbleGoalRaiseMotor = null;
    DcMotorImplEx shooterMotor = null;
    Servo wobbleGoalGrippyThing = null;
    CRServo intakeOne = null;
    CRServo intakeTwo = null;
    CRServo trigger = null;
    DcMotor elevator1 = null;
    DcMotor elevator2 = null;
    RobotClass robot;
    private double ticks = 537;//537

    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
//        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");
        robot = new RobotClass(hardwareMap, telemetry, this);

        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");
        shooterMotor = (DcMotorImplEx) hardwareMap.dcMotor.get("shooterMotor");
        wobbleGoalGrippyThing = hardwareMap.servo.get("wobbleGrip");
        intakeOne = hardwareMap.crservo.get("intakeServoOne");
        intakeTwo = hardwareMap.crservo.get("intakeServoTwo");

        trigger = hardwareMap.crservo.get("trigger");
        elevator1 = hardwareMap.dcMotor.get("elevator1");
        elevator2 = hardwareMap.dcMotor.get("elevator2");
        wobbleGoalRaiseMotor = hardwareMap.dcMotor.get("wobbleLift");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        boolean yPressed = false;
        boolean yOpen = true;
        boolean shooterServoPressed = false;
        boolean shooterServoOn = false;
        boolean intake = false;
        wobbleGoalGrippyThing.setPosition(0.9);
        int elevatorGoal = 0;
        int elevatorPosition = 0;
        boolean wantTriggerOn = false;
        double SHOOTING_SPEED=-5400 * 0.70 * 28 / 60;

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

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
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


            if (gamepad2.left_trigger >= .87) {
                wobbleGoalRaiseMotor.setPower(.6);
            } else if (gamepad2.left_bumper == true) {
                wobbleGoalRaiseMotor.setPower(-.4);
            } else {
                wobbleGoalRaiseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobbleGoalRaiseMotor.setPower(0);
            }

            if (gamepad1.right_trigger >= .87) {
                shooterMotor.setVelocity(SHOOTING_SPEED);
            } else if (gamepad1.right_bumper == true) {
                shooterMotor.setVelocity(0);
            }

            if (gamepad2.right_trigger >= .3) {

                shooterMotor.setVelocity(SHOOTING_SPEED);
                while (shooterMotor.getVelocity() < SHOOTING_SPEED) {

                }
                    trigger.setPower(1);
                    robot.pause(450);
                    trigger.setPower(-1);
                    robot.pause(450);
                    trigger.setPower(0);
               // }
            } else {
                trigger.setPower(0);
            }


            if (gamepad2.a) {
                intakeOne.setPower(0.9);
                intakeTwo.setPower(-0.5);
            } else if (gamepad2.b) {
                intakeOne.setPower(-0.9);
                intakeTwo.setPower(0.5);
            } else if (gamepad2.x) {
                intakeOne.setPower(0);
                intakeTwo.setPower(0);

            }

            if (gamepad2.y) {
                if (!yPressed) {
                    if (yOpen) {
                        wobbleGoalGrippyThing.setPosition(0.2);
                        yOpen = false;
                    } else {
                        wobbleGoalGrippyThing.setPosition(.9);
                        yOpen = true;
                    }
                }
                yPressed = true;
            } else {
                yPressed = false;
            }
            if (gamepad1.a) {
                forwardToWhite(.9, .5, .3);
                forward(.5, -2.7);
            }

            if (gamepad2.dpad_down && elevatorGoal != -1) {
                elevatorGoal = -1;

                //Numbers completely arbitrary for theoretical purposes.
                elevator1.setTargetPosition((int) 1900);
                elevator2.setTargetPosition((int) 1900);
                elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator1.setPower(Math.abs(.8));
                elevator2.setPower(Math.abs(.8));


            }
            if (gamepad2.dpad_up && elevatorGoal != 1) {
                elevatorGoal = 1;

                elevator1.setTargetPosition((int) 0);
                elevator2.setTargetPosition((int) 0);
                elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator1.setPower(Math.abs(.8));
                elevator2.setPower(Math.abs(.8));


            }
            if (elevatorGoal == 1) {
                if (elevator1.getCurrentPosition() <= 0 || elevator2.getCurrentPosition() <= 0) {
                    elevatorPosition = 1;
                    elevatorGoal = 0;
                    elevator1.setPower(0);
                    elevator2.setPower(0);

                }
            } else if (elevatorGoal == -1) {
                if (elevator1.getCurrentPosition() >= 1900 || elevator2.getCurrentPosition() >= 1900) {
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

//            telemetry.addData("Elevator motor 1 current position: ", elevator1.getCurrentPosition());
//            telemetry.addData("Elevator motor 2 current position: ", elevator2.getCurrentPosition());
//            telemetry.addData("Current goal: ", elevatorGoal);
//            telemetry.addData("Current read position: ", elevatorPosition);
//            telemetry.update();


        }
    }



    public void forwardToWhite (double speed, double rotations, double speed2) {
        robot.frontLeft.setPower(speed2);
        robot.frontRight.setPower(speed2);
        robot.backLeft.setPower(speed2);
        robot.backRight.setPower(speed2);

        while (robot.colorSensor.alpha() < 600 && !gamepad1.x) {

            telemetry.addData("Light Level: ", robot.colorSensor.alpha());
            telemetry.update();
        }

        robot.stopMotors();
    }
    public void forward (double speed, double rotations){
        int leftCurrent = robot.frontLeft.getCurrentPosition();
        int rightCurrent = robot.frontRight.getCurrentPosition();
        int backLeftCurrent = robot.backLeft.getCurrentPosition();
        int backRightCurrent = robot.backRight.getCurrentPosition();

        telemetry.addData("Target Front Left Motor Position", leftCurrent);
        telemetry.addData("Target Front Right Motor Position", rightCurrent);
        telemetry.addData("Target Back Left Motor Position", backLeftCurrent);
        telemetry.addData("Target Back Right Motor Position", backRightCurrent);
        telemetry.update();

        double toPositionLeft = leftCurrent + rotations*ticks;
        double toPositionRight = rightCurrent + rotations*ticks;
        double toPositionbackLeft = backLeftCurrent + rotations*ticks;
        double toPositionbackRight = backRightCurrent + rotations*ticks;

        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
        telemetry.addData("Target Front Right Motor Position", toPositionRight);
        telemetry.addData("Target Back Left Motor Position", toPositionbackLeft);
        telemetry.addData("Target Front Left Motor Position", toPositionbackLeft);
        telemetry.update();

        robot.frontLeft.setTargetPosition((int)toPositionLeft);
        robot.frontRight.setTargetPosition((int)toPositionRight);
        robot.backLeft.setTargetPosition((int)toPositionbackLeft);
        robot.backRight.setTargetPosition((int)toPositionbackRight);

        robot.motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(Math.abs(speed));
        robot.frontRight.setPower(Math.abs(speed));
        robot.backLeft.setPower(Math.abs(speed));
        robot.backRight.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) && !gamepad1.x) {

            // Display it for the driver.
            robot.motorTelemetry();
        }
        robot.stopMotors();

        robot.motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
