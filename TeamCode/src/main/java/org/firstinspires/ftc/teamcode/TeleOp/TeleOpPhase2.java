//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//@TeleOp(group = "drive")
//public class TeleOpPhase2 extends LinearOpMode {
//    // "Exponentially weighted moving average". This class can be used to create ramping for translations and heading.
//    // Essentially, the EWMA function creates returns an exponential curve given an jump
//    // Currently, we are using EWMA for our translation ramping.
//    class Ewma {
//        double mAlpha = 0;
//        double mLastValue = 0;
//        public Ewma(double alpha) {
//            mAlpha = alpha;
//        }
//
//        public double update(double x) {
//            mLastValue = mAlpha * x + (1 - mAlpha) * mLastValue;
//            return mLastValue;
//        }
//    }
//
//    Pose2d headingToHold = new Pose2d();
//    boolean isHolding = false;
//
//    private void snapToButtons(double externalHeading) {
//        double relative = 0; // Radians
//        if (gamepad1.start) {
//            relative = externalHeading; // If the start button is pressed, everything will be relative to the current heading
//        }
//
//        if (gamepad1.x) {
//            headingToHold = new Pose2d(0, 0, Math.toRadians(90) + relative);
//        } else if (gamepad1.a) {
//            headingToHold = new Pose2d(0, 0, Math.toRadians(180) + relative);
//        } else if (gamepad1.b) {
//            headingToHold = new Pose2d(0, 0, Math.toRadians(-90) + relative);
//        } else if (gamepad1.y) {
//            headingToHold = new Pose2d(0, 0, Math.toRadians(0) + relative);
//        }
//    }
//
//    private static double sigmoid(int x, double horiStretch, double horiOffset) {
//        return 1 / (1 + Math.exp(horiStretch * (-x + horiOffset)));
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        // Hardware map
//        DcMotorEx liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor");
//        DcMotorEx intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeMotor");
//        ServoImplEx armServo1 = hardwareMap.get(ServoImplEx.class, "armServo1");
//        ServoImplEx armServo2 = hardwareMap.get(ServoImplEx.class, "armServo2");
//
//        PwmControl.PwmRange mArmServoRange = new PwmControl.PwmRange(PwmControl.PwmRange.usPulseLowerDefault, PwmControl.PwmRange.usPulseUpperDefault);
//
//        armServo1.setPwmEnable();
//        armServo1.setPwmRange(PwmControl.PwmRange.);
//        armServo2.setPwmEnable();
//        armServo2.setPwmRange();
//
//
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//
//        // Ramping for x and y translation
//        Ewma statsX = new Ewma(0.3); // Raising alpha will make the ramp more drastic but more potentially create slip
//        Ewma statsY = new Ewma(0.3); // Decreasing alpha will reduce slip
//
//        // PID controller for heading
//        PIDFController headingPID = new PIDFController(new PIDCoefficients(0.01, 0, 0.00000001), 0, 0);
//        headingPID.setInputBounds(0, 360);
//
//        // PID controller for lift
//        PIDFController liftPID = new PIDFController(new PIDCoefficients(1, 1, 1), 0, 0);
//
//        liftPID.setTargetPosition(0);
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        while (!isStopRequested()) {
//
//            double turnCommand = -gamepad1.right_stick_x;
//
//            // This if statement essentially toggles holdHeading if a button is pressed and no joystick motion is detected
//            if ((gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) && Math.abs(turnCommand) <= 0.03 ) {
//                isHolding = true;
//                snapToButtons(drive.getExternalHeading()); // Josh recommended that we make a separate method for just the a/b/x/y buttons
//            } else if (Math.abs(turnCommand) >= 0.03) {
//                isHolding = false;
//            }
//
//            if (isHolding) {
//                // holdHeading utilizes the robots onboard IMU to figure out what direction it's pointed towards
//                // Then, we use a PID controller to go to the target heading
//                double turnErrorDeg = Math.toDegrees(headingToHold.getHeading()) - drive.getExternalHeading();
//                double target = (Math.toDegrees(headingToHold.getHeading()));
//                target = target % 360.0;
//                if (target < 0) {
//                    target += 360.0;
//                }
//                headingPID.setTargetPosition(target);
//                turnCommand = headingPID.update(Math.toDegrees(drive.getExternalHeading()));
//            }
//
//            // TODO WHY???????????????????????????? (ignore??!??!!?)
//            double powX = statsX.update(-gamepad1.left_stick_y);
//            double powY = statsY.update(-gamepad1.left_stick_x);
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//
//            Vector2d input = new Vector2d(
//                    powX,
//                    powY
//            ).rotated(-poseEstimate.getHeading());
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX(),
//                            input.getY(),
//                            turnCommand
//                    )
//            );
//
//            drive.update();
//
//            // Arm stuff beyond this point:
//            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            int liftStage0 = 20;
//            int liftStage1 = -250;
//            int liftStage2 = -523;
//            int liftSpeed = 90;
//
//            if (gamepad1.left_bumper) { // Left bumper raises the arm
//                liftMotor.setPower(1);
//                liftPID.setTargetPosition(liftMotor.getCurrentPosition() - liftSpeed);
//            } else if (gamepad1.right_bumper) { // Right bumper lowers arm
//                liftMotor.setPower(1);
//                liftPID.setTargetPosition(liftMotor.getCurrentPosition() + liftSpeed);
//            }
//
//            int liftStage = 0;
//            if (gamepad1.dpad_down) {
//                liftStage = 0;
//                liftPID.setTargetPosition(liftStage0);
//            } else if (gamepad1.dpad_right|| gamepad1.dpad_left) {
//                liftStage = 1;
//                liftPID.setTargetPosition(liftStage1);
//            } else if (gamepad1.dpad_up) {
//                liftStage = 2;
//                liftPID.setTargetPosition(liftStage2);
//            }
//
//
//            liftMotor.setTargetPosition((int) liftPID.getTargetPosition());
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            if (liftMotor.getCurrentPosition() != liftMotor.getTargetPosition()) {
//                liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                int liftError = Math.abs((int) liftMotor.getCurrentPosition() - (int) liftPID.getTargetPosition());
//                liftMotor.setPower(sigmoid(liftError, 0.01, 250));
//            }
//
//            if (gamepad1.left_trigger > 0.03 || gamepad1.right_trigger > 0.03) {
//                intakeMotor.setPower(1);
//            } else {
//                intakeMotor.setPower(0.0);
//            }
//
//            if (gamepad1.left_trigger > 0.03) {
//                intakeMotor.setDirection(DcMotor.Direction.FORWARD);
//            } else if (gamepad1.right_trigger > 0.03) {
//                intakeMotor.setDirection(DcMotor.Direction.REVERSE);
//            }
//
//            armServo2.setDirection(Servo.Direction.REVERSE);
//
//            if (gamepad2.dpad_up) {
//                armServo1.setPosition(1);
//                armServo2.setPosition(1);
//            } else if (gamepad2.dpad_down) {
//                armServo1.setPosition(0);
//                armServo2.setPosition(0);
//            }
//
//
//            // Here were any values that need to be broadcast on the drive station are declare
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
//            telemetry.addData("headingToHold", Math.toDegrees(headingToHold.getHeading()));
//            telemetry.addData("liftPosition", liftMotor.getCurrentPosition());
//            telemetry.addData("targetLiftPosition", liftPID.getTargetPosition());
//            telemetry.addData("servoPosition", armServo1.getPosition());
//            telemetry.update();
//        }
//    }
//}