package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class TeleOpPhase3 extends LinearOpMode {
    // "Exponentially weighted moving average". This class can be used to create ramping for translations and heading.
    // Essentially, the EWMA function creates returns an exponential curve given an jump
    // Currently, we are using EWMA for our translation ramping.
    class Ewma {
        double mAlpha = 0;
        double mLastValue = 0;
        public Ewma(double alpha) {
            mAlpha = alpha;
        }

        public double update(double x) {
            mLastValue = mAlpha * x + (1 - mAlpha) * mLastValue;
            return mLastValue;
        }
    }

    private static double sigmoid(int x, double horiStretch, double horiOffset) {
        return 1 / (1 + Math.exp(horiStretch * (-x + horiOffset)));
    }

    class Lift {
        private DcMotorEx mLiftMotor;
        private PIDFController mLiftPID;
        private int mLiftStage0, mLiftStage1, mLiftStage2, mLiftStage3;
        private int mLiftStage = 0;
        private int mLiftSpeed = 90;

        public Lift(DcMotorEx liftMotor, PIDFController liftPID, int liftStage0, int liftStage1, int liftStage2, int liftStage3, int liftSpeed) {
            mLiftMotor = liftMotor;
            mLiftPID = liftPID;
            mLiftStage0 = liftStage0;
            mLiftStage1 = liftStage1;
            mLiftStage2 = liftStage2;
            mLiftStage3 = liftStage3;
            mLiftSpeed = liftSpeed;
        }

        public void setReverse(boolean isReverse) {
            if (isReverse) {
                mLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }

        public void setLiftStage(int x) {
            mLiftStage = x;
        }

        public int getLiftStage() {
            return mLiftStage;
        }

        public int getLiftPosition() {
            return mLiftMotor.getCurrentPosition();
        }

        public void setLiftPower() {
            int mLiftError = Math.abs((int) mLiftMotor.getCurrentPosition() - (int) mLiftPID.getTargetPosition());
            mLiftMotor.setPower(sigmoid(mLiftError, 0.01, 250));
        }

        public void setLiftPower(double liftPower) {
            mLiftMotor.setPower(liftPower);
        }

        public void goToLiftStage() {
            // Corresponding each liftStage with a target position
            switch (mLiftStage) {
                case 0:
                    mLiftPID.setTargetPosition(mLiftStage0);
                    break;
                case 1:
                    mLiftPID.setTargetPosition(mLiftStage1);
                    break;
                case 2:
                    mLiftPID.setTargetPosition(mLiftStage2);
                    break;
                case 3:
                    mLiftPID.setTargetPosition(mLiftStage3);
            }

            // Making the liftMotor run to the designated position (measured in ticks)
            mLiftMotor.setTargetPosition((int) mLiftPID.getTargetPosition());
            mLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Set power of the liftMotor depending on how close it is to the target postion
            mLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            setLiftPower(1.0); // TESTING
        }

        public void setLiftPosition(String direction) {
            switch (direction) {
                case "up":
                    mLiftPID.setTargetPosition(mLiftMotor.getCurrentPosition() + mLiftSpeed);
                    break;
                case "down":
                    mLiftPID.setTargetPosition(mLiftMotor.getCurrentPosition() - mLiftSpeed);
                    break;
            }

            // Making the liftMotor run to the designated position, but this time, it's affected by the lift speed
            mLiftMotor.setTargetPosition((int) mLiftPID.getTargetPosition());
            mLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mLiftMotor.setPower(0.5);
        }

        public void setLiftPosition(int positionTicks) {
            mLiftPID.setTargetPosition(positionTicks);
            mLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mLiftMotor.setPower(0.5);
        }
    }

    class Arm {
        private ServoImplEx mArmServo;
        private double mArmStage0, mArmStage1, mArmStage2;
        private int mArmStage = 0;
        private PwmControl.PwmRange mArmServoRange = new PwmControl.PwmRange(500, 2500);

        public Arm(ServoImplEx armServo, double armStage0, double armStage1, double armStage2, double offset) {
            mArmServo = armServo;
            mArmStage0 = armStage0 - offset;
            mArmStage1 = armStage1 - offset;
            mArmStage2 = armStage2 - offset;
            mArmServo.setPwmEnable();
            mArmServo.setPwmRange(mArmServoRange);
        }

        public void setReverse(boolean isReverse) {
            if (isReverse) {
                mArmServo.setDirection(Servo.Direction.REVERSE);
            }
        }

        public void setArmPosition(double x) {
            mArmServo.setPosition(x);
        }

        public double getArmPosition() {
            return mArmServo.getPosition();
        }

        public void setArmStage(int x) {
            mArmStage = x;
        }

        public int getArmStage() {
            return mArmStage;
        }

        public void goToArmStage() {
            switch (mArmStage) {
                case 0:
                    mArmServo.setPosition(mArmStage0);
                    break;
                case 1:
                    mArmServo.setPosition(mArmStage1);
                    break;
                case 2:
                    mArmServo.setPosition(mArmStage2);
            }
        }
    }

    class Claw {
        private ServoImplEx mWristServo;
        private ServoImplEx mFingerServo;
        private boolean mIsClawHolding;
        private boolean mIsClawIntaking;
        private double mWristStage0, mWristStage1;
        private double mFingerStage0, mFingerStage1;
        private PwmControl.PwmRange mClawRange = new PwmControl.PwmRange(500, 2500);

        public Claw(ServoImplEx wristServo, ServoImplEx fingerServo, double wristStage0, double wristStage1, double fingerStage0, double fingerStage1) {
            mWristServo = wristServo;
            mFingerServo = fingerServo;
            mWristStage0 = wristStage0;
            mWristStage1 = wristStage1;
            mFingerStage0 = fingerStage0;
            mFingerStage1 = fingerStage1;
        }

        public void setFinger(String holdOrRelease) {
            switch (holdOrRelease) {
                case "hold":
                    mIsClawHolding = true;
                    break;
                case "release":
                    mIsClawHolding = false;
                    break;
            }
        }

        public void goToFingerStage() {
            // Hold claw
            if (mIsClawHolding) {
                mFingerServo.setPosition(mFingerStage1);
            } else {
                mFingerServo.setPosition(mFingerStage0);
            }
        }

        public void setWrist(String intakeOrScore) {
            switch (intakeOrScore) {
                case "intake":
                    mIsClawIntaking = true;
                    break;
                case "score":
                    mIsClawIntaking = false;
                    break;
            }
        }

        public void goToWristStage() {
            if (mIsClawIntaking) {
                mWristServo.setPosition(mWristStage0);
            } else {
                mWristServo.setPosition(mWristStage1);
            }
        }

        public String getClawState() {
            return "isIntaking: " + mIsClawIntaking + ", isHolding: " + mIsClawHolding;
        }

    }



    public static void initializeLiftAndArm(Lift liftMotor1, Lift liftMotor2, Arm armServo1, Arm armServo2) {
        int liftThreshold = -50;
        double armThreshold = 0.3;

        liftMotor1.setLiftPosition(liftThreshold);
        liftMotor2.setLiftPosition(liftThreshold);
        liftMotor1.setLiftPower(0.5);
        liftMotor2.setLiftPower(0.5);
        armServo1.setArmPosition(armThreshold);
        armServo2.setArmPosition(armThreshold);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Hardware map
        DcMotorEx intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeMotor");
        DcMotorEx liftMotor1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor1");
        DcMotorEx liftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor2");
        ServoImplEx armServo1 = hardwareMap.get(ServoImplEx.class, "armServo1");
        ServoImplEx armServo2 = hardwareMap.get(ServoImplEx.class, "armServo2");
        ServoImplEx wristServo = hardwareMap.get(ServoImplEx.class, "wristServo");
        ServoImplEx fingerServo = hardwareMap.get(ServoImplEx.class, "fingerServo");


        waitForStart();

        // Ramping for x and y translation
        TeleOpPhase3.Ewma statsX = new TeleOpPhase3.Ewma(0.3); // Raising alpha will make the ramp more drastic but more potentially create slip
        TeleOpPhase3.Ewma statsY = new TeleOpPhase3.Ewma(0.3); // Decreasing alpha will reduce slip

        // PID controller for heading
        PIDFController headingPID = new PIDFController(new PIDCoefficients(0, 0, 0), 0, 0); // Heading is kinda messed up rn, so I'm temporarily disabling it
        headingPID.setInputBounds(0, 360);

        // PID controller for lift
        PIDFController liftPID = new PIDFController(new PIDCoefficients(1, 1, 1), 0, 0);

        // Initializing some variables necessary for driving
        Pose2d headingToHold = new Pose2d();
        boolean isHolding = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStopRequested()) {

            // Drive spaghetti code
            double turnCommand = -gamepad1.right_stick_x;

            // This if statement essentially toggles holdHeading if a button is pressed and no joystick motion is detected
            if ((gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) && Math.abs(turnCommand) <= 0.03 ) {
                isHolding = true;
                double relativeHeading;

                if (gamepad1.start) {
                    relativeHeading = drive.getExternalHeading(); // If the start button is pressed, everything will be relative to the current heading
                } else {
                    relativeHeading = 0;
                }

                if (gamepad1.x) {
                    headingToHold = new Pose2d(0, 0, Math.toRadians(90) + relativeHeading);
                } else if (gamepad1.a) {
                    headingToHold = new Pose2d(0, 0, Math.toRadians(180) + relativeHeading);
                } else if (gamepad1.b) {
                    headingToHold = new Pose2d(0, 0, Math.toRadians(-90) + relativeHeading);
                } else if (gamepad1.y) {
                    headingToHold = new Pose2d(0, 0, Math.toRadians(0) + relativeHeading);
                }
            } else if (Math.abs(turnCommand) >= 0.03) {
                isHolding = false;
            }

            if (isHolding) {
                // holdHeading utilizes the robots onboard IMU to figure out what direction it's pointed towards
                // Then, we use a PID controller to go to the target heading
                double turnErrorDeg = Math.toDegrees(headingToHold.getHeading()) - drive.getExternalHeading();
                double target = (Math.toDegrees(headingToHold.getHeading()));
                target = target % 360.0;
                if (target < 0) {
                    target += 360.0;
                }
                headingPID.setTargetPosition(target);
                turnCommand = headingPID.update(Math.toDegrees(drive.getExternalHeading()));
            }

            // TODO WHY???????????????????????????? (ignore??!??!!?)
            double powX = statsX.update(-gamepad1.left_stick_y);
            double powY = statsY.update(-gamepad1.left_stick_x);

            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    powX,
                    powY
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            turnCommand
                    )
            );

            drive.update();

            // Intake stuff
            if (gamepad1.left_trigger > 0.03 || gamepad1.right_trigger > 0.03) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0.0);
            }
            if (gamepad1.left_trigger > 0.03) {
                intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            } else if (gamepad1.right_trigger > 0.03) {
                intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            }

            // Lift & Arm stuff
            int liftStage = 0;
            int liftSpeed = -90;

            int liftStage0 = 0; // Setpoint for intaking pixles
            int liftStage1 = -20; // Setpoint for below first set line
            int liftStage2 = -40; // Setpoint at first set line
            int liftStage3 = -1000; // Setpoint for highest we can go

            int armStage = 0;
            double armStage0 = 0; // Setpoint for intake
            double armStage1 = 1; // Setpoint for placing stuff
            double armStage2 = 1; // Likely won't use this but I'll add it just in case we have a 3 axis arm

            Lift lift1 = new Lift(liftMotor1, liftPID, liftStage0, liftStage1, liftStage2, liftStage3, liftSpeed);
            Lift lift2 = new Lift(liftMotor2, liftPID, liftStage0, liftStage1, liftStage2, liftStage3, liftSpeed);

            Arm arm1 = new Arm(armServo1, armStage0, armStage1, armStage2, 0.0);
            Arm arm2 = new Arm(armServo2, armStage0, armStage1, armStage2, 0.0);

            // Both lifts are in the right direction (i hope)
            lift1.setReverse(false);
            lift2.setReverse(false);

            // One servo is oriented in the opposite direction
            arm1.setReverse(true);
            arm2.setReverse(false);

            // Do some sequence of events if the lift is going from intake to setpoint
            if (liftStage == 1 && (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_down) || (gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_down)) {
                initializeLiftAndArm(lift1, lift2, arm1, arm2);
            }

            // Lift setpoints
            if (gamepad2.dpad_down || gamepad1.dpad_down) {
                liftStage = 0;
                armStage = 0;
            } else if (gamepad2.dpad_left || gamepad1.dpad_left || gamepad2.dpad_right || gamepad1.dpad_right) {
                liftStage = 1;
                armStage = 1;
            } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                liftStage = 2;
                armStage = 1;
            }
            lift1.setLiftStage(liftStage);
            lift2.setLiftStage(liftStage);
            lift1.goToLiftStage();
            lift2.goToLiftStage();
            arm1.setArmStage(armStage);
            arm2.setArmStage(armStage);
            arm1.goToArmStage();
            arm2.goToArmStage();

            // Manual lift control
            if (gamepad2.left_bumper) {
                lift1.setLiftPosition("up");
                lift2.setLiftPosition("up");
            } else if (gamepad2.right_bumper) {
                lift1.setLiftPosition("down");
                lift2.setLiftPosition("down");
            }

            // Manual arm control
            if (gamepad2.a) { // Intake
                arm1.setArmStage(0);
                arm2.setArmStage(0);
                arm1.goToArmStage();
                arm2.goToArmStage();
            } else if (gamepad2.y) { // Placing
                arm1.setArmStage(1);
                arm2.setArmStage(1);
                arm1.goToArmStage();
                arm2.goToArmStage();
            } else if (gamepad2.b) { // Fine tuning
                arm1.setArmPosition(armServo1.getPosition() + 0.3);
                arm2.setArmPosition(armServo2.getPosition() + 0.3);
            }

            // Claw stuff
            double wristStage0 = 0;
            double wristStage1 = 1;
            double fingerStage0 = 0;
            double fingerStage1 = 0;

            Claw claw = new Claw(wristServo, fingerServo, wristStage0, wristStage1, fingerStage0, fingerStage1)
            if (gamepad2.right_trigger > 0.03) {
                claw.setFinger("release");
                claw.setWrist("intake");
            } else {
                claw.setFinger("hold");
                claw.setWrist("score");
            }


            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("liftStage", lift1.getLiftStage());
            telemetry.addData("lift1Position", lift1.getLiftPosition());
            telemetry.addData("lift1Position", lift2.getLiftPosition());
            telemetry.addData("targetLiftPosition", liftPID.getTargetPosition());
            telemetry.addData("arm1Position", arm1.getArmPosition());
            telemetry.addData("arm2Position", arm2.getArmPosition());
            telemetry.addData("clawState", claw.getClawState());

            telemetry.update();
        }

    }
}






