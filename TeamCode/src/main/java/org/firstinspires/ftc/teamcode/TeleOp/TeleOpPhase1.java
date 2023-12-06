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
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class TeleOpPhase1 extends LinearOpMode {
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

    final int kMAX_LIFT_POS = 600;
    final int kMIN_LIFT_POS = 0;
    final double kLIFT_POWER = 0.3;
    final double kLIFT_HOLDING_POWER = 0.05;

    Pose2d headingToHold = new Pose2d();
    boolean isHolding = false;

    private void snapToButtons(double externalHeading) {
        double relative = 0; // Radians
        if (gamepad1.start) {
            relative = externalHeading; // If the start button is pressed, everything will be relative to the current heading
        }

        if (gamepad1.x) {
            headingToHold = new Pose2d(0, 0, Math.toRadians(90) + relative);
        } else if (gamepad1.a) {
            headingToHold = new Pose2d(0, 0, Math.toRadians(180) + relative);
        } else if (gamepad1.b) {
            headingToHold = new Pose2d(0, 0, Math.toRadians(-90) + relative);
        } else if (gamepad1.y) {
            headingToHold = new Pose2d(0, 0, Math.toRadians(0) + relative);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//sets encoder position to 0 at init

        waitForStart();

        if (isStopRequested()) return;

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //running with set power [-1,1]

        // Ramping for x and y translation
        Ewma statsX = new Ewma(0.3); // Raising alpha will make the ramp more drastic but more potentially create slip
        Ewma statsY = new Ewma(0.3); // Decreasing alpha will reduce slip

        // PID controller for heading
        PIDFController headingPID = new PIDFController(new PIDCoefficients(0.019, 0, 0.001), 0, 0);
        headingPID.setInputBounds(0, 360);

        double liftPower = 0.0;

        while (!isStopRequested()) {
            double liftPos = liftMotor.getCurrentPosition();

            double turnCommand = gamepad1.right_stick_x;

            // This if statement essentially toggles holdHeading if a button is pressed and no joystick motion is detected
            if ((gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) && Math.abs(turnCommand) <= 0.03 ) {
                isHolding = true;
                snapToButtons(drive.getExternalHeading()); // Josh recommended that we make a separate method for just the a/b/x/y buttons
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


            if(gamepad1.dpad_up && liftPos < kMAX_LIFT_POS){
                liftPower = kLIFT_POWER;
            } else if (gamepad1.dpad_down && liftPos > kMIN_LIFT_POS){
                liftPower = -kLIFT_POWER;
            } else if(liftPos > kMIN_LIFT_POS) {
                liftPower = kLIFT_HOLDING_POWER;
            } else {
                liftPower = 0;
            }

            liftMotor.setPower(liftPower);

            // Here were any values that need to be broadcast on the drive station are declare
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("headingToHold", Math.toDegrees(headingToHold.getHeading()));
            telemetry.addData("isHolding", isHolding);
            telemetry.addData("joystick", gamepad1.right_stick_x);
            telemetry.addData("lift motor pos", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}