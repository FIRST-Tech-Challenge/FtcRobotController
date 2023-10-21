package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    // Big math thingy that creates the curve for our ramp
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

    Pose2d headingToHold = new Pose2d();
    boolean isHolding = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // Ramping for x and y motion
        Ewma statsX = new Ewma(0.3); // Raising alpha will make the ramp more drastic but more potentially create slip
        Ewma statsY = new Ewma(0.3); // Decreasing alpha will reduce slip
        PIDFController ctrl = new PIDFController(new PIDCoefficients(.01, 0, 0), 0, 0);
        ctrl.setInputBounds(0, 360);
        while (!isStopRequested()) {
            double turnCommand = -gamepad1.right_stick_x;


            if (!isHolding && Math.abs(turnCommand) < 0.03) {
                headingToHold = new Pose2d(0 , 0, Math.toRadians(drive.getExternalHeading()));
                isHolding = true;
            } else if (Math.abs(turnCommand) >= 0.03){
                isHolding = false;
            }

            if (isHolding) {
                if (gamepad1.x) {
                    headingToHold = new Pose2d(0, 0, Math.toRadians(90));
                } else if (gamepad1.a) {
                    headingToHold = new Pose2d(0, 0, Math.toRadians(180));
                } else if (gamepad1.b) {
                    headingToHold = new Pose2d(0, 0, Math.toRadians(-90));
                } else if (gamepad1.y) {
                    headingToHold = new Pose2d(0, 0, Math.toRadians(0));
                }
                double turnErrorDeg = Math.toDegrees(headingToHold.getHeading()) - drive.getExternalHeading();
                double target = (Math.toDegrees(headingToHold.getHeading()));
                target = target % 360.0;
                if (target < 0) {
                    target += 360.0;
                }
                ctrl.setTargetPosition(target);
                turnCommand = ctrl.update(Math.toDegrees(drive.getExternalHeading()));
            }

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

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("isHolding", isHolding);
            telemetry.update();
        }
    }
}
