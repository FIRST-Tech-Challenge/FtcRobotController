package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.CLAW_OPEN;
import static org.firstinspires.ftc.masters.BadgerConstants.TIP_CENTER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.drive.SampleMecanumDriveCancelable;

@Config
@TeleOp(name="Power Play TeleOp test odo", group = "competition")
public class PowerPlayTeleopDrive extends LinearOpMode {


    ArmPIDController armPIDController;
    LiftPIDController liftPIDController;

    SampleMecanumDriveCancelable drive;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDriveCancelable(hardwareMap, telemetry);

        armPIDController = new ArmPIDController(drive.armMotor);
        liftPIDController = new LiftPIDController(drive.linearSlide,drive.frontSlide, drive.slideOtherer);

        drive.openClaw();
        drive.tipCenter();
        int strafeConstant=1;
        double maxPowerConstraint = 0.75;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){


            double y = 0;
            double x = 0;
            double rx = 0;
            telemetry.addData("left y", gamepad1.left_stick_y);
            telemetry.addData("left x", gamepad1.left_stick_x);
            telemetry.addData("right x", gamepad1.right_stick_x);

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            if (Math.abs(y) < 0.2) {
                y = 0;
            }
            if (Math.abs(x) < 0.2) {
                x = 0;
            }

            double leftFrontPower = y + strafeConstant* x + rx;
            double leftRearPower = y - strafeConstant* x + rx;
            double rightFrontPower = y - strafeConstant* x - rx;
            double rightRearPower = y + strafeConstant*x - rx;

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                double max;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                max = Math.max(max, Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(rightRearPower));

                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            drive.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

        }


    }
}
