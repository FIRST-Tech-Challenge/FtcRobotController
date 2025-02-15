package org.firstinspires.ftc.teamcode.BBcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.RedBasketPose;
import org.firstinspires.ftc.teamcode.autoshellclasses.Red_Basket_Auto;

import java.util.Arrays;

public class MecanumDrivetrain {
    OpMode _opMode;
    TelemetryHelper _telemetryHelper;
    DcMotorEx _leftFront;
    DcMotorEx _leftBack;
    DcMotorEx _rightFront;
    DcMotorEx _rightBack;
    GoBildaPinpointDriverRR pinpoint;
    // Proportional control gains (tweak these values during testing)
    private static Pose2d previousPose = new Pose2d(0, 0, 0);
    //private static final Pose2d testTargetPose = new Pose2d(-39.5, -43.5, 0);
    //private static final Pose2d targetPose = new Pose2d(-53.9, -53.275, Math.toRadians(45));
    //TODO drop and target pose needs to be set based on start location red vs blue
    private static final Pose2d dropPose = RedBasketPose.drop;
    private static final Pose2d targetPose = new Pose2d(dropPose.position.x+1.5, dropPose.position.y+1.5, dropPose.heading.toDouble());
    private static final double kpTranslation = 0.07;
    private static final double kpRotation = .7;
    private static final double angleToleranceDeg = 1;
    private static final double distanceToleranceInch = .25;
    // Constructor
    public MecanumDrivetrain(OpMode opMode) {
        _opMode = opMode;
        _telemetryHelper = new TelemetryHelper(opMode);
        // Initialize the motors
        _leftFront = _opMode.hardwareMap.tryGet(DcMotorEx.class, "leftFront");
        _leftBack = _opMode.hardwareMap.tryGet(DcMotorEx.class, "leftBack");
        _rightFront = _opMode.hardwareMap.tryGet(DcMotorEx.class, "rightFront");
        _rightBack = _opMode.hardwareMap.tryGet(DcMotorEx.class, "rightBack");
        pinpoint = _opMode.hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
        double[] motorPowers = new double[]{0, 0, 0, 0};
        //For right now, just add a telemetry message but the code will still fail when it's accessed in code so gracefully handle the null case
        //This could be to exit the OpMode or to continue with the OpMode but not use the device. The latter requires checking for null in the code
        if (_leftFront == null || _leftBack == null || _rightFront == null || _rightBack == null)
        {
            _opMode.telemetry.addLine("One or more motors not found!");
        } else {
            // Reverse the right side motors
            _rightFront.setDirection(DcMotor.Direction.REVERSE);
            _rightBack.setDirection(DcMotor.Direction.REVERSE);
        }

        _telemetryHelper.initMotorTelemetry( _leftFront, "LF");
        _telemetryHelper.initMotorTelemetry( _leftBack, "LB");
        _telemetryHelper.initMotorTelemetry( _rightFront, "RF");
        _telemetryHelper.initMotorTelemetry( _rightBack, "RB");
        opMode.telemetry.addData("Error X", ()->getErrorX(pinpoint.getPositionRR()));
        opMode.telemetry.addData("Error Y", ()->getErrorY(pinpoint.getPositionRR()));
        opMode.telemetry.addData("Error Yaw", ()->getErrorYaw(pinpoint.getPositionRR()));
        opMode.telemetry.addData("LF", ()-> calMotorPowers(previousPose, targetPose)[0]);
        opMode.telemetry.addData("RF", ()-> calMotorPowers(previousPose, targetPose)[1]);
        opMode.telemetry.addData("LB", ()-> calMotorPowers(previousPose, targetPose)[2]);
        opMode.telemetry.addData("RB", ()-> calMotorPowers(previousPose, targetPose)[3]);

    }
    private double getErrorX(Pose2d currentPose) {
        if (currentPose == null) {
            return 0;
        }

        //get error from pinpoint stuff
        double errorX = targetPose.position.x - currentPose.position.x;
        return errorX;
    }
    private double getErrorY(Pose2d currentPose) {
        if (currentPose == null) {
            return 0;
        }

        //get error from pinpoint stuff
        double errorY = targetPose.position.y - currentPose.position.y;
        return errorY;
    }
    private double getErrorYaw(Pose2d currentPose) {
        if (currentPose == null) {
            return 0;
        }

        //get error from pinpoint stuff
        double errorYaw = Math.toDegrees(targetPose.heading.toDouble()) - Math.toDegrees(currentPose.heading.toDouble());
        return errorYaw;
    }
    public void Drive() {
        double drive;
        double turn;
        double strafe;
        double fLeftPow, fRightPow, bLeftPow, bRightPow;
        double speedMultiplier = 0.75;
        double turnSpeedMultiplier = 0.5;
        double turnEasingExponet = 3, turnEasingYIntercept = 0.05;

        Gamepad gamepad1 = _opMode.gamepad1;
        if (gamepad1.right_bumper) {
            previousPose = pinpoint.getPositionRR();

            double[] motorPowers = calMotorPowers(previousPose, targetPose);
            setMotorPowers(motorPowers);

        } else {
            //drive inputs
            drive = gamepad1.left_stick_y;
            turn = turnSpeedMultiplier * (Math.pow((gamepad1.right_stick_x * -1), turnEasingExponet) + (Math.signum(gamepad1.right_stick_x * -1) * turnEasingYIntercept));
            strafe = gamepad1.left_stick_x * -1;
            if (gamepad1.left_trigger > 0) {
                speedMultiplier = 0.25;
            }
            fLeftPow = Range.clip((drive + turn + strafe) * speedMultiplier, -1, 1);
            bLeftPow = Range.clip((drive + turn - strafe) * speedMultiplier, -1, 1);
            fRightPow = Range.clip((drive - turn - strafe) * speedMultiplier, -1, 1);
            bRightPow = Range.clip((drive - turn + strafe) * speedMultiplier, -1, 1);

            if (_leftFront != null) {
                _leftFront.setPower(fLeftPow);
            }
            if (_leftBack != null) {
                _leftBack.setPower(bLeftPow);
            }
            if (_rightFront != null) {
                _rightFront.setPower(fRightPow);
            }
            if (_rightBack != null) {
                _rightBack.setPower(bRightPow);
            }
        }
    }
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
//    public void moveRobot1(double x, double y, double yaw) {
//        // Calculate wheel powers.
//        double leftFrontPower    =  x -y -yaw;
//        double rightFrontPower   =  x +y +yaw;
//        double leftBackPower     =  x +y -yaw;
//        double rightBackPower    =  x -y +yaw;
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Send powers to the wheels.
//        _leftFront.setPower(leftFrontPower);
//        _rightFront.setPower(rightFrontPower);
//        _leftBack.setPower(leftBackPower);
//        _rightBack.setPower(rightBackPower);
//    }
    private void setMotorPowers(double[] powers) {


        // Send powers to the wheels.
        _leftFront.setPower(powers[0]);
        _rightFront.setPower(powers[1]);
        _leftBack.setPower(powers[2]);
        _rightBack.setPower(powers[3]);
    }
    private double[] calMotorPowers(Pose2d currentPose, Pose2d targetPose) {
        //get error from pinpoint stuff
        double errorX = targetPose.position.x - currentPose.position.x;
        double errorY = targetPose.position.y - currentPose.position.y;
        double distanceError = Math.hypot(errorX, errorY);
        double maxDrive = 1;
        double targetSlowDownDistance = 8;
        double minDrive = 0.30;
        /*
         * For a mecanum drive, we want to translate the field-centric error vector into
         * robot–centric coordinates. To do this, rotate the error vector by the negative
         * of the robot’s current heading.
         */
        double robotHeading = currentPose.heading.toDouble();
        double robotRelativeX = errorX * Math.cos(robotHeading) + errorY * Math.sin(robotHeading);
        double robotRelativeY = -errorX * Math.sin(robotHeading) + errorY * Math.cos(robotHeading);
        robotRelativeY = -robotRelativeY;
        double errorYaw = targetPose.heading.toDouble() - robotHeading;
        if (distanceError < distanceToleranceInch && Math.abs(Math.toDegrees(errorYaw)) < angleToleranceDeg) {
            return new double[]{0, 0, 0, 0};
        } else {

            double drive = robotRelativeX * kpTranslation;
            double strafe = robotRelativeY * kpTranslation;
            double turn = errorYaw * kpRotation;
            // Calculate wheel powers.
//        double leftFrontPower    =  x - y - yaw;
//        double rightFrontPower   =  x + y + yaw;
//        double leftBackPower     =  x + y - yaw;
//        double rightBackPower    =  x - y + yaw;

            // Calculate individual motor powers for mecanum drive
            double leftFrontPower = strafe + drive - turn;
            double rightFrontPower = strafe - drive - turn;
            double leftBackPower = strafe - drive + turn;
            double rightBackPower = strafe + drive + turn;

            leftFrontPower = -leftFrontPower;
            rightBackPower = -rightBackPower;
            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (distanceError < targetSlowDownDistance) {
                maxDrive = distanceError / targetSlowDownDistance;
                maxDrive = Math.max(maxDrive, minDrive);
            } else {
                maxDrive = 1;
            }
            if (max > maxDrive) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Return the motor powers.
            return new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
        }
    }
}
