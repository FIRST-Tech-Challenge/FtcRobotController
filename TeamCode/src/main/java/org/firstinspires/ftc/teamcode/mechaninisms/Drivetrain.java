package org.firstinspires.ftc.teamcode.mechaninisms;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotVars;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class Drivetrain {
    private HardwareMap hardwareMap;
    public Follower follower;

    RobotStateHandler robotStateHandler;

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public Drivetrain(HardwareMap hardwareMapRef, RobotVars robotVars) {
        hardwareMap = hardwareMapRef;

        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftBack = hardwareMap.get(DcMotorEx.class, "lr");
        rightBack = hardwareMap.get(DcMotorEx.class, "rr");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        follower = Constants.create(hardwareMap);
        follower.setPose(robotVars.startPose);

    }


    public void runDriveCode(float left_stick_x, float left_stick_y, float right_stick_x, boolean isHiveCentric) {

        if (isHiveCentric) {
            runHiveCentricDriveCode(
                left_stick_x,
                left_stick_y
            );
        } else {
            runRobotCentricDriveCode(
                left_stick_x,
                left_stick_y,
                right_stick_x
            );
        }

    }


    public void runRobotCentricDriveCode(float left_stick_x, float left_stick_y, float right_stick_x) {

        // Get user values
        double strafe = left_stick_x;
        double drive = left_stick_y;
        double rotation = right_stick_x;

        // Calculate powers
        double frontLeftPower = drive - rotation - strafe;
        double frontRightPower = drive + rotation + strafe;
        double backLeftPower = drive - rotation + strafe;
        double backRightPower = drive + rotation - strafe;

        // Send calculated power to wheels
        setPowers(frontLeftPower , frontRightPower, backLeftPower,backRightPower );
    }


    public void runHiveCentricDriveCode(float left_stick_x, float left_stick_y) {

        // Get user values
        double strafe = left_stick_x;
        double drive = left_stick_y;
        double rotation = RobotStateHandler.limelight.directionToHive();

        // Calculate powers
        double frontLeftPower = drive - rotation - strafe;
        double frontRightPower = drive + rotation + strafe;
        double backLeftPower = drive - rotation + strafe;
        double backRightPower = drive + rotation - strafe;

        // Send calculated power to wheels
        setPowers(frontLeftPower , frontRightPower, backLeftPower,backRightPower );
    }





    public void runFieldCentricDrive(float left_stick_x, float left_stick_y, float right_stick_x, boolean faceHiveButton) {
        follower.update();

        // Figure out the follower
        double yaw = follower.pose().heading();

        // Get user values
        double x = left_stick_x;
        double y = left_stick_y;
        double rot = 0 - right_stick_x;

        // Results after matrix multiplication
        double newX = (x * Math.cos(yaw)) - (y * Math.sin(yaw));
        double newY = (y * Math.cos(yaw)) + (x * Math.sin(yaw));


        if (faceHiveButton) {
            rot = 0 - RobotStateHandler.limelight.directionToHive();
        }

        // Calculate powers
        double frontLeftPower = newY - rot - newX;
        double frontRightPower = newY + rot + newX;
        double backLeftPower = newY - rot + newX;
        double backRightPower = newY + rot - newX;

        setPowers(frontLeftPower, frontRightPower, backLeftPower,backRightPower);

    }


    public void setPowers(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

}
