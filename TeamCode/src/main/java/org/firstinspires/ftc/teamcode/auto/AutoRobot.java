package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.*;

import static java.lang.Math.*;


public class AutoRobot {
    private DcMotor backRightDrive;
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    // private Servo outtakeAngle; needs to be implemented
    // private Servo outtakeClaw; needs to be implemented

    //private Servo intakeAngle; needs to be implemented
    //private Servo intakeClaw; needs to be implemented

    //private Servo intakeSlide1; needs to be implemented
    //private Servo intakeSlide2; needs to be implemented

    // private DcMotor elavator1; needs to be implemented
    // private DcMotor elavator2; needs to be implemented

    private IMU imu;


    private static final double WHEEL_DIAMETER = 48; // In milimeters
    private static final double TICKS_PER_REVOLUTION = 1120;

    Telemetry telemetry;
    //private double current robotX; unable to reliably solve
    //private double current robotY;


    //this function will move the robot x distance and y distance, and make it face the direction
    //currently does not work correctly
    public void driveRelative(double direction, double x, double y) {


        long time = (long) (1000000000 * sqrt(x * x + y * y));
        long currentTime = System.nanoTime();
        long totalTime = currentTime + time;
        double timeAlotted;


        double rX = 1;
        final double MULTIPLIER = .01; //adjust this value to make the robot move less or more,
        // smaller values make it move farther
        double moveSpeed = 0;
        double currentYaw;
        double angleDifference = 180;

        if (Math.abs(x) > Math.abs(y) && x != 0) {
            x = Math.signum(x);
            y = y / Math.abs(x);
        }
        if (Math.abs(y) > Math.abs(x) && y != 0) {
            x = x / Math.abs(y);
            y = Math.signum(y);
        }

        Vector2D toGo = new Vector2D(x, y);
        if (direction > 180) {
            direction -= 360;
        }


        do {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            currentTime = System.nanoTime();


            angleDifference = currentYaw - direction;

            if (angleDifference > 1 || angleDifference < -1) {


                angleDifference = currentYaw - direction;

                if (angleDifference > 180) {
                    angleDifference -= 360;
                }

                if (angleDifference < -180) {
                    angleDifference += 360;
                }

                if (angleDifference > 0) {
                    rX = .3 * MovementCurves.circleArc(angleDifference / 360);
                }
                ;
                if (angleDifference < 0) {
                    angleDifference *= -1;
                    rX = -.3 * MovementCurves.circleArc(angleDifference / 360);
                }
                ;


                if (rX > 0 && rX < .1) {
                    rX = .1;
                }
                if (rX < 0 && rX > -.1) {
                    rX = -.1;
                }

            } else {
                rX = 0;
            }


            if (currentTime < totalTime) {
                timeAlotted = (totalTime - currentTime) / ((double) (time));
                toGo.setVector(x, y);
                toGo.adjustAngle(currentYaw);
                moveSpeed = .3 * Math.sin(Math.PI * (timeAlotted));
                toGo.scaleVector(moveSpeed);
            } else {
                toGo.scaleVector(0);
            }

            frontRightDrive.setPower(toGo.getJ() - toGo.getI() - rX); //double check these values
            frontLeftDrive.setPower(toGo.getJ() + toGo.getI() + rX);
            backLeftDrive.setPower(toGo.getJ() - toGo.getI() + rX);
            backRightDrive.setPower(toGo.getJ() + toGo.getI() - rX);


            telemetry.addData("toGoI", toGo.getI());
            telemetry.addData("toGoJ", toGo.getJ());
            telemetry.addData("toGoAngle", toGo.getAngle());
            telemetry.addData("target", direction);
            telemetry.addData("current", currentYaw);
            telemetry.addData("power", rX);
            telemetry.addData("moveSpeed", moveSpeed);
            telemetry.addData("angleDifference", angleDifference);
            telemetry.update();


        } while (rX != 0 || currentTime < totalTime);

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }

    //drives forward for a certain amount of seconds
    public void driveForward(double seconds) {
        long time = (long) (1000000000 * seconds);
        long currentTime = System.nanoTime();
        long totalTime = currentTime + time;
        double timeAlotted;
        double power;


        while (currentTime < totalTime) {
            timeAlotted = (totalTime - currentTime) / ((double) (time));
            power = Math.sin(Math.PI * (timeAlotted));

            frontRightDrive.setPower(power); //double check these values
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            backRightDrive.setPower(power);
            telemetry.addData("power", power);
            telemetry.update();
            currentTime = System.nanoTime();
        }
        frontRightDrive.setPower(0); //double check these values
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    //drives backwards for a certain amount of seconds

    public void driveBackward(double seconds) {
        long time = (long) (1000000000 * seconds);
        long currentTime = System.nanoTime();
        long totalTime = currentTime + time;
        double timeAlotted;
        double power;
        while (currentTime < totalTime) {
            timeAlotted = (totalTime - currentTime) / ((double) (time));

            power = Math.sin(Math.PI * (timeAlotted));
            frontRightDrive.setPower(-power); //double check these values
            frontLeftDrive.setPower(-power);
            backLeftDrive.setPower(-power);
            backRightDrive.setPower(-power);
            telemetry.addData("power", power);
            telemetry.update();
            currentTime = System.nanoTime();
        }
        frontRightDrive.setPower(0); //double check these values
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void driveRight(double seconds) {
        long time = (long) (1000000000 * seconds);
        long currentTime = System.nanoTime();
        long totalTime = currentTime + time;
        double timeAlotted;
        double power;
        while (currentTime < totalTime) {
            timeAlotted = (totalTime - currentTime) / ((double) (time));

            power = Math.sin(Math.PI * (timeAlotted));
            frontRightDrive.setPower(-power); //double check these values
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(-power);
            backRightDrive.setPower(power);
            telemetry.addData("power", power);
            telemetry.update();
            currentTime = System.nanoTime();
        }
        frontRightDrive.setPower(0); //double check these values
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    public void driveLeft(double seconds) {
        long time = (long) (1000000000 * seconds);
        long currentTime = System.nanoTime();
        long totalTime = currentTime + time;
        double timeAlotted;
        double power;
        while (currentTime < totalTime) {
            timeAlotted = (totalTime - currentTime) / ((double) (time));

            power = Math.sin(Math.PI * (timeAlotted));
            frontRightDrive.setPower(power); //double check these values
            frontLeftDrive.setPower(-power);
            backLeftDrive.setPower(power);
            backRightDrive.setPower(-power);
            telemetry.addData("power", power);
            telemetry.update();
            currentTime = System.nanoTime();
        }
        frontRightDrive.setPower(0); //double check these values
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    //makes the robot face a specific direction relative to the starting position
    //0 is facing initial direction

    public void face(double direction) {

        //forward 0
        //right -90, 270
        //backwards 180, -180
        //left 90

        //may be dependent on hub position

        double currentYaw;
        double angleDifference = 180;
        double rX = 1;

        if (direction > 180) {
            direction -= 360;
        }
        while (angleDifference > .5 || angleDifference < -.5) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            angleDifference = currentYaw - direction;

            if (angleDifference > 180) {
                angleDifference -= 360;
            }

            if (angleDifference < -180) {
                angleDifference += 360;
            }

            if (angleDifference > 0) {
                rX = Math.sqrt(180 * 180 - Math.pow((180 - angleDifference), 2)) / 180;
            }

            if (angleDifference < 0) {
                rX = -Math.sqrt(180 * 180 - Math.pow((-180 - angleDifference), 2)) / 180;
            }



            if (rX > 0 && rX < .05) {
                rX = .05;
            }
            if (rX < 0 && rX > -.05) {
                rX = -.05;
            }


            telemetry.addData("target", direction);
            telemetry.addData("current", currentYaw);
            telemetry.addData("power", rX);
            telemetry.addData("angleDifference", angleDifference);
            telemetry.update();


            frontRightDrive.setPower(-rX);
            frontLeftDrive.setPower(rX);
            backLeftDrive.setPower(rX);
            backRightDrive.setPower(-rX);

        }

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);


    }


    public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the hardware devices
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);


        // outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        //outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        //intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        //intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        //intakeSlide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        //intakeSlide2 = hardwareMap.get(Servo.class, "intakeSlide2");

        //elavator1 = hardwareMap.get(DcMotor.class, "elavator1");
        //elavator2 = hardwareMap.get(DcMotor.class, "elavator2");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();


        this.telemetry = telemetry;

    }


}
