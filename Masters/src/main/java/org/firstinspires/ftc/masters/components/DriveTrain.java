package org.firstinspires.ftc.masters.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.masters.components.Init;

import org.firstinspires.ftc.masters.old.CSCons;

public class DriveTrain implements Component{

    /*

             Y m
       |-------------|
      FL?           FR?
     |- -|         |- -|
     | \ |---------| / |     ---
     |- -|         |- -|      |                    ^
       |             |        |                    |  Vx
       |             |        |    X m         <---|      ↺ w
       |             |        |                  Vy
       |             |        |
     |- -|         |- -|      |   ---
     | / |---------| \ |     ---   |  D m
     |- -|         |- -|          ---
      BL?           BR?
     L = X/2
     W = Y/2
     R = D/2

     Vx and Vy are the measured velocities of the robot
     at full motor power
     */

    private double L = 0.177425;
    private double W = 0.156525;
    private double R = 0.047999;
    private double Vx = 0;
    private double Vy = 0;


    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;
    private HardwareMap hardwareMap = null;

    public enum RestrictTo {
        X,
        Y,
        T,
        XY,
        XT,
        YT,
        XYT

    }
    public DriveTrain (HardwareMap hardwareMap){

        this.hardwareMap=hardwareMap;

        initializeHardware();
    }
    public void initializeHardware() {
        // Read from the hardware maps
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        // Reset the encoder values
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        // Don't use the encoders for motor odometry
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Engage the brakes when the robot cuts off power to the motors
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

//    public double[] kinematics(double vx, double vy, double w) {
//        double t1 = (vx + vy - (L + W) * w)/R;
//        double t2 = (vx - vy + (L + W) * w)/R;
//        double t3 = (vx - vy - (L + W) * w)/R;
//        double t4 = (vx + vy + (L + W) * w)/R;
//    }

    // Drive using gamepad (THIS IS THE PREFERRED METHOD TO USE)
    public void drive(Gamepad gamepad) {
        cartesianDrive(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_trigger - gamepad.left_trigger);
    }

    // Drive, but restrict the axis
    public void drive(Gamepad gamepad, RestrictTo axis){
        switch(axis){
            case X:
                cartesianDrive(gamepad.left_stick_x, 0, 0);
                break;
            case Y:
                cartesianDrive(0, -gamepad.left_stick_y, 0);
                break;
            case T:
                cartesianDrive(0, 0, gamepad.right_trigger - gamepad.left_trigger);
                break;
            case XY:
                cartesianDrive(gamepad.left_stick_x, -gamepad.left_stick_y, 0);
                break;
            case XT:
                cartesianDrive(gamepad.left_stick_x, 0, gamepad.right_trigger - gamepad.left_trigger);
                break;
            case YT:
                cartesianDrive(0, -gamepad.left_stick_y, gamepad.right_trigger - gamepad.left_trigger);
                break;
            case XYT:
                cartesianDrive(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_trigger - gamepad.left_trigger);
                break;
        }

    }

    public void driveNoMultiplier(Gamepad gamepad, RestrictTo axis){
        switch(axis){
            case X:
                cartesianDriveNoMultiplier(gamepad.left_stick_x, 0, 0);
                break;
            case Y:
                cartesianDriveNoMultiplier(0, -gamepad.left_stick_y, 0);
                break;
            case T:
                cartesianDriveNoMultiplier(0, 0, gamepad.right_trigger - gamepad.left_trigger);
                break;
            case XY:
                cartesianDriveNoMultiplier(gamepad.left_stick_x, -gamepad.left_stick_y, 0);
                break;
            case XT:
                cartesianDriveNoMultiplier(gamepad.left_stick_x, 0, gamepad.right_trigger - gamepad.left_trigger);
                break;
            case YT:
                cartesianDriveNoMultiplier(0, -gamepad.left_stick_y, gamepad.right_trigger - gamepad.left_trigger);
                break;
            case XYT:
                cartesianDriveNoMultiplier(Math.pow(gamepad.left_stick_x, 3), -Math.pow(gamepad.left_stick_y, 3), Math.pow(gamepad.right_trigger - gamepad.left_trigger, 3));
                break;
        }

    }

    // Cartesian based driving
    public void cartesianDrive(double x, double y, double t) {

        //        float threshold = 0.1;
        //
        //        if (Math.abs(t) < threshold)
        //        {
        //            t = 0;
        //        }

        if (Math.abs(y) < 0.2) {
            y = 0;
        }
        if (Math.abs(x) < 0.2) {
            x = 0;
        }

        double leftFrontPower = y + x * CSCons.frontMultiplier + t;
        double leftRearPower = y - (x * CSCons.backMultiplier) + t;
        double rightFrontPower = y - x * CSCons.frontMultiplier - t;
        double rightRearPower = y + (x * CSCons.backMultiplier) - t;

        //if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1);
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightRearPower));

        leftFrontPower /= denominator;
        leftRearPower /= denominator;
        rightFrontPower /= denominator;
        rightRearPower /= denominator;
        //}

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    public void cartesianDriveNoMultiplier(double x, double y, double t) {

        //        float threshold = 0.1;
        //
        //        if (Math.abs(t) < threshold)
        //        {
        //            t = 0;
        //        }

        if (Math.abs(y) < 0.2) {
            y = 0;
        }
        if (Math.abs(x) < 0.2) {
            x = 0;
        }

        double leftFrontPower = y + x + t;
        double leftRearPower = y - x + t;
        double rightFrontPower = y - x - t;
        double rightRearPower = y + x - t;

        //if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1);
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightRearPower));

        leftFrontPower /= denominator;
        leftRearPower /= denominator;
        rightFrontPower /= denominator;
        rightRearPower /= denominator;
        //}

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }


    // Angle based driving
    public void polarDrive(double theta, double t){
        cartesianDrive(Math.cos(theta), Math.sin(theta), t);
    }
}