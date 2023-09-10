package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot extends Thread {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    //private static final int TICKS_PER_ROTATION = 1440; //Tetrix motor specific
    private static final double TICKS_PER_ROTATION = 537.7; //Gobilda 5203 312 RMP motor specific
    private static final double WHEEL_DIAMETER = 3.78; //Wheel diameter in inches
    private String TAG = "FTC";

    private DcMotorEx Motor_FL;
    private DcMotorEx Motor_FR;
    private DcMotorEx Motor_BR;
    private DcMotorEx Motor_BL;
    private BNO055IMU imu;

    private final int tollerance = 15;


    public boolean isTeleOp = true;
    private boolean DEBUG_DEBUG = true;
    private boolean DEBUG_INFO = true;

    Robot(HardwareMap map, Telemetry tel) {
        hardwareMap = map;
        telemetry = tel;
        initDevices();
    }

    /* Calculate Drivetrain PID cofficients */

    private final int motorFLMaxSpeed = 2780;
    double motorFLF = 32767 / (double) motorFLMaxSpeed;
    double motorFLP = 0.1 * motorFLF;
    double mototFLI = 0.1 * motorFLP;

    private final int motorFRMaxSpeed = 2800;
    double motorFRF = 32767 / (double) motorFRMaxSpeed;
    double motorFRP = 0.1 * motorFRF;
    double mototFRI = 0.1 * motorFRP;

    private final int motorBLMaxSpeed = 2760;
    double motorBLF = 32767 / (double) motorBLMaxSpeed;
    double motorBLP = 0.1 * motorBLF;
    double mototBLI = 0.1 * motorBLP;

    private final int motorBRMaxSpeed = 2720;
    double motorBRF = 32767 / (double) motorBRMaxSpeed;
    double motorBRP = 0.1 * motorBRF;
    double mototBRI = 0.1 * motorBRP;


    private void initDeviceCore() throws Exception {

        telemetry.addData("Please wait", "In function init devices");
        telemetry.update();

        //Wheels
        Motor_FL = hardwareMap.get(DcMotorEx.class, "motor_fl");
        Motor_FR = hardwareMap.get(DcMotorEx.class, "motor_fr");
        Motor_BR = hardwareMap.get(DcMotorEx.class, "motor_br");
        Motor_BL = hardwareMap.get(DcMotorEx.class, "motor_bl");

       /*Motor_FR.setVelocityPIDFCoefficients(0.95, 0.095, 0, 9.5);
        Motor_FR.setPositionPIDFCoefficients(5.0);

        Motor_FL.setVelocityPIDFCoefficients(0.95, 0.095, 0, 9.5);
        Motor_FL.setPositionPIDFCoefficients(5.0);

        Motor_BR.setVelocityPIDFCoefficients(0.93, 0.093, 0, 9.3);
        Motor_BR.setPositionPIDFCoefficients(5.0);

        Motor_BL.setVelocityPIDFCoefficients(0.93, 0.093, 0, 9.3);
        Motor_BL.setPositionPIDFCoefficients(5.0);*/

        Log.i(TAG, "Motor FR Cofficients: P: " + motorFRP + " I: " + mototFRI + " F: " + motorFRF);
        Log.i(TAG, "Motor FL Cofficients: P: " + motorFLP + " I: " + mototFLI + " F: " + motorFLF);
        Log.i(TAG, "Motor BR Cofficients: P: " + motorBRP + " I: " + mototBRI + " F: " + motorBRF);
        Log.i(TAG, "Motor BL Cofficients: P: " + motorBLP + " I: " + mototBLI + " F: " + motorBLF);

        Motor_FR.setVelocityPIDFCoefficients(motorFRP, mototFRI, 0, motorFRF);
        Motor_FR.setPositionPIDFCoefficients(8.2);

        Motor_FL.setVelocityPIDFCoefficients(motorFLP, mototFLI, 0, motorFLF);
        Motor_FL.setPositionPIDFCoefficients(8.0);

        Motor_BR.setVelocityPIDFCoefficients(motorBRP, mototBRI, 0, motorBRF);
        Motor_BR.setPositionPIDFCoefficients(8.2);

        Motor_BL.setVelocityPIDFCoefficients(motorBLP, mototBLI, 0, motorBLF);
        Motor_BL.setPositionPIDFCoefficients(8.0);

        Motor_FL.setTargetPositionTolerance(15);
        Motor_FR.setTargetPositionTolerance(15);
        Motor_BL.setTargetPositionTolerance(15);
        Motor_BR.setTargetPositionTolerance(15);


        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json";
        parametersIMU.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    private void initDevices() {
        ElapsedTime mRuntime;
        mRuntime = new ElapsedTime();
        mRuntime.reset();

        try {
            initDeviceCore();
        } catch (Exception e) {
            telemetry.addData("Exception", "In function init devices" + e);
            telemetry.update();
            try {
                sleep(10000);
            } catch (Exception e1) {
            }

        }

    }

    private void pause(int milliSec) {
        try {
            sleep(milliSec);
        } catch (Exception e) {
        }
    }

    // This function takes input distance in inches and will return Motor ticks needed
    // to travel that distance based on wheel diameter
    private int DistanceToTick(double distance) {
        // Log.i(TAG, "Enter FUNC: DistanceToTick");

        double circumference = WHEEL_DIAMETER * 3.14;
        double num_rotation = distance / circumference;
        int encoder_ticks = (int) (num_rotation * TICKS_PER_ROTATION);

        //       Log.i(TAG,"Rotation Needed : " + num_rotation);
        //if (DEBUG_INFO) {
        Log.i(TAG, "Ticks Needed : " + encoder_ticks);
        //  Log.i(TAG, "Exit FUNC: DistanceToTick");
        //}

        return (encoder_ticks);
    }


    boolean drivetrainBusy(int ticks) {
        int avg = (Math.abs(Motor_FL.getCurrentPosition())
                + Math.abs(Motor_FR.getCurrentPosition())
                + Math.abs(Motor_BL.getCurrentPosition())
                + Math.abs(Motor_BR.getCurrentPosition())) / 4;
        if ((avg >= (ticks - tollerance)) || (avg <= (ticks + tollerance))) {
            return false;
        }
        return true;
    }

    /*****************************************************************************/
    /* Section:      Move to specific distance functions                         */
    /*                                                                           */
    /* Purpose:    Used for moving motor specific inches                         */
    /*                                                                           */
    /* Returns:   Nothing                                                        */
    /*                                                                           */
    /* Params:    IN     power         - Speed  (-1 to 1)                        */
    /*            IN     distance      -  in inches                              */
    /*                                                                           */

    /*****************************************************************************/
    // Move forward to specific distance in inches, with power (0 to 1)
    public void moveForwardToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveForwardToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        long startTime = System.currentTimeMillis();

        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);

        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition((-1) * ticks);
        Motor_FR.setTargetPosition(ticks);
        Motor_BR.setTargetPosition(ticks);
        Motor_BL.setTargetPosition((-1) * ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if ((System.currentTimeMillis() - startTime) > 3000) {
                break;
            }
            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Forward", "Moving");
            telemetry.update();
        }


        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveForwardToPosition");
        }
    }
}

