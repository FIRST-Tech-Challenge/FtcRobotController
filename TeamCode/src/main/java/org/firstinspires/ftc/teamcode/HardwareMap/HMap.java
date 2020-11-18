package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MISC.CONSTANTS;

public class HMap {

    // Members of the HardwareMap
    public DcMotor TL_ = null, TR_ = null, BL_ = null, BR_ = null;
    private DcMotor IntakeMotor_ = null;
    private DcMotor LauncherMotor_ = null;
    public BNO055IMU imu_;

    // PP extension for Hardware Devices
    public MotorPP TL, BL, TR, BR;
    public IMUPlus imu;

    // Instantiate them
    com.qualcomm.robotcore.hardware.HardwareMap hwMap =  null;
    public ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public HMap(){

    }

    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        TL_ = hwMap.get(DcMotor.class, "TL");
        TR_ = hwMap.get(DcMotor.class, "TR");
        BL_ = hwMap.get(DcMotor.class, "BL");
        BR_ = hwMap.get(DcMotor.class, "BR");


        TR_.setDirection(DcMotorSimple.Direction.REVERSE);
        BR_.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Encoder Stuff
        resetEncoders();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu_ = hwMap.get(BNO055IMU.class, "imu");
        imu_.initialize(parameters);

        // Initializing PP Hardware Classes
        TL = new MotorPP(new DcMotor[]{TL_}, CONSTANTS.dPid_, CONSTANTS.aPid_, CONSTANTS.vPid_);
        TR = new MotorPP(new DcMotor[]{TR_}, CONSTANTS.dPid_, CONSTANTS.aPid_, CONSTANTS.vPid_);
        BL = new MotorPP(new DcMotor[]{BL_}, CONSTANTS.dPid_, CONSTANTS.aPid_, CONSTANTS.vPid_);
        BR = new MotorPP(new DcMotor[]{BR_}, CONSTANTS.dPid_, CONSTANTS.aPid_, CONSTANTS.vPid_);
        imu = new IMUPlus(imu_, CONSTANTS.imuPid_);

        // Set zero power
        TL.setPower(0.0);
        BL.setPower(0.0);
        TR.setPower(0.0);
        BR.setPower(0.0);

        // Final Actions of Init
        runtime.reset();
    }

    public void resetEncoders(){
        TL_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TL_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TR_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
