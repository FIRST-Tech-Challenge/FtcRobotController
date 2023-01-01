package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Hardware {

    // Motor variable names
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public DcMotor rightSlides = null;
    public DcMotor leftSlides = null;

    public CRServo leftIntake = null;
    public CRServo rightIntake = null;

    public BNO055IMU imu = null;
    public Orientation lastAngles = null;
    public double globalAngle = 0.0;
    public boolean encoder;


    // Other variable names
    public HardwareMap hwMap;

    public Hardware(boolean encoder) {
        hwMap = null;
        this.encoder = encoder; //setting if you want to run with encoders
    }

    //called in initializeRobot method in AutonomousMethods
    public void initializeHardware(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;

        // Define Motors
        frontLeftMotor = hwMap.dcMotor.get("front_left");

        frontRightMotor = hwMap.dcMotor.get("front_right");

        backLeftMotor = hwMap.dcMotor.get("back_left");

        backRightMotor = hwMap.dcMotor.get("back_right");

        rightSlides = hwMap.dcMotor.get("rightSlides");

        leftSlides = hwMap.dcMotor.get("leftSlides");


        // Define Servos
        leftIntake = hwMap.crservo.get("leftIntake");

        rightIntake = hwMap.crservo.get("rightIntake");

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing
        //Drivetrain Motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Originally Reverse
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);//Originally Forward
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); //Originally Reverse
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);//Originally Forward

        //Subsystem Motors & Servos
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setDirection(DcMotorSimple.Direction.FORWARD);

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        lastAngles = new Orientation();

        if(encoder) {
            // May use RUN_USING_ENCODERS if encoders are installed
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else{
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Setting Motor/Servo Powers
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }
    public void sleep(double time){
        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds()<(time/1000)){

        }

    }
}