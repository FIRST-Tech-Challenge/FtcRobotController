package org.firstinspires.team8923_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

abstract public class MasterOpMode extends LinearOpMode {
    //declare drive motors
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public Servo CarouselServo = null;
    public Servo linearServo = null;

    //declare imu
    public BNO055IMU imu;

    public void initHardware() {
        //init drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        CarouselServo = hardwareMap.servo.get("CarouselServo");
        linearServo = hardwareMap.servo.get("linearServo");

        //reset encoder
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //CarouselServo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //linearServo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //init imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void driveMecanum(double driveAngle, double drivePower, double turnPower) {
        // Calculate x and y components of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
        double x = drivePower * -Math.sin(Math.toRadians(driveAngle));
        double y = drivePower * Math.cos(Math.toRadians(driveAngle));

        double powerFL = x - y + turnPower;
        double powerFR = x + y + turnPower;
        double powerBL = -x + y + turnPower;
        double powerBR = -x - y + turnPower;

        // gets the largest power
        double scaleFactor = Math.max(Math.max(powerFL, powerFR), Math.max(powerBL, powerBR));
        // scale the power between the range of -1 and 1
        if (scaleFactor > 1) {
            powerFL /= scaleFactor;
            powerFR /= scaleFactor;
            powerBL /= scaleFactor;
            powerBR /= scaleFactor;
        }

        motorLeft.setPower(powerFL);
        motorRight.setPower(powerFR);
    }


    /*///if you subtract 359 from 0, you would get -359 instead of 1
    //this method handles cases when one angle is multiple rotations from the other
    public double adjustAngles(double firstAngle, double secondAngle)
    {
        double delta = firstAngle - secondAngle;
        while(delta > 180)
            delta -= 360;
        while(delta < -180)
            delta += 360;
        return delta;
    }*/
    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

}
