package org.firstinspires.ftc.team8923_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

abstract public class MasterOpMode extends LinearOpMode {
    //declare drive motors
    public DcMotor motorFL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBL = null;
    public DcMotor motorBR = null;

    //declare misc motors
    public DcMotor motorIntake = null;
    public DcMotor motorLift = null;
    public DcMotor motorShooter = null;
    public DcMotor motorWobble = null;
    public DcMotor motorDankUnderglow = null;

    //declare servos
    public Servo servoFlicker = null;
    public Servo servoGrabber = null;

    //declare imu
    public BNO055IMU imu;

    public void initHardware(){
        //init drive motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        //reset encoder
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //init misc motors
        motorIntake =hardwareMap.dcMotor.get("motorIntake");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        motorShooter = hardwareMap.dcMotor.get("motorShooter");
        motorWobble = hardwareMap.dcMotor.get("motorWobble");

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //intake motor doesn't really need encoders
        motorLift.setTargetPosition(motorLift.getCurrentPosition());
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //init servos
        servoFlicker = hardwareMap.get(Servo.class, "servoFlicker");
        //servoGrabber = hardwareMap.get(Servo.class, "servoGrabber");

        //start positions
        servoFlicker.setPosition(0.55);
        servoGrabber.setPosition(0.0);

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

    public void driveMecanum(double driveAngle, double drivePower, double turnPower){
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

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    //if you subtract 359 from 0, you would get -359 instead of 1
    //this method handles cases when one angle is multiple rotations from the other
    public double adjustAngles(double firstAngle, double secondAngle)
    {
        double delta = firstAngle - secondAngle;
        while(delta > 180)
            delta -= 360;
        while(delta < -180)
            delta += 360;
        return delta;
    }

    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

}

