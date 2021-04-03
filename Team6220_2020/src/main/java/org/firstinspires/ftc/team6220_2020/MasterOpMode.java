package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team6220_2020.ResourceClasses.DriverInput;

public abstract class MasterOpMode extends LinearOpMode
{
    //Motors
    public static DcMotor motorFrontLeft;
    public static DcMotor motorFrontRight;
    public static DcMotor motorBackLeft;
    public static DcMotor motorBackRight;
    // Todo - move to miscellaneous motors.
    // Make sure to declare the 3.7 launch motor as a 20 on the control hub
    public static DcMotor motorLauncher;

    //Other Devices
    public static Servo servoLauncher;

    // Create drivers
    public DriverInput driver1;
    public DriverInput driver2;

    //Booleans
    public boolean isSlowMode = false;

    //
    public BNO055IMU imu;

    //This method initializes the motors.
    public void Initialize(){
        // Drive train motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        motorBackLeft = hardwareMap.dcMotor.get("motorBL");
        motorBackRight = hardwareMap.dcMotor.get("motorBR");
        // Todo - move to miscellaneous motors.
        motorLauncher = hardwareMap.dcMotor.get("motorLauncher");

        //Servos
        servoLauncher = hardwareMap.servo.get("servoLauncher");


        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Todo - move to miscellaneous motors.
        motorLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //This method drives mecanum when given an angle drive power and turning power
    public void driveMecanum(double driveAngle, double drivePower, double turningPower)
    {

        if(isSlowMode){
            drivePower *= 0.5;
            turningPower *= 0.5;
        }

        double x = drivePower * Math.cos(driveAngle);
        double y = drivePower * Math.sin(driveAngle);

        double motorFLPower = x + y + -turningPower;
        double motorFRPower = x + -y + -turningPower;
        double motorBLPower = -x + y + -turningPower;
        double motorBRPower = -x + -y + -turningPower;

        double scaleFactor = Math.max(Math.max(motorFLPower, motorFRPower), Math.max(motorBLPower, motorBRPower));

        if(scaleFactor > 1){
            motorFrontLeft.setPower(motorFLPower / scaleFactor);
            motorFrontRight.setPower(motorFRPower / scaleFactor);
            motorBackLeft.setPower(motorBLPower / scaleFactor);
            motorBackRight.setPower(motorBRPower / scaleFactor);
        } else {
            motorFrontLeft.setPower(motorFLPower);
            motorFrontRight.setPower(motorFRPower);
            motorBackLeft.setPower(motorBLPower);
            motorBackRight.setPower(motorBRPower);
        }
    }

    //Sets the launch motor to a given power
    public void driveLauncher(double power){
        motorLauncher.setPower(power);
    }

    //This method returns the speed of a given motor after a delay of delayInMillis
    //@param motor Input the motor you want to know the RPM of
    //@param delayInMillis Input the delay you want to measure the change in encoder ticks in milliseconds.
    public double getMotorTicksPerMinute(DcMotor motor, int delayInMillis) {

        //Variables used only in this method
        double startTime = System.currentTimeMillis();
        double startPosition = motor.getCurrentPosition();
        double endTime;
        double endPosition;
        double positionChange;
        double timeChange;

        //Waits delayInMillis milliseconds before recording endTime and endPosition
        while (true) {
            if (System.currentTimeMillis() - startTime >= delayInMillis) break;
        }

        endTime = System.currentTimeMillis();
        endPosition = motor.getCurrentPosition();

        //Calculates the ▲Position and the ▲Time
        positionChange = endPosition - startPosition;
        timeChange = endTime - startTime;

        //Converts the ▲Time from milliseconds to minutes then finds encoder ticks per minute
        double timeChangeInMin = timeChange / Constants.MILLIS_TO_MIN;

        //To avoid divide by zero we need to be sure timeChangeInMin does not equal zero.
        double ticksPerMinute = 0;
        if(timeChangeInMin != 0) {
            ticksPerMinute = positionChange / timeChangeInMin;
        }

        //If timeChange is not zero return the motor RPM otherwise return zero.
        return (ticksPerMinute);

    }

    public void fireLauncher(boolean checkSpeed)
    {
        if (checkSpeed) {
            double launcherRPM = getMotorTicksPerMinute(motorLauncher, 100) / Constants.AM_37_TICKS_PER_ROTATION;
            if (launcherRPM >= Constants.MINIMUM_LAUNCHER_SPEED) {
                servoLauncher.setPosition(Constants.SERVO_LAUNCH_FIRE);
                pauseMillis(100);
                servoLauncher.setPosition(Constants.SERVO_LAUNCH_REST);
            }
        } else {
            servoLauncher.setPosition(Constants.SERVO_LAUNCH_FIRE);
            pauseMillis(100);
            servoLauncher.setPosition(Constants.SERVO_LAUNCH_REST);
        }

    }

    //Pauses for time milliseconds
    public void pauseMillis(double time)
    {
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < time && opModeIsActive()){
            idle();
        }
    }

}