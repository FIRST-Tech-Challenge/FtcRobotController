package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.DriverInput;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.PIDFilter;

public abstract class MasterOpMode extends LinearOpMode {
    // Motors
    public static DcMotor motorFrontLeft;
    public static DcMotor motorFrontRight;
    public static DcMotor motorBackLeft;
    public static DcMotor motorBackRight;
    // Todo - move to miscellaneous motors.
    // Make sure to declare the 3.7 launch motor as a 20 on the control hub
    public static DcMotor motorLauncher;
    public static DcMotor motorBelt;
    public static DcMotor motorZiptie;

    // Other Devices
    public static Servo servoLauncher;

    // Create drivers
    public DriverInput driver1;
    public DriverInput driver2;

    // Booleans
    public boolean isSlowMode = false;

    // IMUs
    public BNO055IMU imu;

    // This method initializes the motors.
    public void Initialize() {
        // Drive train motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        motorBackLeft = hardwareMap.dcMotor.get("motorBL");
        motorBackRight = hardwareMap.dcMotor.get("motorBR");
        // Todo - move to miscellaneous motors.
        motorLauncher = hardwareMap.dcMotor.get("motorLauncher");
        motorBelt = hardwareMap.dcMotor.get("motorBelt");
        motorZiptie = hardwareMap.dcMotor.get("motorZiptie");

        // Servos
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
        motorBelt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorZiptie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    // This method drives mecanum when given an angle drive power and turning power
    public void driveMecanum(double driveAngle, double drivePower, double turningPower) {
        double x = drivePower * Math.cos(driveAngle);
        double y = drivePower * Math.sin(driveAngle);

        double motorFLPower = x + y + -turningPower;
        double motorFRPower = x + -y + -turningPower;
        double motorBLPower = -x + y + -turningPower;
        double motorBRPower = -x + -y + -turningPower;

        double scaleFactor = Math.max(Math.max(motorFLPower, motorFRPower), Math.max(motorBLPower, motorBRPower));

        if (scaleFactor > 1) {
            motorFrontLeft.setPower(motorFLPower / scaleFactor);
            motorFrontRight.setPower(motorFRPower / scaleFactor);
            motorBackLeft.setPower(motorBLPower / scaleFactor);
            motorBackRight.setPower(motorBRPower / scaleFactor);
        }
        else {
            motorFrontLeft.setPower(motorFLPower);
            motorFrontRight.setPower(motorFRPower);
            motorBackLeft.setPower(motorBLPower);
            motorBackRight.setPower(motorBRPower);
        }
    }

    // Sets the launch motor to a given power
    public void driveLauncher(double power) {
        motorLauncher.setPower(power);
    }

    // Sets the belt motor to a given power
    public void driveBelt(double power) {
        motorBelt.setPower(power);
    }

    // Sets the ziptie motor to a given power
    public void driveZiptie(double power) {
        motorZiptie.setPower(power);
    }

    // This method returns the speed of a given motor after a delay of delayInMillis
    // @param motor Input the motor you want to know the RPM of
    // @param delayInMillis Input the delay you want to measure the change in encoder ticks in milliseconds.
    public double getMotorTicksPerMinute(DcMotor motor, int delayInMillis) {
        // Variables used only in this method
        double startTime = System.currentTimeMillis();
        double startPosition = motor.getCurrentPosition();
        double endTime;
        double endPosition;
        double positionChange;
        double timeChange;

        // Waits delayInMillis milliseconds before recording endTime and endPosition
        while (true) {
            if (System.currentTimeMillis() - startTime >= delayInMillis) break;
        }

        endTime = System.currentTimeMillis();
        endPosition = motor.getCurrentPosition();

        // Calculates the ▲Position and the ▲Time
        positionChange = endPosition - startPosition;
        timeChange = endTime - startTime;

        // Converts the ▲Time from milliseconds to minutes then finds encoder ticks per minute
        double timeChangeInMin = timeChange / Constants.MILLIS_TO_MIN;

        // To avoid divide by zero we need to be sure timeChangeInMin does not equal zero.
        double ticksPerMinute = 0;
        if (timeChangeInMin != 0) {
            ticksPerMinute = positionChange / timeChangeInMin;
        }

        // If timeChange is not zero return the motor RPM otherwise return zero.
        return (ticksPerMinute);
    }

    public void fireLauncher(double targetRPM) {
        if (targetRPM == 0) {
            servoLauncher.setPosition(Constants.SERVO_LAUNCH_FIRE);
            pauseMillis(100);
            servoLauncher.setPosition(Constants.SERVO_LAUNCH_REST);
        }
        else {
            int fireTimeOut = 0;
            int correctSpeedTick = 0;
            double motorRPM = getMotorTicksPerMinute(motorLauncher, 100) / Constants.AM_37_TICKS_PER_ROTATION;
            while (correctSpeedTick < 3 || fireTimeOut >= 50) {
                telemetry.addData("launcher RPM: ", motorRPM);
                telemetry.update();

                if (motorRPM - targetRPM > 10) {
                    motorLauncher.setPower(motorLauncher.getPower() - 0.05);
                    correctSpeedTick = 0;
                }
                else if(motorRPM - targetRPM < 10){
                    motorLauncher.setPower(motorLauncher.getPower() + 0.05);
                    correctSpeedTick = 0;
                } else{
                    correctSpeedTick++;
                }

                motorRPM = getMotorTicksPerMinute(motorLauncher, 100) / Constants.AM_37_TICKS_PER_ROTATION;

                fireTimeOut++;
            }

            servoLauncher.setPosition(Constants.SERVO_LAUNCH_FIRE);
            pauseMillis(100);
            servoLauncher.setPosition(Constants.SERVO_LAUNCH_REST);
        }
    }

    public void fireLauncher() {
         servoLauncher.setPosition(Constants.SERVO_LAUNCH_FIRE);
         pauseMillis(100);
         servoLauncher.setPosition(Constants.SERVO_LAUNCH_REST);
    }

    // Pauses for time milliseconds
    public void pauseMillis(double time) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            idle();
        }
    }

    public void driveInches(double targetDistance, double degDriveAngle, double maxSpeed) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean distanceReached = false;

        double xPosition = 0;
        double yPosition = 0;
        double distanceLeft;
        double radDriveAngle = Math.toRadians(degDriveAngle);
        double angleDeviation;
        double turningPower;

        PIDFilter translationPID;
        translationPID = new PIDFilter(Constants.TRANSLATION_P, Constants.TRANSLATION_I, Constants.TRANSLATION_D);

        while (!distanceReached && opModeIsActive()) {
            // This calculates the angle deviation
            angleDeviation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - degDriveAngle;

            // This calculates the distance traveled in inches
            double distanceTraveled = Math.sqrt(Math.pow((xPosition - 0), 2) + Math.pow((yPosition - 0), 2));

            // This adds a value to the PID loop so it can update
            distanceLeft = targetDistance - distanceTraveled;
            translationPID.roll(distanceLeft);

            // Todo - "10"
            turningPower = angleDeviation/10;

            // We drive the mecanum wheels with the PID value
            driveMecanum(radDriveAngle, Math.min(Math.max(translationPID.getFilteredValue(), Constants.MINIMUM_DRIVE_POWER), maxSpeed), turningPower);

            // Update positions using last distance measured by encoders
            xPosition = (Constants.IN_PER_ANDYMARK_TICK * (-motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition() - motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 4);

            yPosition = (Constants.IN_PER_ANDYMARK_TICK * (-motorFrontLeft.getCurrentPosition() - motorBackLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 4);

            if (distanceTraveled > targetDistance) {
                driveMecanum(radDriveAngle, 0.0, 0.0);
                distanceReached = true;
            }
        }
    }
}