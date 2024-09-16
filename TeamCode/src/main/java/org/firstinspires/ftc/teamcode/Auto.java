package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoMain")
public class Auto extends LinearOpMode{
    static final double TICKS_PER_MOTOR_REV = 100; // needs to be changed
    static final double DRIVE_GEAR_REDUCTION = 1.0; // needs to be changed
    static final double WHEEL_DIAMETER_INCHES = 100; // needs to be changed
    static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    final private double DEFAULT_POWER = 1.0;
    Hardware hw = Hardware.getInstance(this);
//    PIDController drivePID = new PIDController(hw.encoderOdomX, 1, 0, 0);
//    PIDController turnPID = new PIDController(hw.gyro, 1, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        hw.init(hardwareMap);

        waitForStart();
        drive(10);
        drive(20);
        drive(20, 0.5);
        turn(20);
        turn(20, 0.5, 0.12);
        turn(23);
        turn(24, 0.3, 0.10);
        strafe(20);
        strafe(20, 0.5);
        strafe(23);
        strafe(23, 0.5);
    }

    private void drive(double inches) {
        drive(inches, DEFAULT_POWER);
    }

    private void drive(double inches, double power) {
        double Kp = 1.0;
        double Ki = 0.0;
        double Kd = 0.0;
        int targetPos = (int) (inches/TICKS_PER_INCH);
        double encoderPos = 0;
        double error;
        double integralSum = 0;
        double lastError = 0;
        double derivative;
        double out;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && hw.notInRange(targetPos)) {
            // obtain the encoder position
//            encoderPos = hw.odomX.getPosition();
            // calculate the error
            error = targetPos - encoderPos;

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            hw.setMotorsToPower(out);

            lastError = error;
            timer.reset();
        }
        hw.setMotorsToPower(0);
    }

    private void turn(double degrees) {
        turn(degrees, degrees > 0 ? 0.15 : -0.15, 0.50);
    }

    /**
     * @param degrees - turning right is positive, left is negative
     */
    private void turn(double degrees, double minPower, double threshold) {
        double kP = 1.0;
        double error;
        double out;
        degrees = (degrees - 360.0) % 360.0;
        while(opModeIsActive() && ((hw.getGyroAngle() - threshold) <= degrees) && 
                (hw.getGyroAngle() + threshold) >= degrees){
            error = degrees = hw.getGyroAngle();
            out = (error * kP) + minPower;
            hw.frontLeft.setPower(out);
            hw.frontRight.setPower(out);
            hw.backLeft.setPower(-out);
            hw.backRight.setPower(-out);
            hw.telemetryHardware();
        }
    }

    private void strafe(double inches) {
        strafe(inches, DEFAULT_POWER);
    }

    /**
     * This should NOT be used in regular auto, this is just here as a backup method. Strafe will not
     * have as much power as turning then driving which is the method which should be used instead
     *  of strafing.
    @param inches - left is negative, right is positive
     */
    private void strafe(double inches, double power) {
        int targetPos = (int) (inches/TICKS_PER_INCH);
        hw.frontLeft.setTargetPosition(targetPos + hw.frontLeft.getCurrentPosition());
        hw.frontRight.setTargetPosition(-targetPos + hw.frontRight.getCurrentPosition());
        hw.backLeft.setTargetPosition(-targetPos + hw.backLeft.getCurrentPosition());
        hw.backRight.setTargetPosition(targetPos + hw.backRight.getCurrentPosition());

        hw.setMotorsToPower(power);

        hw.setMotorsToRunToPosition();
        while(opModeIsActive() && hw.notInRange(targetPos)){
            hw.telemetryHardware();
        }

        hw.setMotorsToPower(0);
    }
}
