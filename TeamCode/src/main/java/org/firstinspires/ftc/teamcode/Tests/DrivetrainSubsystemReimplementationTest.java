package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MiniPID;

public class DrivetrainSubsystemReimplementationTest {
    private ElapsedTime runtime;
    private Telemetry telemetry;

    //Drive Powers
    private double backLeftPower;
    private double backRightPower;
    private double frontLeftPower;
    private double frontRightPower;

    //Drive Motors
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;

    private IMU imu;

    private MiniPID drivePID = new MiniPID(Constants.driveK, Constants.driveI, Constants.driveD);

    public enum Directions {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    public DrivetrainSubsystemReimplementationTest(DcMotor backRightDrive, DcMotor backLeftDrive, DcMotor frontRightDrive,
                                                   DcMotor frontLeftDrive, IMU imu, ElapsedTime runtime, Telemetry telemetry) {
        this.backRightDrive = backRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.frontLeftDrive = frontLeftDrive;
        this.imu = imu;
        this.runtime = runtime;
        this.telemetry = telemetry;

        initialize();
    }

    //Initializes Hardware
    private void initialize() {
        //Drive Motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        if (Constants.unlockDriveSpeed) {
            drivePID.setOutputLimits(-1.0, 1.0);
        }
        else {
            drivePID.setOutputLimits(-0.25, 0.25);
        }
        boolean drivePIDReversed = false;
        drivePID.setDirection(drivePIDReversed);
    }

    //General Drive / Motor Methods

    //Drive Not Utilizing Field Orientation
    protected void drive(double shuffleDrive, double forwardDrive, double turnDrive) {

        if (Constants.unlockDriveSpeed) {
            frontLeftPower = Range.clip(shuffleDrive + forwardDrive + turnDrive, -1.0, 1.0);
            frontRightPower = Range.clip(-shuffleDrive + forwardDrive - turnDrive, -1.0, 1.0);
            backLeftPower = Range.clip(-shuffleDrive + forwardDrive + turnDrive, -1.0, 1.0);
            backRightPower = Range.clip(shuffleDrive + forwardDrive - turnDrive, -1.0, 1.0);
        }
        else {
            frontLeftPower = Range.clip(shuffleDrive + forwardDrive + turnDrive, -Constants.maxDrivePower, Constants.maxDrivePower);
            frontRightPower = Range.clip(-shuffleDrive + forwardDrive - turnDrive, -Constants.maxDrivePower, Constants.maxDrivePower);
            backLeftPower = Range.clip(-shuffleDrive + forwardDrive + turnDrive, -Constants.maxDrivePower, Constants.maxDrivePower);
            backRightPower = Range.clip(shuffleDrive + forwardDrive - turnDrive, -Constants.maxDrivePower, Constants.maxDrivePower);
        }

        moveMotors();
    }

    //Drive Utilizing Field Orientation
    protected void drive(double shuffleDrive, double forwardDrive, double turnDrive, double a, double b) {

        frontLeftPower = turnDrive + (shuffleDrive * b) + (forwardDrive * a);
        frontRightPower = -turnDrive + (-shuffleDrive * a) + (forwardDrive * b);
        backLeftPower = turnDrive + (-shuffleDrive * a) + (forwardDrive * b);
        backRightPower = -turnDrive + (shuffleDrive * b) + (forwardDrive * a);

        if (Constants.unlockDriveSpeed) {
            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            backRightPower = Range.clip(backRightPower, -1.0, 1.0);
        }
        else {
            frontLeftPower = Range.clip(frontLeftPower, -Constants.maxDrivePower, Constants.maxDrivePower);
            frontRightPower = Range.clip(frontRightPower, -Constants.maxDrivePower, Constants.maxDrivePower);
            backLeftPower = Range.clip(backLeftPower, -Constants.maxDrivePower, Constants.maxDrivePower);
            backRightPower = Range.clip(backRightPower, -Constants.maxDrivePower, Constants.maxDrivePower);
        }

        moveMotors();
    }

    protected void moveMotors() {
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
    }

    //Stop Motors
    protected void stopMotors() {
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
    }

    //Specific Actions

    //Calculates Joystick Inputs into Drive Commands
    public void driveManual(double leftStickX, double leftStickY, double rightStickX) {
        if (Constants.fieldOrientation) {
            double angleRadians = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double a = Math.cos(angleRadians) - Math.sin(angleRadians);
            double b = Math.cos(angleRadians) + Math.sin(angleRadians);
            drive(leftStickX, -leftStickY, rightStickX, a, b);
        }
        else {
            drive(leftStickX, -leftStickY, rightStickX);
        }
    }

    public void driveAuto(double distanceInches, Directions direction) {
        int startPosition = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()
                + frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition();
        int currentPosition = startPosition;

        int targetPosition;
        double motorPower;
        boolean motorsActive = true;

        switch (direction) {
            case FORWARD:
                targetPosition = (int) (4 * Constants.driveMotorCPI * distanceInches) + startPosition;
                break;
            case BACKWARD:
                targetPosition = (int) (4 * Constants.driveMotorCPI * distanceInches) - startPosition;
                break;
            default:
                return;
        }

        drivePID.setSetpoint(targetPosition);
        double startTime = runtime.seconds();
        //Need to make sure that this will actually always stop eventually... don't want any infinite loops.
        while (motorsActive) {
            motorPower = drivePID.getOutput(currentPosition);

            double derivative = drivePID.derivativeOfError;
            if (-1 < derivative && derivative < 1 & runtime.seconds() - startTime > 1) {
                motorsActive = false;
            }
            drive(0.0, motorPower, 0.0);
        }
        //Once motorsActive is false, stop motors.
        stopMotors();
    }

    public void turnMax180Auto(double turnAngleDegrees, Directions direction) {
        double turnAngleRadians = turnAngleDegrees * (Math.PI / 180);
        turnAngleRadians = Range.clip(turnAngleRadians, -Math.PI, Math.PI);
        double startAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double currentAngle = startAngle;
        double targetAngle;
        boolean motorsActive = true;
        boolean loopOverCorrection = true;
        double distance;
        double motorPower;
        double startTime;

        switch (direction) {
            case RIGHT:
                targetAngle = startAngle + turnAngleRadians;
                if (targetAngle > Math.PI) {
                    targetAngle -= 2 * Math.PI;
                }
                drivePID.setSetpoint(0);
                startTime = runtime.seconds();

                while(motorsActive) {
                    if(currentAngle > targetAngle && loopOverCorrection) {
                        distance = (Math.PI - currentAngle) + (targetAngle + Math.PI);
                    }
                    else {
                        loopOverCorrection = false;
                        distance = targetAngle - currentAngle;
                    }

                    motorPower = -drivePID.getOutput(distance * 180/Math.PI);

                    double derivative = drivePID.derivativeOfError;
                    if (-0.005 < derivative && derivative < 0.005 && runtime.seconds() - startTime > 1) {
                        motorsActive = false;
                    }
                    telemetry.addData("Passed derivative check", true);
                    telemetry.addData("Distance", distance);
                    telemetry.addData("Angle", getAngle());
                    telemetry.update();

                    drive(0.0, 0.0, motorPower);
                    currentAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }

                stopMotors();
                telemetry.addData("Stopped Motors", true);
                telemetry.addData("Angle", getAngle());
                telemetry.update();
                break;

            case LEFT:
                targetAngle = startAngle - turnAngleRadians;
                if (targetAngle < -Math.PI) {
                    targetAngle += 2 * Math.PI;
                }
                drivePID.setSetpoint(0);
                startTime = runtime.seconds();

                while(motorsActive) {
                    if(currentAngle < targetAngle && loopOverCorrection) {
                        distance = (Math.PI - targetAngle) + (currentAngle + Math.PI);
                    }
                    else {
                        loopOverCorrection = false;
                        distance = targetAngle - currentAngle;
                    }

                    motorPower = -drivePID.getOutput(distance * 180/Math.PI);

                    double derivative = drivePID.derivativeOfError;
                    if (-0.005 < derivative && derivative < 0.005 && runtime.seconds() - startTime > 1) {
                        motorsActive = false;
                    }
                    telemetry.addData("Passed derivative check", true);
                    telemetry.addData("Distance", distance);
                    telemetry.addData("Angle", getAngle());
                    telemetry.update();

                    drive(0.0, 0.0, motorPower);
                    currentAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }

                stopMotors();
                telemetry.addData("Stopped Motors", true);
                telemetry.addData("Angle", getAngle());
                telemetry.update();
                break;

            default:
                return;
        }
    }

    public void resetGyro() {
        imu.resetYaw();
    }


    //Get Methods
    public double getBackLeftPower() {
        return backLeftDrive.getPower();
    }

    public double getBackRightPower() {
        return backRightDrive.getPower();
    }

    public double getFrontLeftPower() {
        return frontLeftDrive.getPower();
    }

    public double getFrontRightPower() {
        return  frontRightDrive.getPower();
    }

    public double getAngle() {
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
