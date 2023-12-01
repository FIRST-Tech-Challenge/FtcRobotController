package org.firstinspires.ftc.teamcode.To_Be_Removed;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MiniPID;

public class DrivetrainSubsystemOld {
    //Variable Declarations;

    //Drive Powers
    private double backLeftPower;
    private double backRightPower;
    private double frontLeftPower;
    private double frontRightPower;

    private double forwardDrive;
    private double shuffleDrive;
    private double turnDrive;

    //Drive Motors
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;

    //PIDs
    private MiniPID drivePID, turnPID;

    //IMU
    private IMU imu;

    public DrivetrainSubsystemOld(DcMotor backLeftDrive, DcMotor backRightDrive, DcMotor frontLeftDrive, DcMotor frontRightDrive, IMU imu) {
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive= frontRightDrive;

        this.imu = imu;

        initialize();
    }

    //Initializes the subsystem and its members. Call while the OpMode is running.
    public void initialize() {
        //Drive Motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //PIDs
        drivePID = new MiniPID(Constants.driveK, Constants.driveI, Constants.driveD);
        drivePID.setOutputLimits(-0.2, 0.2);

        turnPID = new MiniPID(Constants.turnP, Constants.turnI, Constants.turnD);
        turnPID.setOutputLimits(-0.2, 0.2);

        //IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    //Moves the motors according to joystick input. Written for mecanum drivetrains.
    public void teleOPDrive(double gamepad1LeftStickY, double gamepad1LeftStickX, double gamepad1RightStickX) {
        forwardDrive = -gamepad1LeftStickY;
        shuffleDrive = gamepad1LeftStickX;
        turnDrive = gamepad1RightStickX;

        frontLeftPower = Range.clip(shuffleDrive + forwardDrive + turnDrive, -1.0, 1.0);
        frontRightPower = Range.clip(-shuffleDrive + forwardDrive - turnDrive, -1.0, 1.0);
        backLeftPower = Range.clip(-shuffleDrive + forwardDrive + turnDrive, -1.0, 1.0);
        backRightPower = Range.clip(shuffleDrive + forwardDrive - turnDrive, -1.0, 1.0);

        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
    }

    //Utilizes the drive PID and a target position to drive forwards or backwards. Ends when the distance between positions becomes minimal.
    public void autoDrive(double targetPositionInches) {
        Long targetPositionLong = Math.round(Constants.driveMotorCPI * targetPositionInches);
        double targetPosition = targetPositionLong.doubleValue() + frontLeftDrive.getCurrentPosition();

        boolean inRange = false;

        android.util.Range<Double> endRange = android.util.Range.create(-0.005, 0.005);

        //Runs when target position is not within the motor stop range.
        while (!inRange) {
            double driveInput = drivePID.getOutput(frontLeftDrive.getCurrentPosition(), targetPosition);

            if (endRange.contains(driveInput)) {
                inRange = true;
            }

            frontLeftDrive.setPower(driveInput);
            frontRightDrive.setPower(driveInput);
            backLeftDrive.setPower(driveInput);
            backRightDrive.setPower(driveInput);
        }

        //Fully stops the motors once distance is reached
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
    }

    //Autonomously shuffles the robot left or right to a target position without changing orientation.
    public void autoShuffle(Directions direction, double targetPositionInches) {
        //Variable Declaration
        Long targetPositionLong;
        double forwardsTargetPosition;
        double backwardsTargetPosition;
        double forwardsDriveInput;
        double backwardsDriveInput;

        boolean inRange = false;

        android.util.Range<Double> endRange = android.util.Range.create(-0.05, 0.05);

        switch (direction) {
            case SHUFFLE_LEFT:
                //Calculates target positions for the forward and backward moving motors. The variable backwardsTargetPosition is negated to reverse the motors' directions.
                targetPositionLong = Math.round(Constants.driveMotorCPI * targetPositionInches);
                forwardsTargetPosition = targetPositionLong.doubleValue();
                backwardsTargetPosition = -forwardsTargetPosition;

                //Runs while neither forward or backward moving motors are within the motor stop range.
                //Note: Shuffling left requires forward movement from (frontRightDrive, backLeftDrive) and backwards movement from (frontLeftDrive, backRightDrive).
                while (!inRange) {
                    forwardsDriveInput = drivePID.getOutput(frontRightDrive.getCurrentPosition(), forwardsTargetPosition);
                    backwardsDriveInput = drivePID.getOutput(frontLeftDrive.getCurrentPosition(), backwardsTargetPosition);

                    if (endRange.contains(forwardsDriveInput) & endRange.contains(backwardsDriveInput)) {
                        inRange = true;
                    }

                    frontLeftDrive.setPower(backwardsDriveInput);
                    frontRightDrive.setPower(forwardsDriveInput);
                    backLeftDrive.setPower(forwardsDriveInput);
                    backRightDrive.setPower(backwardsDriveInput);
                }

                //Fully stops motors once distance is reached.
                frontLeftDrive.setPower(0.0);
                frontRightDrive.setPower(0.0);
                backLeftDrive.setPower(0.0);
                backRightDrive.setPower(0.0);
                break;

            case SHUFFLE_RIGHT:
                //Calculates target positions for the forward and backward moving motors. The variable backwardsTargetPosition is negated to reverse the motors' directions.
                targetPositionLong = Math.round(Constants.driveMotorCPI * targetPositionInches);
                forwardsTargetPosition = targetPositionLong.doubleValue();
                backwardsTargetPosition = -forwardsTargetPosition;

                //Runs while neither forward or backward moving motors are within the motor stop range.
                //Note: Shuffling right requires forward movement from (frontLeftDrive, backRightDrive) and backwards movement from (frontRightDrive, backLeftDrive).
                while (inRange) {
                    forwardsDriveInput = drivePID.getOutput(frontLeftDrive.getCurrentPosition(), forwardsTargetPosition);
                    backwardsDriveInput = drivePID.getOutput(frontRightDrive.getCurrentPosition(), backwardsTargetPosition);

                    if (endRange.contains(forwardsDriveInput) & endRange.contains(backwardsDriveInput)) {
                        inRange = true;
                    }

                    frontLeftDrive.setPower(forwardsDriveInput);
                    frontRightDrive.setPower(backwardsDriveInput);
                    backLeftDrive.setPower(backwardsDriveInput);
                    backRightDrive.setPower(forwardsDriveInput);
                }

                //Fully stops motors once distance is reached.
                frontLeftDrive.setPower(0.0);
                frontRightDrive.setPower(0.0);
                backLeftDrive.setPower(0.0);
                backRightDrive.setPower(0.0);
                break;

            default:
                //Does nothing if provided enum matches no possible directions.
                break;
        }
    }

    public void autoTurn(Directions direction, double turnAngle) {
        double leftDriveInput, rightDriveInput;
        double currentAngle, targetAngle, distance, lastDistance;
        double lastTime = System.currentTimeMillis();
        boolean inRange = false;

        switch (direction) {
            case TURN_RIGHT:
                currentAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                targetAngle = (currentAngle + turnAngle) % 360;
                if (targetAngle > 180) {
                    targetAngle -= 360;
                }

                distance = calculateLargeAngleDistance(currentAngle, targetAngle);
                lastDistance = distance;
                turnPID.setSetpoint(0);

                while (!inRange) {
                    if (distance > 20) {
                        leftDriveInput = -turnPID.getOutput(distance);
                        rightDriveInput = turnPID.getOutput(distance);

                        frontLeftDrive.setPower(leftDriveInput);
                        backLeftDrive.setPower(leftDriveInput);
                        frontRightDrive.setPower(rightDriveInput);
                        backRightDrive.setPower(rightDriveInput);

                        currentAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        lastDistance = distance;
                        distance = calculateLargeAngleDistance(currentAngle, targetAngle);
                        lastTime = System.currentTimeMillis();

                    }

                    else if (distance <= 20 && distance > 5) {
                        leftDriveInput = -turnPID.getOutput(distance);
                        rightDriveInput = turnPID.getOutput(distance);

                        frontLeftDrive.setPower(leftDriveInput);
                        backLeftDrive.setPower(leftDriveInput);
                        frontRightDrive.setPower(rightDriveInput);
                        backRightDrive.setPower(rightDriveInput);

                        currentAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        lastDistance = distance;
                        distance = calculateSmallAngleDistance(currentAngle, targetAngle);
                        lastTime = System.currentTimeMillis();
                    }

                    else {
                        inRange = true;
                        /*
                        double deltaDistance = distance - lastDistance;
                        double timeElapsed = System.currentTimeMillis() - lastTime;
                        double dSdT = deltaDistance / timeElapsed;

                        if (-0.05 < dSdT && dSdT < 0.05) {
                            inRange = true;
                        }

                        leftDriveInput = -turnPID.getOutput(distance);
                        rightDriveInput = turnPID.getOutput(distance);

                        frontLeftDrive.setPower(leftDriveInput);
                        backLeftDrive.setPower(leftDriveInput);
                        frontRightDrive.setPower(rightDriveInput);
                        backRightDrive.setPower(rightDriveInput);

                        currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        lastDistance = distance;
                        distance = calculateSmallAngleDistance(currentAngle, targetAngle);
                        lastTime = System.currentTimeMillis();
                         */
                    }
                }

                frontLeftDrive.setPower(0);
                backLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backRightDrive.setPower(0);
        }
    }

    private double  calculateLargeAngleDistance(double currentAngle, double targetAngle) {
        if (currentAngle > targetAngle) {
            return (180 - currentAngle) + (180 + targetAngle);
        }
        else if (currentAngle < targetAngle) {
            return (180 + targetAngle) - (180 + currentAngle);
        }
        else {
            return 0;
        }
    }

    private double calculateSmallAngleDistance(double currentAngle, double targetAngle) {
        double distance = Math.abs(targetAngle - currentAngle);
        if (distance > 20) {
            distance =  Math.abs(distance - 360);
        }
        return distance;
    }

    //Autonomously turns the robot a degree amount in the provided direction. Utilizes a PID on inputted IMU gyroscope angles.
    public void autoTurnOld(Directions direction, double turnAngle) {
        //Variable Declaration
        double leftDriveInput;
        double rightDriveInput;
        double targetAngle;
        double targetAngleDistance;
        double previousTargetAngleDistance;
        double loopAmount = 0;

        boolean inRange = false;

        android.util.Range<Double> endRange = android.util.Range.create(-0.05, 0.05);;

        switch (direction) {
            case TURN_LEFT:
                //Calculates the target angle on the IMU's gyroscope.
                targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - turnAngle;

                //Calculates the distance between the current gyroscope value and the target value.
                //Gyroscope measures from -180 to 180 degrees. If a target angle crosses over this value , it would loop over (20 degrees left of -170 degrees is 170 degrees).
                //Inputting 170 degrees would cause the robot to turn violently to the right, the opposite of the wanted direction.
                //By finding the distance between the two angles, a vector value is turned to a scalar one, allowing manual decision over the turn direction.
                targetAngleDistance = Math.abs(targetAngle - (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - (loopAmount * 360)));
                previousTargetAngleDistance = targetAngleDistance;

                //Runs when motors are not within endRange.
                while (!inRange) {
                    //Right input is negated to make right motors move forward (PID will return negative values, since distance will be greater than 0).
                    leftDriveInput = turnPID.getOutput(targetAngleDistance, 0);
                    rightDriveInput = -turnPID.getOutput(targetAngleDistance, 0);

                    if (endRange.contains(leftDriveInput) & endRange.contains(rightDriveInput)) {
                        inRange = true;
                    }

                    frontLeftDrive.setPower(leftDriveInput);
                    frontRightDrive.setPower(rightDriveInput);
                    backLeftDrive.setPower(leftDriveInput);
                    backRightDrive.setPower(rightDriveInput);

                    //Distance is recalculated so PID outputs can be adjusted on next loop.
                    //If gyro loops over, the regular distance calculation breaks.
                    //If change in distance is substantially negative, a loop over occurred in the gyroscope readings.
                    //Negative values can happen when robot briefly passes over target distance or skips over Gryoscope value. Thus, a very low value is needed.
                    if (previousTargetAngleDistance - targetAngleDistance < -300) {
                        loopAmount += 1;

                        //Need to reset previous target angle distance, don't want it inheriting a bad value.
                        targetAngleDistance = Math.abs(targetAngle - (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - (loopAmount * 360)));
                        previousTargetAngleDistance = targetAngleDistance;
                    }

                    else {
                        previousTargetAngleDistance = targetAngleDistance;
                        targetAngleDistance = Math.abs(targetAngle - (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - (loopAmount * 360)));
                    }
                }

                //Fully stops motors once angle is reached.
                frontLeftDrive.setPower(0.0);
                frontRightDrive.setPower(0.0);
                backLeftDrive.setPower(0.0);
                backRightDrive.setPower(0.0);
                break;

            case TURN_RIGHT:
                //Calculates the target angle on the IMU's gyroscope.
                targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + turnAngle;

                //Calculates the distance between the current gyroscope value and the target value.
                targetAngleDistance = Math.abs(targetAngle - (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - (loopAmount * 360)));
                previousTargetAngleDistance = targetAngleDistance;

                //Runs when the target angle distance is not within motor stop range.
                while (inRange) {
                    //Left input is negated to make left motors move forward (PID will return negative values, since distance will be greater than 0).
                    leftDriveInput = -turnPID.getOutput(targetAngleDistance, 0);
                    rightDriveInput = turnPID.getOutput(targetAngleDistance, 0);

                    if (endRange.contains(leftDriveInput) & endRange.contains(rightDriveInput)) {
                        inRange = true;
                    }

                    frontLeftDrive.setPower(leftDriveInput);
                    frontRightDrive.setPower(rightDriveInput);
                    backLeftDrive.setPower(leftDriveInput);
                    backRightDrive.setPower(rightDriveInput);

                    //If change in distance is substantially negative, a loop over occurred in the gyroscope readings.
                    //Negative values can happen when robot briefly passes over target distance or skips over Gryo value. Thus, a very low value is needed.
                    if (previousTargetAngleDistance - targetAngleDistance < -300) {
                        loopAmount += 1;

                        //Need to reset previous target angle distance, don't want it inheriting a bad value.
                        targetAngleDistance = Math.abs(targetAngle - (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - (loopAmount * 360)));
                        previousTargetAngleDistance = targetAngleDistance;
                    }

                    else {
                        previousTargetAngleDistance = targetAngleDistance;
                        targetAngleDistance = Math.abs(targetAngle - (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - (loopAmount * 360)));
                    }
                }

                //Fully stops motors once angle is reached.
                frontLeftDrive.setPower(0.0);
                frontRightDrive.setPower(0.0);
                backLeftDrive.setPower(0.0);
                backRightDrive.setPower(0.0);
                break;

            default:
                break;
        }
    }

    //Variable Gets
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
        return frontRightDrive.getPower();
    }

    public double getAngle() {
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    //Directions for Autonomous Commands
    public enum Directions {
        SHUFFLE_LEFT,
        SHUFFLE_RIGHT,
        TURN_LEFT,
        TURN_RIGHT
    }
}

