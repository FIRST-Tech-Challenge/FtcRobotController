package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DrivetrainEncoders {
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftRearDrive = null;
    private DcMotorEx rightRearDrive = null;
    private double maxSpeed = 0.8; // Factor (0.0-1.0) to control drive speed
    private double maxVelocity = Constants.driveTrainMaxVelocity;
    private double moveCountsPerInch = Constants.mecanumMoveCountsPerInch;
    private double strafeCountsPerInch = Constants.mecanumStrafeCountsPerInch;
    private IMU imu = null;
    LinearOpMode opMode = null;

    public DrivetrainEncoders(LinearOpMode opMode, double maxSpeed) {
        this.opMode = opMode;
        this.maxSpeed = maxSpeed;

        leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftRearDrive = opMode.hardwareMap.get(DcMotorEx.class, "left_rear_drive");
        rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightRearDrive = opMode.hardwareMap.get(DcMotorEx.class, "right_rear_drive");

        leftFrontDrive.setDirection(Constants.drivetrainLeftFrontDirection);
        leftRearDrive.setDirection(Constants.drivetrainLeftRearDirection);
        rightFrontDrive.setDirection(Constants.drivetrainRightFrontDirection);
        rightRearDrive.setDirection(Constants.drivetrainRightRearDirection);

        stopAndResetEncoders();
        setRunUsingEncoder();
        setBrakingOn();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void setToFastSpeed()
    {
        this.maxSpeed = Constants.maxNormalSpeed;
    }

    public void setToSlowedSpeed()
    {
        this.maxSpeed = Constants.maxSlowedSpeed;
    }

    public void turnToHeading(double targetHeading) {
        double turnSpeed = Constants.maxAutoCorrectionTurnSpeed;
        double headingError = getHeadingError(targetHeading);

        // keep looping while we are still active, and not on heading.
        while (Math.abs(headingError) > Constants.autoHeadingThreshold) {
            headingError = getHeadingError(targetHeading);

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(headingError, Constants.autoTurnGain);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -Constants.maxAutoCorrectionTurnSpeed, Constants.maxAutoCorrectionTurnSpeed);

            // Pivot in place by applying the turning correction
            moveDirection(0, 0, turnSpeed);
        }
        stop();
    }

    public void turnForDistance(double distance) {
        double targetHeading = getHeading() + distance;
        turnToHeading(targetHeading);
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        moveDirection(axial * Constants.maxCreepSpeed, strafe * Constants.maxCreepSpeed, yaw * Constants.maxCreepSpeed);
    }

    public void creepDirectionNoEnc(double axial, double strafe, double yaw) {
        moveDirection(axial * Constants.maxCreepSpeed, strafe * Constants.maxCreepSpeed, yaw * Constants.maxCreepSpeed);
    }

    public void moveDirection(double axial, double strafe, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = axial - strafe - yaw;
        double rightFrontPower = axial + strafe + yaw;
        double leftRearPower = axial + strafe - yaw;
        double rightRearPower = axial - strafe + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        leftFrontDrive.setVelocity(leftFrontPower * maxSpeed * maxVelocity);
        rightFrontDrive.setVelocity(rightFrontPower * maxSpeed * maxVelocity);
        leftRearDrive.setVelocity(leftRearPower * maxSpeed * maxVelocity);
        rightRearDrive.setVelocity(rightRearPower * maxSpeed * maxVelocity);
    }

    public void strafeForDistance(double distance) {
        int targetCounts = (int) (distance * strafeCountsPerInch);
        int leftFrontTarget = 0;
        int leftRearTarget = 0;
        int rightFrontTarget = 0;
        int rightRearTarget = 0;
        double strafeSpeed = Constants.maxAutoStrafeSpeed;

        leftFrontTarget = leftFrontDrive.getCurrentPosition() + targetCounts;
        leftRearTarget = leftRearDrive.getCurrentPosition() - targetCounts;
        rightFrontTarget = rightFrontDrive.getCurrentPosition() - targetCounts;
        rightRearTarget = rightRearDrive.getCurrentPosition() + targetCounts;

        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftRearDrive.setTargetPosition(leftRearTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);
        rightRearDrive.setTargetPosition(rightRearTarget);

        setRunToPosition();

        while (leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy() && !opMode.isStopRequested()) {
            moveDirection(0, strafeSpeed, 0);
        }
        stop();
        setRunUsingEncoder();
    }

    public void moveStraightForDistance(double distance) {
        moveStraightForDistance(distance, Constants.maxAutoCorrectionTurnSpeed, Constants.maxAutoCorrectionDriveSpeed);
    }

    public void creepStraightForDistance(double distance) {
        moveStraightForDistance(distance, Constants.maxCreepSpeed, Constants.maxCreepSpeed);
    }

    private void moveStraightForDistance(double distance, double turnSpeed, double driveSpeed) {
        int targetCounts = (int) (distance * moveCountsPerInch);
        int leftFrontTarget = 0;
        int leftRearTarget = 0;
        int rightFrontTarget = 0;
        int rightRearTarget = 0;
        double headingError = 0;
        double targetHeading = getHeading();

        leftFrontTarget = leftFrontDrive.getCurrentPosition() + targetCounts;
        leftRearTarget = leftRearDrive.getCurrentPosition() + targetCounts;
        rightFrontTarget = rightFrontDrive.getCurrentPosition() + targetCounts;
        rightRearTarget = rightRearDrive.getCurrentPosition() + targetCounts;

        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftRearDrive.setTargetPosition(leftRearTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);
        rightRearDrive.setTargetPosition(rightRearTarget);

        setRunToPosition();

        while (leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy() && !opMode.isStopRequested()) {
            //while (leftFrontDrive.isBusy()) {
            headingError = getHeadingError(targetHeading);
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(headingError, Constants.autoDriveGain);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveDirection(driveSpeed, 0.0, turnSpeed);
        }
        stop();
        setRunUsingEncoder();
    }

    public double getHeadingError(double targetHeading) {
        return (targetHeading - getHeading());
    }

    public double getSteeringCorrection(double headingError, double gain) {
        // Determine the heading current error

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * gain, -1, 1);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void stop() {
        leftFrontDrive.setVelocity(0.0);
        leftRearDrive.setVelocity(0.0);
        rightFrontDrive.setVelocity(0.0);
        rightRearDrive.setVelocity(0.0);
    }

    private void setBrakingOn() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setBrakingOff() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void stopAndResetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRunWithoutEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setRunUsingEncoder() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setRunToPosition() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
