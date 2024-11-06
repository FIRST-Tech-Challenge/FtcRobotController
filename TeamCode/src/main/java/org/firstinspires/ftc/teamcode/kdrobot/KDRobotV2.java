package org.firstinspires.ftc.teamcode.kdrobot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class KDRobotV2 {
    DcMotorEx armMotor = null;
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;
    CRServo intakeServo = null;
    double driveTrainSpeed;
    int armPosition;

    public Telemetry telemetry = null;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        armMotor = hwMap.get(DcMotorEx.class, "armMotor");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        //intakeServo = hwMap.get(CRServo.class, "intakeServo");
        this.telemetry = telemetry;

        armPosition = 0;
        driveTrainSpeed = 0.1;
        this.setZeroPowerBehavior();
        this.setInitDriveTrainPower();
        this.initArmMotor();
        this.setWheelDirection();
        this.initImu(hwMap);
    }

    private void initImu(HardwareMap hwMap) {
        IMU imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();
    }

    private void setZeroPowerBehavior() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setInitDriveTrainPower(){
        this.setWheelPower(0, 0,0, 0);
    }

    public void setDriveTrainSpeed(double speed) {
        this.driveTrainSpeed = speed;
    }
    private void initArmMotor(){
        // armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public void setArmMotorVelocity(double velocity){
        //  armMotor.setVelocity(velocity);
    }

    public void setArmMotorMode(DcMotor.RunMode mode){
        // armMotor.setMode(mode);
    }

    private void setWheelDirection(){

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void stopDriveTrain(){
        this.setWheelPower(0,0,0,0);
    }

    public void moveDriveTrain(String direction) {
        switch (direction) {
            case DriveTrainDirection.FORWARD:
                this.setWheelPower(driveTrainSpeed, driveTrainSpeed, driveTrainSpeed, driveTrainSpeed);
                break;
            case DriveTrainDirection.BACKWARD:
                this.setWheelPower(-driveTrainSpeed, -driveTrainSpeed, -driveTrainSpeed, -driveTrainSpeed);
                break;
            case DriveTrainDirection.STRAFE_LEFT:
                this.setWheelPower(-driveTrainSpeed, driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed);
                break;
            case DriveTrainDirection.STRAFE_RIGHT:
                this.setWheelPower(driveTrainSpeed, -driveTrainSpeed, -driveTrainSpeed, driveTrainSpeed);
                break;
            case DriveTrainDirection.TURN_LEFT:
                this.setWheelPower(-driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed, driveTrainSpeed);
                break;
            case DriveTrainDirection.TURN_RIGHT:
                this.setWheelPower(driveTrainSpeed, -driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed);
                break;
            case DriveTrainDirection.STRAFE_DIAGONAL_FORWARD_LEFT:
                this.setWheelPower(0, driveTrainSpeed, driveTrainSpeed, 0);
                break;
            case DriveTrainDirection.STRAFE_DIAGONAL_FORWARD_RIGHT:
                this.setWheelPower(driveTrainSpeed, 0, 0, driveTrainSpeed);
                break;
            case DriveTrainDirection.STRAFE_DIAGONAL_BACKWARD_LEFT:
                this.setWheelPower(-driveTrainSpeed, 0, 0, -driveTrainSpeed);
                break;
            case DriveTrainDirection.STRAFE_DIAGONAL_BACKWARD_RIGHT:
                this.setWheelPower(0, -driveTrainSpeed, -driveTrainSpeed, 0);
                break;
        }
        //telemetry.addData("Motor running forward at", driveTrainSpeed);
        //telemetry.update();
    }

    public void setWheelPower(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        this.frontLeft.setPower(frontLeftSpeed);
        this.frontRight.setPower(frontRightSpeed);
        this.backLeft.setPower(backLeftSpeed);
        this.backRight.setPower(backRightSpeed);
    }
    public void changeDriveTrainSpeed(String direction){
        switch (direction){
            case DriveTrainSpeed.INCREASE:
                this.driveTrainSpeed = Range.clip(this.driveTrainSpeed + 0.1, 0.1, 1.0);
                break;
            case DriveTrainSpeed.DECREASE:
                this.driveTrainSpeed = Range.clip(this.driveTrainSpeed - 0.1, 0.1, 1.0);
                break;
        }
        telemetry.addData("Changed Drive Train Speed ", driveTrainSpeed);
        telemetry.update();
    }

    public void moveArmPosition(String direction) {
        switch (direction) {
            case ArmDirection.UP:
                //sleep(10);
                this.armPosition = Range.clip(this.armPosition + 3, 0, 550);
                break;
            case ArmDirection.DOWN:
                //sleep(10);
                this.armPosition = Range.clip(this.armPosition  - 3, 0, 550);
                break;
        }
        //armMotor.setTargetPosition(this.armPosition);
        //armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //telemetry.addData("armposition",armPosition);
        //telemetry.addData("armDirection",direction);
        //telemetry.addData("is at target", !armMotor.isBusy());
        //telemetry.update();
    }

    public void setServoPower(String direction) {
        switch (direction) {
            case ServoDirection.INTAKE:
                this.intakeServo.setPower(1);
                telemetry.addData("Servo position", "1");

                break;
            case ServoDirection.OUTTAKE:
                this.intakeServo.setPower(-1);
                telemetry.addData("Servo position", "-1");
                break;
            case ServoDirection.STOP:
                this.intakeServo.setPower(0);
                telemetry.addData("Servo position", "0");
                break;
        }

        telemetry.update();
    }
}


