package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.aim.Vision;
import org.firstinspires.ftc.teamcode.aim.components.Button;
import org.firstinspires.ftc.teamcode.aim.drive.MecanumIMUDrive;

public class Robot {
    private IMU imu = null;
    private DcMotor frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor;
    private MecanumIMUDrive driveCtrl;
    private LinearOpMode opMode;
    private Button botRotateButton = new Button();
    private boolean botRotated = false;

    public Vision vision = new Vision();

    private void initImu() {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RobotConfig.logoDirection, RobotConfig.usbDirection);
        imu = this.opMode.hardwareMap.get(IMU.class, RobotConfig.imuName);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void initDriveMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initWheels() {
        this.frontLeftMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotConfig.frontLeftWheelName);
        this.frontRightMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotConfig.frontRightWheelName);
        this.backLeftMotor = this.opMode.hardwareMap.get(DcMotor.class,  RobotConfig.backLeftWheelName);
        this.backRightMotor = this.opMode.hardwareMap.get(DcMotor.class, RobotConfig.backRightWheelName);

        this.frontLeftMotor.setDirection(RobotConfig.frontLeftWheelDirection);
        this.backLeftMotor.setDirection(RobotConfig.backLeftWheelDirection);
        this.frontRightMotor.setDirection(RobotConfig.frontRightWheelDirection);
        this.backLeftMotor.setDirection(RobotConfig.backLeftWheelDirection);

        initDriveMotor(this.frontLeftMotor);
        initDriveMotor(this.frontRightMotor);
        initDriveMotor(this.backLeftMotor);
        initDriveMotor(this.backRightMotor);
    }

    private void initMecanumIMUDrive() {
        MecanumIMUDrive gryo = new MecanumIMUDrive();
        MecanumIMUDrive.InitParams params = gryo.defaultParams();
        params.opMode = this.opMode;
        params.imuName = RobotConfig.imuName;
        params.frontLeftWheelName = RobotConfig.frontLeftWheelName;
        params.frontRightWheelName = RobotConfig.frontRightWheelName;
        params.backLeftWheelName =  RobotConfig.backLeftWheelName;
        params.backRightWheelName = RobotConfig.backRightWheelName;
        params.countPerMotorRev = RobotConfig.countPerMotorRev; //537.7; //12;
        params.driveGearReduction = RobotConfig.driveGearReduction; // 1.0; // 19.2;
        params.wheelDiameterInches = RobotConfig.wheelDiameterInches; //4.0; //3.78;
        gryo.init(params);

        this.driveCtrl = gryo;
    }

    private void initCamera() {
        vision.init(this.opMode.hardwareMap, this.opMode.telemetry, 9,
                RobotConfig.cameraName, RobotConfig.cameraHeight, RobotConfig.cameraAngel,
                RobotConfig.targetHeight);
    }

    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        this.initImu();
        this.initWheels();
        this.initMecanumIMUDrive();
        if (RobotConfig.cameraEnabled) {
            this.initCamera();
        }
    }

    public void start() {
        if (RobotConfig.cameraEnabled) {
            this.vision.start();
        }
    }

    public void handleRobotMove() {
        this.botRotateButton.update(this.opMode.gamepad1.x);
        if (this.botRotateButton.isToggleOn()) {
            this.botRotated = true;
        }  else if (this.botRotateButton.isToggleOff()) {
            this.botRotated = false;
        }
        int botDirection = 1;
        if (this.botRotated) {
            botDirection = -1;
        }
        double power = 0, x = 0, y = 0, turn = 0;
        double smallTurn = 0.5, bigTurn = 2;

        if (this.opMode.gamepad1.left_stick_x != 0 || this.opMode.gamepad1.left_stick_y != 0) {
            power = 0.5;
            x = (botDirection)*this.opMode.gamepad1.left_stick_x;
            y = -(botDirection)*this.opMode.gamepad1.left_stick_y;
        } else if (this.opMode.gamepad1.right_stick_x != 0 || this.opMode.gamepad1.right_stick_y != 0) {
            power = 3;
            x = (botDirection)*this.opMode.gamepad1.right_stick_x;
            y = -(botDirection)*this.opMode.gamepad1.right_stick_y;
        }

        if (this.opMode.gamepad1.left_bumper){
            turn = bigTurn ;
        } else if (this.opMode.gamepad1.right_bumper){
            turn = -bigTurn;
        } else if (this.opMode.gamepad1.left_trigger > 0.5){
            turn = smallTurn;
        } else if (this.opMode.gamepad1.right_trigger > 0.5){
            turn = -smallTurn;
        }

        this.driveCtrl.moveByPower(power, x, y, turn);
    }

    public void update() {
        handleRobotMove();
        if (RobotConfig.cameraEnabled) {
            vision.update();
        }
    }
}
