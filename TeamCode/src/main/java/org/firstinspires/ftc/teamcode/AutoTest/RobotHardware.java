package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

/*
Hardware config:
Motor:
Control hub motor: FL_Motor 0, BL_motor 1, FR_Motor,2, BR_Motor
Expansion hub motor: VS_Motor_Left 0, VS_Motor_Right 1

Servo:
IntakeArm_Servo 0, Intake_Servo 1

Color Sensor:
Color_Sensor I2C 1

 */

public class RobotHardware {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    public DcMotorEx liftMotorLeft;// Vertical Slide Motor
    public DcMotorEx liftMotorRight;// Vertical Slide Motor

    public Servo intakeSlideServo;
    public Servo intakeLeftArmServo;
    public Servo intakeRightArmServo;
    public Servo intakeRotationServo;
    public Servo intakeClawServo;
    public Servo depositLeftArmServo;
    public Servo depositRightArmServo;
    public Servo depositWristServo;
    public Servo depositClawServo;


    public ColorSensor Color_Sensor;// Color Sensor

    public IMU imu; //IMU
    public HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap; // store the hardwareMap reference
        //set Motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");
        liftMotorLeft = hardwareMap.get(DcMotorEx.class,"VS_Left_Motor");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "VS_Right_Motor");

        //set servos
        intakeSlideServo = hardwareMap.get(Servo.class, "Intake_Slide_Servo");
        intakeLeftArmServo = hardwareMap.get(Servo.class, "Intake_Arm_Left_Servo");
        intakeRightArmServo = hardwareMap.get(Servo.class, "Intake_Arm_Right_Servo");
        intakeRotationServo = hardwareMap.get(Servo.class, "Intake_Rotation_Servo");
        intakeClawServo = hardwareMap.get(Servo.class, "Intake_Claw_Servo");
        depositLeftArmServo = hardwareMap.get(Servo.class, "Deposit_Arm_Left_Servo");
        depositRightArmServo = hardwareMap.get(Servo.class, "Deposit_Arm_Right_Servo");
        depositWristServo = hardwareMap.get(Servo.class, "Deposit_Wrist_Servo");
        depositClawServo = hardwareMap.get(Servo.class, "Deposit_Claw_Servo");


        //set motor mode and motor direction
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // Reverse the left motor if needed
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // Reverse the left motor if needed
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motor mode

        // Reset to RUN_USING_ENCODER mode
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set servo direction
        intakeRightArmServo.setDirection(Servo.Direction.REVERSE);
        depositRightArmServo.setDirection(Servo.Direction.REVERSE);

        //set to RUN_TO_POSITION for vertical slide motor
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set robot motor power 0
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }// End of init

    // Initialize IMU
    public void initIMU() {
        // set up REVimu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                ));
        imu.initialize(myIMUparameters);
        imu.resetYaw();
    }
}