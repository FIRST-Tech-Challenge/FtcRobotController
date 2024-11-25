package org.firstinspires.ftc.teamcode.TeleOp;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //Drive chassis motor
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    public DcMotorEx liftMotorLeft;// Vertical Slide Motor
    public DcMotorEx liftMotorRight;// Vertical Slide Motor

    //Intake servos
    public Servo intakeSlideServo;
    public Servo intakeLeftArmServo;
    public Servo intakeRightArmServo;
    public Servo intakeRotationServo;
    public Servo intakeClawServo;

    //Deposit servos
    public Servo depositLeftArmServo;
    public Servo depositRightArmServo;
    public Servo depositWristServo;
    public Servo depositClawServo;


    public ColorSensor Color_Sensor;// Color Sensor

    public IMU imu; //IMU
    public HardwareMap hardwareMap;

    public void init(@NonNull HardwareMap hardwareMap) {
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

        //Reset the drive train motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set drive train motor run mode
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode

        //set servo direction - intake and deposit
        intakeRightArmServo.setDirection(Servo.Direction.REVERSE);
        depositRightArmServo.setDirection(Servo.Direction.REVERSE);

        //set slide motors to RUN_TO_POSITION for vertical slide motor
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Reset the motor encoder
        liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Set the run mode of the motors
        liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // set robot motor power 0
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }// End of init

    // Initialize IMU
    public void initIMU() {
        // set up REV imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                ));
        imu.initialize(myIMUparameters);
    }
}
