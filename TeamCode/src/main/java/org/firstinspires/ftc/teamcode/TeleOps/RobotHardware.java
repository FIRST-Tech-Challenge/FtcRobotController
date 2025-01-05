package org.firstinspires.ftc.teamcode.TeleOps;

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
Control hub motor:
                port 0: FL_Motor
                port 1: BL_motor
                port 2: FR_Motor
                port 3: BR_Motor
Expansion hub motor:
                port 0 VS_Left_Motor
                port 2: VS_Right_Motor

Servo:
Control hub:
                port 0: Intake_Wrist_Servo
                port 1: Intake_Arm_Left_Servo
                port 2: Deposit_Wrist_Servo
                port 3: Deposit_Claw_Servo
                port 4: Deposit_Arm_Servo
                port 5: Empty

Expansion hub:
                port 0: Empty
                port 1: Intake_Slide_Right_Servo
                port 2: Intake_Slide_Left_Servo
                port 3: Intake_Claw_Servo
                port 4: Intake_Rotation_Servo
                port 5: Intake_Arm_Right_Servo


I2C port
control hub
                port 0: control hub imu
                port 1: pinpoint odometry computer
                port 2: Color_Sensor

 */

public class RobotHardware {
    //Drive chassis motor
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    public DcMotorEx liftMotorLeft;// Vertical Slide Motor
    public DcMotorEx liftMotorRight;// Vertical Slide Motor

    //Intake servos
    public Servo intakeLeftSlideServo;
    public Servo intakeRightSlideServo;
    public Servo intakeLeftArmServo;
    public Servo intakeRightArmServo;
    public Servo intakeRotationServo;
    public Servo intakeClawServo;
    public Servo intakeWristServo;

    //Deposit servos
    public Servo depositArmServo;
    public Servo depositWristServo;
    public Servo depositClawServo;

    public ColorSensor colorSensor;// Color Sensor

    public IMU imu; //IMU
    public HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap; // store the hardwareMap reference
        /**Set up motors**/
        //Drive train motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");
        //Lift motors
        liftMotorLeft = hardwareMap.get(DcMotorEx.class,"VS_Left_Motor");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "VS_Right_Motor");


        /**set servos**/
        //Intake servo
        intakeLeftSlideServo = hardwareMap.get(Servo.class, "Intake_Slide_Left_Servo");
        intakeRightSlideServo = hardwareMap.get(Servo.class, "Intake_Slide_Right_Servo");
        intakeLeftArmServo = hardwareMap.get(Servo.class, "Intake_Arm_Left_Servo");
        intakeRightArmServo = hardwareMap.get(Servo.class, "Intake_Arm_Right_Servo");
        intakeWristServo = hardwareMap.get(Servo.class, "Intake_Wrist_Servo");
        intakeRotationServo = hardwareMap.get(Servo.class, "Intake_Rotation_Servo");
        intakeClawServo = hardwareMap.get(Servo.class, "Intake_Claw_Servo");
        //Deposit servo
        depositArmServo = hardwareMap.get(Servo.class, "Deposit_Arm_Servo");
        depositWristServo = hardwareMap.get(Servo.class, "Deposit_Wrist_Servo");
        depositClawServo = hardwareMap.get(Servo.class, "Deposit_Claw_Servo");
        //Color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");

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
        intakeLeftSlideServo.setDirection(Servo.Direction.REVERSE);

        //set slide motors direction
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
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                ));
        imu.initialize(myIMUparameters);
        imu.resetYaw();
    }
}