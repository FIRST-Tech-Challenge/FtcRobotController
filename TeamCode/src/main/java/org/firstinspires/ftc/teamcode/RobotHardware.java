package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    // Declare Motors
    // public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    // public DcMotorEx slideLeft, slideRight;

    // Declare Servos
    // public Servo claw, grabber, leftOuttake, rightOuttake, wrist, leftIntake, rightIntake;

    // public GoBildaPinpointDriver odo;

    // Sensors
    public IMU imu;

    // declare some constants (servo positions, etc)
    /*public final double INTAKE_IN_LEFT = .78;
    public final double INTAKE_IN_RIGHT = .22;
    public final double GRABBER_OPEN = 0;
    public final double GRABBER_CLOSE = .2;
    public final double WRIST_NEUTRAL = .3;
    */


    // Hardware Map Reference
    private HardwareMap hardwareMap;

    // Constructor
    public RobotHardware(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

    // Initialize hardware
    public void init() {
        // Make sure the device name matches what is in the config on the bot.
        // Motors
        /*leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        slideLeft = hardwareMap.get(DcMotorEx.class, "SL");
        slideRight = hardwareMap.get(DcMotorEx.class, "SR");
        */

        // Set motor directions (typically the left side)
        // leftFront.setDirection(DcMotor.Direction.REVERSE);
        // leftRear.setDirection(DcMotor.Direction.REVERSE);


        // Set motors to brake when power is zero
        // slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");


        // clear encoder values for slides
        /* slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */

        // Servos (make sure names match config)
        /* claw = hardwareMap.get(Servo.class, "Claw");
        grabber = hardwareMap.get(Servo.class, "Grabber");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        leftOuttake = hardwareMap.get(Servo.class, "LeftOuttake");
        rightOuttake = hardwareMap.get(Servo.class, "RightOuttake");
        leftIntake = hardwareMap.get(Servo.class, "LeftIntake");
        rightIntake = hardwareMap.get(Servo.class, "RightIntake");
         */

        // Sensors (make sure imu is created in config)
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }


    // Create methods/functions that you will use multiple times across multiple
    // op modes (auto and/or tele).
        

}