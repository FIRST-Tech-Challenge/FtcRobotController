package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotHardware {
    // Declare Motors
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public DcMotorEx slideLeft, slideRight;

    private PIDController liftPID;

    // Declare Servos
    public Servo claw, grabber, leftOuttake, rightOuttake, wrist, leftIntake, rightIntake;

    //public GoBildaPinpointDriver odo;

    // Sensors
    public IMU imu;

    public final double INTAKE_IN_LEFT = .78;
    public final double INTAKE_IN_RIGHT = .22;
    public final double GRABBER_OPEN = 0;
    public final double GRABBER_CLOSE = .2;
    public final double WRIST_NEUTRAL = .3;
    public final double WRIST_BACK = 0;
    public final double WRIST_MID = .5;
    public final double WRIST_HOVER = .9;
    public final double WRIST_GRAB = 1;
    public final double CLAW_OPEN = 0.3;
    public final double CLAW_CLOSE = 0.1;

    // Hardware Map Reference
    private HardwareMap hardwareMap;

    // Constructor
    public RobotHardware(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

    // Initialize hardware
    public void init() {
        // Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        slideLeft = hardwareMap.get(DcMotorEx.class, "SL");
        slideRight = hardwareMap.get(DcMotorEx.class, "SR");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake when power is zero
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");


        // clear encoder values for slides
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize PID Controller for slide(Tune these values!)
        liftPID = new PIDController(0.01, 0, 0);
        liftPID.setMaxOutput(1.0);

        // Servos
        claw = hardwareMap.get(Servo.class, "Claw");
        grabber = hardwareMap.get(Servo.class, "Grabber");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        leftOuttake = hardwareMap.get(Servo.class, "LeftOuttake");
        rightOuttake = hardwareMap.get(Servo.class, "RightOuttake");
        leftIntake = hardwareMap.get(Servo.class, "LeftIntake");
        rightIntake = hardwareMap.get(Servo.class, "RightIntake");

        // Sensors
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }



    // Helper method to set all motors to a specific run mode
    private void setMotorRunMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }

    public void setLiftPosition(int targetPosition, double power) {
        slideLeft.setTargetPosition(targetPosition);
        slideRight.setTargetPosition(targetPosition);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(power);
        slideRight.setPower(power);
    }

    public void syncLift(double targetPower) {
        int leftPos = slideLeft.getCurrentPosition();
        int rightPos = slideRight.getCurrentPosition();

        // Calculate power correction using PID
        double correction = liftPID.calculate(leftPos, rightPos);

        slideLeft.setPower(targetPower - correction);
        slideRight.setPower(targetPower + correction);
    }

}