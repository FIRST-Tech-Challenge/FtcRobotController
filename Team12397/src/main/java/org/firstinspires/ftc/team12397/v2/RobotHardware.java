package org.firstinspires.ftc.team12397.v2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    private LinearOpMode myOpMode = null;

    public ElapsedTime runtime = new ElapsedTime();

    public IMU imu = null;

    public double drivePower=0;
    public double strafePower=0;
    public double turnPower=0;

    public double leftFrontPower = 0;
    public double leftBackPower = 0;
    public double rightFrontPower = 0;
    public double rightBackPower = 0;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;


    private DcMotor slideMotorL = null;
    private DcMotor slideMotorR = null;

    private Servo leftExtend = null;
    private Servo rightExtend = null;
    private Servo inClawPitch = null;
    private Servo inClaw = null;
    private Servo getInClawYaw = null;
    private Servo outClaw = null;



    IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));


    public RobotHardware(LinearOpMode OpMode) {myOpMode = OpMode;}
/**
  Initialize all the robot's hardware.
  This method must be called ONCE when the OpMode is initialized.
**/
    public void init() {
        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        leftBack = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "right_back");

        slideMotorL = myOpMode.hardwareMap.get(DcMotor.class, "slide_motor_left");
        slideMotorR = myOpMode.hardwareMap.get(DcMotor.class, "slide_motor_right");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        leftExtend = myOpMode.hardwareMap.get(Servo.class, "leftExtend");
        rightExtend = myOpMode.hardwareMap.get(Servo.class, "rightExtend");
        inClawPitch = myOpMode.hardwareMap.get(Servo.class, "inClawPitch");
        inClaw = myOpMode.hardwareMap.get(Servo.class, "inClaw");
        getInClawYaw = myOpMode.hardwareMap.get(Servo.class, "getInClawYaw");
        outClaw = myOpMode.hardwareMap.get(Servo.class, "outClaw");

        imu = myOpMode.hardwareMap.get(IMU .class, "imu");
        IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

    }

//     @param Drive     Fwd/Rev driving power(-1.0 to 1.0) +ve is forward
//     @param Turn      Right/Left turning power(-1.0 to 1.0) +ve is CW
//     @param Strafe

    public void driveRobotCentric(double Drive,double Strafe, double Turn) {


        //Combine drive and turn for blended motion.
        double leftFrontPower = Drive + Strafe + Turn;
        double leftBackPower = Drive - Strafe + Turn;
        double rightFrontPower = Drive - Strafe - Turn;
        double rightBackPower = Drive + Strafe - Turn;

        //Scale the values so neither exceed +/-1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

    }

    public void driveFieldCentric(double drive, double strafe, double turn) {
        //make the parameters store the value assigned when calling the method
        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);


        // Combine drive and Turn for blended motion.
        leftFrontPower = driveRotation + strafeRotation + turn;
        leftBackPower = driveRotation - strafeRotation + turn;
        rightFrontPower = driveRotation - strafeRotation - turn;
        rightBackPower = driveRotation + strafeRotation -turn;

        // Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFrontPower) , Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max= Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0){
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower (leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);


    }

    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFront.setPower(leftFrontWheel);
        leftBack.setPower(leftBackWheel);
        rightFront.setPower(rightFrontWheel);
        rightBack.setPower(rightBackWheel);
    }

    // placeholder
    public void setServoPosition(int servoNum, double position){
        switch (servoNum) {
            case 0:
                leftExtend.setPosition(position);
                break;
            case 1:
                rightExtend.setPosition(position);
                break;
            case 2:
                inClawPitch.setPosition(position);
                break;
            case 3:
                inClaw.setPosition(position);
                break;
            case 4:
                getInClawYaw.setPosition(position);
                break;
            case 5:
                outClaw.setPosition(position);
                break;
        }
    }
}