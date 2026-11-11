package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public static final String BACK_LEFT_NAME   = "backLeft";
    public static final String BACK_RIGHT_NAME  = "backRight";
    public static final String INTAKE_NAME      = "intake";
    public static final String LAUNCHER_NAME    = "launcher";
    public static final String LEFT_SERVO_NAME  = "frontLeft";
    public static final String RIGHT_SERVO_NAME = "frontRight";

    public static final double DRIVE_SPEED     = 0.8;
    public static final double INTAKE_POWER    = 1.0;
    public static final double LAUNCHER_POWER  = 1.0;
    public static final double SERVO_OPEN      = 1.0;
    public static final double SERVO_CLOSED    = 0.0;

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotor launcher;
    private Servo leftServo;
    private Servo rightServo;

    private final LinearOpMode opMode;

    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        HardwareMap hardwareMap = opMode.hardwareMap;

        backLeft  = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);
        intake    = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        launcher  = hardwareMap.get(DcMotor.class, LAUNCHER_NAME);
        leftServo  = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }


    public void arcadeDrive(double drive, double turn) {
        double leftPower  = drive + turn;
        double rightPower = drive - turn;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower  /= max;
            rightPower /= max;
        }

        backLeft.setPower(leftPower * DRIVE_SPEED);
        backRight.setPower(rightPower * DRIVE_SPEED);
    }

    public void driveStraight(double power) {
        backLeft.setPower(power);
        backRight.setPower(power);
    }


    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public double getIntakePower() {
        return intake.getPower();
    }


    public void setLauncherPower(double power) {
        launcher.setPower(power);
    }

    public double getLauncherPower() {
        return launcher.getPower();
    }


    public void openServos() {
        setServoPositions(SERVO_OPEN);
    }

    public void closeServos() {
        setServoPositions(SERVO_CLOSED);
    }

    public void setServoPositions(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }


    public void stop() {
        backLeft.setPower(0);
        backRight.setPower(0);
        intake.setPower(0);
        launcher.setPower(0);
    }
}
