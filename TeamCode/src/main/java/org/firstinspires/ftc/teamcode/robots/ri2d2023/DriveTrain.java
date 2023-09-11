package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class DriveTrain {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackRight = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // mecanum types
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
    // Number variables
    private static final float DEADZONE = .1f;
    double robotSpeed = 1;
    public DriveTrain(Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void resetMotors() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setPower(1);
        motorBackLeft.setPower(1);
        motorBackRight.setPower(1);
        motorFrontRight.setPower(1);
        mechanumDrive(0, 0, 0);
    }
    public void tankDrive(double left, double right) {
        powerRight = 0;
        powerLeft = 0;
        if (Math.abs(left) > DEADZONE) {
            powerLeft = left;
        }
        if (Math.abs(right) > DEADZONE) {
            powerRight = right;
        }
        motorFrontRight.setPower(powerRight);
        motorFrontLeft.setPower(powerLeft);
        motorBackRight.setPower(powerRight);
        motorBackLeft.setPower(powerLeft);
    }

    public void mechanumDrive(double forward, double strafe, double turn) {
        forward = forward;
        turn = -turn;
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
        double rightX = turn;
        powerFrontLeft = r * Math.cos(robotAngle) - rightX;
        powerFrontRight = r * Math.sin(robotAngle) + rightX;
        powerBackLeft = r * Math.sin(robotAngle) - rightX;
        powerBackRight = r * Math.cos(robotAngle) + rightX;
        motorFrontLeft.setPower(powerFrontLeft*robotSpeed);
        motorFrontRight.setPower(powerFrontRight*robotSpeed);
        motorBackLeft.setPower(powerBackLeft*robotSpeed);
        motorBackRight.setPower(powerBackRight*robotSpeed);
    }
    public void telemetryOutput()
    {
        telemetry.addData("Back Right Position \t", motorBackRight.getCurrentPosition());
        telemetry.addData("Back Left Position \t", motorBackLeft.getCurrentPosition());
        telemetry.addData("Front Right Position \t", motorFrontRight.getCurrentPosition());
        telemetry.addData("Front Left Position \t", motorFrontLeft.getCurrentPosition());
    }
    public void motorInit()
    {
        motorFrontLeft = this.hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public double getMotorAvgPosition(){return (double)(Math.abs(motorFrontLeft.getCurrentPosition())+Math.abs(motorFrontRight.getCurrentPosition())+Math.abs(motorBackLeft.getCurrentPosition())+Math.abs(motorBackRight.getCurrentPosition()))/4.0;}
}
