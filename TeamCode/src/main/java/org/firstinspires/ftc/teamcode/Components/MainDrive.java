package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MainDrive {
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

    private final OpMode opMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    double direction = 1;

    float y;
    float x;
    float rx;

    int divideAmount = 1;

    public MainDrive(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;

        initMotors();
    }

    public void checkInputs(float y,
                            float x,
                            float rx,
                            boolean forwardButton,
                            boolean reverseButton,
                            boolean halfSpeedButton,
                            boolean fourthSpeedButton
    ) {
        this.y = y;
        this.x = x;
        this.rx = rx;

        if (forwardButton) {
            direction = 1;
            setMotorDirections();
        } else if (reverseButton) {
            direction = -1;
            setMotorDirections();
        }

        // Slow speeds
        if (halfSpeedButton) {
            divideAmount = 2;
        } else if (fourthSpeedButton) {
            divideAmount = 4;
        } else {
            divideAmount = 1;
        }
        checkSpeed();

        updateMotors();
    }

    void initMotors() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
    }

    void setMotorDirections() {
        if (direction == 1) { // forward
            frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            backLeft.setDirection(DcMotorEx.Direction.REVERSE);
            frontRight.setDirection(DcMotorEx.Direction.FORWARD);
            backRight.setDirection(DcMotorEx.Direction.FORWARD);
        } else if (direction == -1) { // reversed
            frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            backLeft.setDirection(DcMotorEx.Direction.FORWARD);
            frontRight.setDirection(DcMotorEx.Direction.REVERSE);
            backRight.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }

    void checkSpeed() {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + (direction * rx)) / denominator;
        backLeftPower = (y - x + (direction * rx)) / denominator;
        frontRightPower = (y - x - (direction * rx)) / denominator;
        backRightPower = (y + x - (direction * rx)) / denominator;
    }

    void updateMotors() {
        frontLeft.setPower(frontLeftPower / divideAmount);
        backLeft.setPower(backLeftPower / divideAmount);
        frontRight.setPower(frontRightPower / divideAmount);
        backRight.setPower(backRightPower / divideAmount);
    }
}