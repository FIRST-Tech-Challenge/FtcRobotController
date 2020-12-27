package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrivebase {

    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    private static double quarterPi = Math.PI / 4;

    private double _magnitude = 0.0;
    private double _theta = 0.0;
    private double _turnRate = 0.0;
    private double _powerFL = 0.0;
    private double _powerFR = 0.0;
    private double _powerBL = 0.0;
    private double _powerBR = 0.0;
    private double _encoderFL = 0.0;
    private double _encoderFR = 0.0;
    private double _encoderBL = 0.0;
    private double _encoderBR = 0.0;
    private double _gyroAngle = 0.0;
    private boolean _thirdPersonDrive = false;
    private boolean _a = false;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: goBILDA Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void initialize(LinearOpMode op) {
        FL = op.hardwareMap.get(DcMotor.class, "FL");
        FR = op.hardwareMap.get(DcMotor.class, "FR");
        BL = op.hardwareMap.get(DcMotor.class, "BL");
        BR = op.hardwareMap.get(DcMotor.class, "BR");

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }

    public void setGyroAngle(double angle) {
        _gyroAngle = angle;

    }

    public void setThirdPersonDrive(boolean thirdPersonDrive) {
        _thirdPersonDrive = thirdPersonDrive;
    }

    public void startControl() {

    }

    public void readController (Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;

        if (gamepad.a && !_a) {
            _thirdPersonDrive = !_thirdPersonDrive;
        }
        _a = gamepad.a;

        _magnitude = Math.sqrt((x*x)+(y*y));
        _theta = Math.atan2(x, y);
        _turnRate = gamepad.right_stick_x;
    }


    public void calculatePower() {

        _powerFL = _magnitude * Math.sin(_theta + quarterPi) + _turnRate;
        _powerFR = _magnitude * Math.cos(_theta + quarterPi) - _turnRate;
        _powerBL = _magnitude * Math.cos(_theta + quarterPi) + _turnRate;
        _powerBR = _magnitude * Math.sin(_theta + quarterPi) - _turnRate;

        double scale = Math.max(Math.max(Math.abs(_powerFL), Math.abs(_powerFR)),
                Math.max(Math.abs(_powerBL), Math.abs(_powerBR)));

        if (scale > 1.0) {
            _powerFL /= scale;
            _powerFR /= scale;
            _powerBL /= scale;
            _powerBR /= scale;
        }
    }


    public void calculatePower2() {

        _powerFL = _magnitude * Math.sin(_theta + quarterPi);
        _powerFR = _magnitude * Math.cos(_theta + quarterPi);
        _powerBL = _magnitude * Math.cos(_theta + quarterPi);
        _powerBR = _magnitude * Math.sin(_theta + quarterPi);

        double maxPower = Math.max(Math.max(Math.abs(_powerFL), Math.abs(_powerFR)),
                Math.max(Math.abs(_powerBL), Math.abs(_powerBR)));

        double driveScale = 1.0;
        if (maxPower > 0.0) {
            driveScale = _magnitude / maxPower;
        }

        _powerFL = (driveScale * _powerFL) + _turnRate;
        _powerFR = (driveScale * _powerFR) - _turnRate;
        _powerBL = (driveScale * _powerBL) + _turnRate;
        _powerBR = (driveScale * _powerBR) - _turnRate;

        double scale = Math.max(Math.max(Math.abs(_powerFL), Math.abs(_powerFR)),
                Math.max(Math.abs(_powerBL), Math.abs(_powerBR)));

        if (scale > 1.0) {
            _powerFL /= scale;
            _powerFR /= scale;
            _powerBL /= scale;
            _powerBR /= scale;
        }
    }

    public void calculatePower3() {
        // Below is like an if-else (if false, then add both values; if true, then only theta)
        double controlAngle = ((_thirdPersonDrive) ? _theta  + _gyroAngle : _theta);
        _powerFL = _magnitude * Math.sin(controlAngle + quarterPi);
        _powerFR = _magnitude * Math.cos(controlAngle + quarterPi);
        _powerBL = _magnitude * Math.cos(controlAngle + quarterPi);
        _powerBR = _magnitude * Math.sin(controlAngle + quarterPi);

        double maxPower = Math.max(Math.max(Math.abs(_powerFL), Math.abs(_powerFR)),
                Math.max(Math.abs(_powerBL), Math.abs(_powerBR)));

        double driveScale = 1.0;
        if (maxPower > 0.0) {
            driveScale = _magnitude / maxPower;
        }

        _powerFL = (driveScale * _powerFL) + _turnRate;
        _powerFR = (driveScale * _powerFR) - _turnRate;
        _powerBL = (driveScale * _powerBL) + _turnRate;
        _powerBR = (driveScale * _powerBR) - _turnRate;

        double scale = Math.max(Math.max(Math.abs(_powerFL), Math.abs(_powerFR)),
                Math.max(Math.abs(_powerBL), Math.abs(_powerBR)));

        if (scale > 1.0) {
            _powerFL /= scale;
            _powerFR /= scale;
            _powerBL /= scale;
            _powerBR /= scale;
        }
    }

    public void driveStraight (boolean forward, double distance, double power) {
        double newDistance = COUNTS_PER_INCH * distance;
        resetEncoders();
        if (forward == true) {
            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);
            FR.setDirection(DcMotorSimple.Direction.FORWARD);
            BR.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            BL.setDirection(DcMotorSimple.Direction.FORWARD);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        /* FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FR.getCurrentPosition();
        FL.getCurrentPosition();
        BR.getCurrentPosition();
        BL.getCurrentPosition(); */

        FR.setTargetPosition((int) newDistance);
        FL.setTargetPosition((int) newDistance);
        BR.setTargetPosition((int) newDistance);
        BL.setTargetPosition((int) newDistance);

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FR.isBusy() && FL.isBusy()) /*((FR.getCurrentPosition() < newDistance) && (FL.getCurrentPosition() < newDistance))*/ {
            FR.setPower(power);
            FL.setPower(power);
            BR.setPower(power);
            BL.setPower(power);

            /* FR.getCurrentPosition();
            FL.getCurrentPosition();
            BR.getCurrentPosition();
            BL.getCurrentPosition(); */
        }
    }



    public void driveStrafe (boolean right, double distance, double power) {
        double newDistance = COUNTS_PER_INCH * distance;
        resetEncoders();
        if (right == true) {
            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.FORWARD);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);
            BR.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);
            FR.setDirection(DcMotorSimple.Direction.FORWARD);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        FR.setTargetPosition((int) (newDistance));
        FL.setTargetPosition((int) (newDistance));
        BR.setTargetPosition((int) (newDistance));
        BL.setTargetPosition((int) (newDistance));

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FR.isBusy() && BR.isBusy()) {
            FR.setPower(power);
            FL.setPower(power);
            BR.setPower(power);
            BL.setPower(power);
        }
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad1);
        // this.calculatePower2();
        this.calculatePower3();

        FL.setPower(_powerFL);
        FR.setPower(_powerFR);
        BL.setPower(_powerBL);
        BR.setPower(_powerBR);

    }

    public void drivebaseEncoders(Telemetry telemetry) {
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _encoderFR = FR.getCurrentPosition() / COUNTS_PER_INCH;
        _encoderFL = FL.getCurrentPosition() / COUNTS_PER_INCH;
        _encoderBR = BR.getCurrentPosition() / COUNTS_PER_INCH;
        _encoderBL = BL.getCurrentPosition() / COUNTS_PER_INCH;

        telemetry.addData("FL Inches", "%.03f", _encoderFL);
        telemetry.addData("FR Inches", "%.03f", _encoderFR);
        telemetry.addData("BL Inches", "%.03f", _encoderBL);
        telemetry.addData("BR Inches", "%.03f", _encoderBR);
    }

    public void resetEncoders () {
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("FL Power", "%.03f %%", _powerFL);
        telemetry.addData("FR Power", "%.03f %%", _powerFR);
        telemetry.addData("BL Power", "%.03f %%", _powerBL);
        telemetry.addData("BR Power", "%.03f %%", _powerBR);
    }

    public void stop () {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }
}